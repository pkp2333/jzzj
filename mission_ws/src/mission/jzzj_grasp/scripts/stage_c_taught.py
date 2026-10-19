#!/usr/bin/env python3
"""Local template tools and explicit operator-controlled taught C-stage tests."""
import argparse
import copy
import datetime
import json
import math
from pathlib import Path
import signal
import threading
import time
import uuid
from jzzj_grasp.taught import (Coordinator,PHASES,fit_base_matrix,template_hash,
                              validate_template,split_sequence)


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))


def save(path,value):
    p=Path(path);p.parent.mkdir(parents=True,exist_ok=True)
    temporary=p.with_name(p.name+'.tmp');temporary.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    temporary.replace(p)


def release_arm_only(io,session):
    if io.rospy.get_param('/stage_c/taught/arm_only_session','')!=session:
        raise RuntimeError('arm-only recovery session mismatch; cannot release a base-alignment or unknown session')
    io.wait_arm_still()
    if not io.hold_arm():raise RuntimeError('STOP_FAILED: measured arm hold failed; recovery remains latched')
    io.wait_arm_still()
    io.rospy.set_param('/stage_c/taught/arm_only_session','')
    io.rospy.set_param('/stage_c/taught/recovery_required',False)


def interactive(io,template,path,calibrate=False):
    """All movement is explicit operator input, one bounded step at a time."""
    io.deadline=time.monotonic()+1800.  # Manual input pauses do not consume a 6-minute automatic run.
    io.ready();io.wait_arm_still()
    if not template['start_rad']:template['start_rad']=list(io.arm.measured())
    io.check_start(template['start_rad'])
    if not getattr(io,'arm_only',False):io.acquire_base();io.hold_base(locked=not calibrate)
    io.select();time.sleep(.5)
    print('base calibration: pulse x|y +/-metres, fit, quit' if calibrate else
          'teach: target, start, jog ID +/-degrees, record PHASE SECONDS, check, quit',flush=True)
    while True:
        text=input('taught> ').strip().split()
        if not text:continue
        io.guard();command=text[0]
        if command=='quit':break
        if calibrate:
            if command=='pulse' and len(text)==3:
                axis={'x':0,'y':1}[text[1]];delta=float(text[2])
                if not math.isfinite(delta) or not .005<=abs(delta)<=.01:raise ValueError('calibration pulse is 5..10mm')
                io.hold_base();before=io.observe(None,time.monotonic()+5.)
                if io.capture_identity:template['identity']=io.capture_identity
                io.pulse(axis,math.copysign(.02,delta),abs(delta)/.02)
                after=io.observe(None,time.monotonic()+5.)
                actual=float(input('Independently measured signed base displacement (metres): '))
                if not math.isfinite(actual) or not .005<=abs(actual)<=.03 or actual*delta<=0:
                    raise ValueError('invalid independently measured displacement')
                vector=[0.,0.];vector[axis]=actual
                template['base_calibration_samples'].append(dict(base_delta_m=vector,before_m=before,after_m=after))
            elif command=='fit':
                matrix,residual=fit_base_matrix(template['base_calibration_samples'],template['table_normal'])
                template.update(base_matrix=matrix,base_calibrated=True,bindings=io.bindings())
                print('fitted measured matrix, residual(m)=',residual,flush=True)
            else:raise ValueError('use pulse x|y +/-metres, fit, quit')
        elif command=='target':
            io.wait_arm_still();io.after=io.rospy.Time.now().to_sec()
            template['reference_position_m']=list(io.observe(None,time.monotonic()+5.))
            template['reference_observation_stamps']=list(io.last_snapshot_stamps)
            if io.capture_identity:template['identity']=io.capture_identity
            template['bindings']=io.bindings();print('reference',template['reference_position_m'],flush=True)
        elif command=='start':
            if template['steps']:raise ValueError('start must precede recorded steps; create a new template to reteach')
            io.wait_arm_still();template['start_rad']=list(io.arm.measured())
            template['start_feedback_stamp']=io.arm.fresh_joints().header.stamp.to_sec()
            template['bindings']=io.bindings()
        elif command=='jog' and len(text)==3:
            if getattr(io,'arm_only',False):io.check_base_isolated(force=True)
            elif io.base.fresh_status().phase!='LOCKED':raise ValueError('lock base before arm teaching')
            jid=int(text[1]);degrees=float(text[2])
            if not 1<=jid<=6 or not math.isfinite(degrees) or not 0<abs(degrees)<=3:
                raise ValueError('joint 1..6, explicit jog <=3 degrees; default recommendation is 1 degree')
            actual=io.arm.measured();q=list(actual);q[jid-1]+=math.radians(degrees)
            step=dict(phase='open' if jid==6 else 'approach',positions_rad=q,seconds=.8)
            # Build and validate the measured local edge using FK only.
            io.arm_started=True;io.preflight([step]);io.execute(step)
        elif command=='record' and len(text)==3:
            phase=text[1];seconds=float(text[2])
            if phase not in PHASES or not math.isfinite(seconds) or not .2<=seconds<=5:
                raise ValueError('record open|approach|descend|close|lift|transport seconds(0.2..5)')
            io.wait_arm_still();template['steps'].append(dict(phase=phase,positions_rad=list(io.arm.measured()),seconds=seconds,
                    feedback_stamp=io.arm.fresh_joints().header.stamp.to_sec(),recorded_at=datetime.datetime.now().isoformat()))
        elif command=='check':
            validate_template(template,io.bindings(),require_empty=False,arm_only=getattr(io,'arm_only',False));io.preflight(split_sequence(template))
            print('recorded sequence structure/ranges/FK: OK (no motion)',flush=True)
        else:raise ValueError('unknown teaching command')
        template['empty_validation']={};save(path,template)
    save(path,template)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('init','check','fit','approve-empty','confirm','teach','calibrate-base','run','release'))
    parser.add_argument('--template',required=True)
    parser.add_argument('--example');parser.add_argument('--config');parser.add_argument('--log')
    parser.add_argument('--samples');parser.add_argument('--output',default='.local/taught-runs')
    parser.add_argument('--verdict',choices=('success','failure','unknown'))
    parser.add_argument('--real',action='store_true');parser.add_argument('--empty',action='store_true')
    parser.add_argument('--session');parser.add_argument('--confirm-stopped-and-clear',action='store_true')
    parser.add_argument('--arm-only',action='store_true',help='fixed arm sequence; physically immobilized base, no base control')
    parser.add_argument('--confirm-base-immobilized',action='store_true',help='operator confirms base drive disabled and chassis physically secured')
    args=parser.parse_args()
    if args.command=='init':
        if Path(args.template).exists():parser.error('refusing to overwrite existing template')
        if not args.example:parser.error('--example required')
        template=read(args.example)
        if args.arm_only:template['execution_mode']='arm_only'
        save(args.template,template);return
    template=read(args.template)
    if template.get('execution_mode','base_alignment')!=('arm_only' if args.arm_only else 'base_alignment'):
        parser.error('template execution mode mismatch; use a separate template for arm-only tests')
    if args.arm_only and args.command in ('fit','calibrate-base'):
        parser.error('base calibration forbidden in arm-only mode')
    if args.command=='check':
        validate_template(template,require_empty=False,arm_only=args.arm_only);print('template structure OK; runtime bindings/FK still required');return
    if args.command=='fit':
        if not args.samples:parser.error('--samples required')
        samples=read(args.samples);matrix,residual=fit_base_matrix(samples,template['table_normal'])
        template.update(base_matrix=matrix,base_calibration_samples=samples,base_calibrated=True,empty_validation={})
        save(args.template,template);print('residual_m=',residual);return
    if args.command in ('approve-empty','confirm'):
        if not args.log:parser.error('--log required')
        record=read(args.log)
        if record.get('template_hash')!=template_hash(template):parser.error('log belongs to another template revision')
        if args.command=='approve-empty':
            if not record.get('empty') or not record.get('motion_complete') or not record.get('stop_confirmed'):
                parser.error('successful empty execution and measured hold required')
            template['empty_validation']=dict(template_hash=template_hash(template),evidence=str(Path(args.log).resolve()))
            save(args.template,template)
        else:
            if not args.verdict:parser.error('--verdict required')
            record['human_verdict']=args.verdict;record['human_confirmed_at']=datetime.datetime.now().isoformat()
            save(args.log,record)
        return
    if not args.real or not args.config:parser.error('ROS operations require --real and --config; no implicit motion')
    if args.arm_only and not args.confirm_base_immobilized:
        parser.error('arm-only requires --confirm-base-immobilized after disabling base drive and physically securing chassis')
    for gate in ('real_motion_enabled','workspace_clear_confirmed','table_height_confirmed'):
        if template.get(gate) is not True:parser.error('confirm actual field condition in template: '+gate)
    import rospy
    from jzzj_grasp.execution_lease import ExecutionLease
    from jzzj_grasp.taught_ros import RosAdapter
    rospy.init_node('stage_c_taught_operator',anonymous=True,disable_signals=True)
    cancel=threading.Event();signal.signal(signal.SIGINT,lambda *a:cancel.set());rospy.on_shutdown(cancel.set)
    session=args.session or ('taught-'+uuid.uuid4().hex)
    print('Session:',session,flush=True)
    lease=ExecutionLease();io=None;record=None;released=False
    try:
        io=RosAdapter(rospy,read(args.config)['execution'],template,session,cancel,arm_only=args.arm_only)
        if args.command=='release':
            if not args.session or not args.confirm_stopped_and_clear:parser.error('release requires --session and --confirm-stopped-and-clear')
            for flag in ('stop_failed_latched','holding_latched'):
                if rospy.get_param('/stage_c_server/'+flag,False):raise RuntimeError('recover C server '+flag+' before base release')
            from actionlib_msgs.msg import GoalStatusArray
            m=rospy.wait_for_message('/arm_left/follow_joint_trajectory/status',GoalStatusArray,timeout=3.)
            if any(s.status in (0,1,6,7) for s in m.status_list):raise RuntimeError('arm still active; no base release')
            if args.arm_only:
                release_arm_only(io,session)
            else:
                io.wait_arm_still();io.base.release()
                rospy.set_param('/stage_c/taught/recovery_required',False)
            released=True
            return
        if args.command in ('teach','calibrate-base'):
            interactive(io,template,args.template,args.command=='calibrate-base');return
        io.select();time.sleep(.5)
        started=datetime.datetime.now().isoformat();record=Coordinator(io,template,time.monotonic,arm_only=args.arm_only).run(empty=args.empty)
        record.update(session=session,empty=args.empty,started_at=started,finished_at=datetime.datetime.now().isoformat(),
                      bindings=template.get('bindings',{}),template_file=str(Path(args.template).resolve()))
        path=Path(args.output)/(session+'.json');save(path,record)
        print(json.dumps(record,indent=2),flush=True);print('Evidence:',path,flush=True)
        if not record['motion_complete']:raise SystemExit(2)
    finally:
        if io:
            if io.base is not None and io.base.owned:io.stop_base()
            if io.arm.sent and record is None and not released:
                if not io.hold_arm():
                    rospy.set_param('/stage_c/taught/recovery_required',True)
                    io.close();lease.close()
                    raise RuntimeError('STOP_FAILED: arm hold unconfirmed; operator recovery required')
            io.close()
        lease.close()


if __name__=='__main__':main()
