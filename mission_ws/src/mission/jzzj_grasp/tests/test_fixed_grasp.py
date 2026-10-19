"""Fixed-angle execution, visual gate and actual driver transport. No hardware."""
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import types
import unittest
from unittest.mock import patch
import test_stage_c
from test_stage_c import ROOT
from jzzj_grasp.core import Observation,Failure
from jzzj_grasp.fixed_grasp import (REAR_DEG,OPEN_DEG,CLOSE_DEG,SIGNS,OFFSETS_DEG,
                                    ReferenceGate,FixedSequence,validate_reference,calibration_digest)
from jzzj_grasp.fixed_grasp_ros import FixedArm,FixedRos

M=types.SimpleNamespace


def config():
    cfg=json.loads((ROOT/'deploy/stage_c_fixed_grasp.example.yaml').read_text())
    cfg.update(reference_position_m=[-.2239400495909601,.06185350241391599,-.05],
               reference_frame=cfg['arm_frame'],identity=dict(class_id=1,label='lemon',color='yellow'),
               calibration_snapshot={'camera':'measured'})
    cfg['calibration_sha256']=calibration_digest(cfg['calibration_snapshot'])
    return cfg


def target(stamp=2.,point=None,track=1):
    return Observation(stamp,'s','arm_left/arm_base_link',tuple(point or config()['reference_position_m']),
                       'yellow','lemon',1,track,1.,True)


class Adapter:
    def __init__(self):self.events=[];self.fail=None;self.hold_ok=True
    def ready(self):self.events.append('ready')
    def validate_all(self,poses):self.events.append(('validate',tuple(poses)))
    def guard(self):
        if self.fail=='cancel' and ('move',OPEN_DEG) in self.events:raise Failure('CANCELED','operator')
    def move(self,q):
        self.events.append(('move',q))
        if self.fail==q:raise Failure('MOTION_FAILED','arrival timeout')
    def wait_reference(self):
        self.events.append('vision')
        if self.fail=='vision':raise Failure('NO_TARGET','out of reference region')
        return config()['reference_position_m']
    def dwell(self,s):self.guard();self.events.append(('wait',s))
    def hold(self):self.events.append('hold');return self.hold_ok


class FixedSequenceTests(unittest.TestCase):
    def test_exact_sequence_runs_once_and_never_reports_grasp_success(self):
        io=Adapter();result=FixedSequence(io,config()).run()
        self.assertTrue(result['motion_complete'],result)
        self.assertFalse(result['grasp_confirmed']);self.assertEqual(result['human_verdict'],'pending')
        self.assertEqual(io.events,['ready',('validate',(REAR_DEG,OPEN_DEG,CLOSE_DEG)),
                                  ('move',REAR_DEG),'vision',('move',OPEN_DEG),('wait',3.),('move',CLOSE_DEG),'hold'])
        self.assertEqual(OPEN_DEG[:5],CLOSE_DEG[:5]);self.assertEqual((OPEN_DEG[5],CLOSE_DEG[5]),(60,20))

    def test_vision_and_approach_failure_cancel_and_stop_failure_cannot_advance(self):
        for fault in ('vision',OPEN_DEG,'cancel'):
            io=Adapter();io.fail=fault;r=FixedSequence(io,config()).run()
            self.assertFalse(r['motion_complete']);self.assertNotIn(('move',CLOSE_DEG),io.events)
            self.assertEqual(io.events[-1],'hold')
        io=Adapter();io.fail=OPEN_DEG;io.hold_ok=False
        self.assertEqual(FixedSequence(io,config()).run()['status'],'STOP_FAILED')

    def test_bad_reference_angles_and_calibration_fail_before_motion(self):
        for change in (lambda c:c.update(reference_position_m=None),lambda c:c.update(calibration_sha256='wrong'),
                       lambda c:c.update(grasp_close_angles_deg=[-1,56,58,50,72,-20]),
                       lambda c:c.update(wait_before_close_seconds=0),lambda c:c.update(trigger_radius_m=.021)):
            c=config();change(c);io=Adapter();r=FixedSequence(io,c).run()
            self.assertFalse(r['motion_complete']);self.assertEqual(io.events,[])


class VisualGateTests(unittest.TestCase):
    def test_requires_three_distinct_fresh_frames_over_one_second_inside_total_20mm(self):
        gate=ReferenceGate(config(),'s',1.)
        for s in (2.,2.3,2.6,2.9):self.assertIsNone(gate.add(s,'s','ok',[target(s)],s+.02))
        self.assertIsNotNone(gate.add(3.1,'s','ok',[target(3.1)],3.12))
        gate=ReferenceGate(config(),'s',1.);p=list(config()['reference_position_m']);p[0]+=.015;p[1]+=.015
        for s in (2.,2.4,2.8,3.2):self.assertIsNone(gate.add(s,'s','ok',[target(s,p)],s+.02))

    def test_20mm_radius_accepts_19mm_rejects_21mm_and_retains_5mm_stability(self):
        centre=config()['reference_position_m']
        for offset,accepted in ((.019,True),(.021,False)):
            gate=ReferenceGate(config(),'s',1.);point=list(centre);point[0]+=offset
            result=None
            for s in (2.,2.3,2.6,2.9,3.1):result=gate.add(s,'s','ok',[target(s,point)],s+.02)
            self.assertEqual(result is not None,accepted)
        gate=ReferenceGate(config(),'s',1.)
        for i,s in enumerate((2.,2.3,2.6,2.9,3.1)):
            point=list(centre);point[0]+=.01 if i%2 else -.01
            self.assertIsNone(gate.add(s,'s','ok',[target(s,point)],s+.02))

    def test_old_5mm_reference_retains_its_limit(self):
        cfg=config();cfg['trigger_radius_m']=.005
        gate=ReferenceGate(cfg,'s',1.);point=list(cfg['reference_position_m']);point[0]+=.006
        for s in (2.,2.3,2.6,2.9,3.1):
            self.assertIsNone(gate.add(s,'s','ok',[target(s,point)],s+.02))

    def test_empty_old_duplicate_and_stale_frames_do_not_complete_window(self):
        gate=ReferenceGate(config(),'s',1.)
        self.assertIsNone(gate.add(2.,'old','ok',[target(2.)],2.02))
        gate.add(2.,'s','ok',[target(2.)],2.02)
        self.assertIsNone(gate.add(2.,'s','ok',[target(2.)],2.1))
        gate.add(2.3,'s','ok',[],2.32);self.assertEqual(gate.samples,[])
        gate.add(2.5,'s','ok',[target(2.5)],3.5);self.assertEqual(gate.samples,[])
        gate.add(3.,'s','ok',[target(3.)],3.02)
        self.assertIsNone(gate.add(3.7,'s','ok',[target(3.7)],3.72))
        self.assertEqual(len(gate.samples),1)

    def test_ambiguity_track_switch_and_sensor_failure_stop(self):
        for mode in ('ambiguous','track','sensor'):
            gate=ReferenceGate(config(),'s',1.);gate.add(2.,'s','ok',[target()],2.02)
            with self.assertRaises(Failure):
                gate.add(2.2,'s','failed' if mode=='sensor' else 'ok',
                         [target(2.2),target(2.2,track=2)] if mode=='ambiguous' else [target(2.2,track=2 if mode=='track' else 1)],2.22)


class TransportTests(unittest.TestCase):
    def test_preflight_uses_driver_raw_derived_limits_without_importing_arm_scripts(self):
        from servo_protocol import joint_limits
        lo,hi=joint_limits(SIGNS,OFFSETS_DEG,96,4000,left=True)
        params={'/arm_left/arm_trajectory/joint_lower_rad':lo,'/arm_left/arm_trajectory/joint_upper_rad':hi}
        io=FixedRos.__new__(FixedRos);io.rospy=M(get_param=lambda k:params[k])
        io.validate_all((REAR_DEG,OPEN_DEG,CLOSE_DEG))
        bad=list(OPEN_DEG);bad[4]=-200
        with self.assertRaises(Failure):io.validate_all((bad,))

    def test_release_requires_own_session_idle_action_and_measured_hold(self):
        for mode in ('wrong','active','hold_failed','ok'):
            params={'/stage_c/fixed_grasp/session':'other' if mode=='wrong' else 'fixed',
                    '/stage_c/taught/recovery_required':True}
            io=FixedRos.__new__(FixedRos)
            io.rospy=M(get_param=lambda k,d:params.get(k,d),set_param=lambda k,v:params.__setitem__(k,v),
                       wait_for_message=lambda *a,**kw:M(status_list=[M(status=1)] if mode=='active' else []))
            io.guard=lambda:None;io.check_graph=lambda **kw:None;io.hold=lambda:mode!='hold_failed'
            with patch.dict(sys.modules,{'actionlib_msgs.msg':M(GoalStatusArray=object)}):
                if mode=='ok':
                    io.release('fixed');self.assertFalse(params['/stage_c/taught/recovery_required'])
                else:
                    with self.assertRaises(Failure):io.release('fixed')
                    self.assertTrue(params['/stage_c/taught/recovery_required'])

    def test_actual_adapter_sends_exact_hardcoded_radians_without_ik(self):
        io=FixedRos.__new__(FixedRos);io.session='fixed-test';events=[];q=[]
        io.guard=lambda:None;io.rospy=M(set_param=lambda k,v:events.append((k,v)))
        def trajectory(values,guard,seconds):q[:]=values;events.append(('trajectory',list(values),seconds))
        io.arm=M(trajectory=trajectory,measured=lambda:list(q))
        io.move(OPEN_DEG);io.move(CLOSE_DEG)
        goals=[e for e in events if e[0]=='trajectory']
        self.assertEqual(goals[0][1],[math.radians(v) for v in OPEN_DEG])
        self.assertEqual(goals[1][1],[math.radians(v) for v in CLOSE_DEG])
        self.assertEqual(goals[0][1][:5],goals[1][1][:5]);self.assertEqual(goals[1][2],3.)

    def test_ready_requires_exact_mapping_but_no_ik_service(self):
        import time
        io=FixedRos.__new__(FixedRos);io.cfg=config();io.session='s';io.lock=threading.RLock()
        params={}
        for k,v in dict(hardware_enabled=True,calibration_confirmed=True,workspace_clear_confirmed=True,
                        servo_signs=list(SIGNS),servo_offsets_deg=list(OFFSETS_DEG),servo_position_min=96,
                        servo_position_max=4000,servo_write_mode='individual').items():
            params['/arm_left/arm_trajectory/'+k]=v
        io.rospy=M(get_param=lambda k,d:params.get(k,d),Duration=lambda x:x,Time=M(now=lambda:M(to_sec=lambda:100.)))
        io.check_graph=lambda **kw:None;io.guard=lambda:None;io.calibration=lambda:io.cfg['calibration_snapshot']
        io.select=lambda enabled:None;io.arm=M(client=M(wait_for_server=lambda d:True))
        io.last_received=(time.monotonic(),(100.,'s','ok',[]))
        io.ready()  # no wait_for_service or IK transport exists on these fakes.
        params['/arm_left/arm_trajectory/servo_signs']=[1,-1,1,1,1,-1]
        with self.assertRaisesRegex(Failure,'mapping mismatch'):io.ready()

    def test_inverse_and_cartesian_interfaces_are_forbidden(self):
        arm=FixedArm.__new__(FixedArm)
        for method in ('solve','planner'):
            with self.assertRaises(Failure):getattr(arm,method)()

    def test_actual_driver_individual_mode_preserves_legacy_second_and_claw_signs(self):
        import test_ros_contracts as contracts
        from checked_arm_sdk import CheckedArmDevice
        from test_control_repair import SwallowingVendor
        contracts.DriverContractTests.setUpClass()
        driver=contracts.DriverContractTests.module.TrajectoryDriver.__new__(contracts.DriverContractTests.module.TrajectoryDriver)
        driver.names=['joint'+str(i) for i in range(1,7)];driver.lower=[-10.]*6;driver.upper=[10.]*6
        driver.signs=SIGNS;driver.offsets=OFFSETS_DEG;driver.io=threading.RLock();driver.write_mode='individual'
        driver.device=CheckedArmDevice(lr='left',factory=SwallowingVendor)
        with patch('checked_arm_sdk.time.sleep'):
            report=driver.write([math.radians(x) for x in OPEN_DEG],3.)
            self.assertTrue(report.complete)
            calls=driver.device.device.bus.bus.calls
            self.assertEqual([call[1] for call in calls],[0x19]*6)
            self.assertEqual([call[2][0] for call in calls],list(range(1,7)))
            from servo_protocol import angle_to_position
            self.assertEqual((calls[1][2][1]<<8)|calls[1][2][2],angle_to_position(2,-53.,True))
            self.assertEqual((calls[5][2][1]<<8)|calls[5][2][2],angle_to_position(6,60.,True))
            q=[math.radians(x) for x in CLOSE_DEG];q[4]=math.radians(-200)
            report=driver.write(q,3.);self.assertEqual(report.skipped_ids,(5,))

    def test_graph_rejects_legacy_and_base_publishers(self):
        io=FixedRos.__new__(FixedRos);io.graph_checked=None;io.rospy=M(get_name=lambda:'/fixed')
        clean=[('/arm_left/joint_states',['/arm_left/arm_trajectory']),('/stage_c/observations',['/observer'])]
        state=[clean,[],[]]
        with patch.dict(sys.modules,{'rosgraph':M(Master=lambda n:M(getSystemState=lambda:state))}):
            io.check_graph(True)
            state[:]=[clean+[('/arm_left/arm_joint_states',['/arm_left/arm_test_node'])],[],[]]
            with self.assertRaises(Failure):io.check_graph(True)
            state[:]=[clean,[('/cmd_vel',['/chassis'])],[]]
            with self.assertRaises(Failure):io.check_graph(True)


class CliTests(unittest.TestCase):
    def test_check_is_offline_and_real_run_requires_explicit_gates(self):
        import tempfile
        script=ROOT/'mission_ws/src/mission/jzzj_grasp/scripts/stage_c_fixed_grasp.py'
        env=dict(os.environ,PYTHONPATH=os.pathsep.join(str(ROOT/p) for p in
                 ('mission_ws/src/mission/jzzj_grasp/src','mission_ws/src/mission/jzzj_visual_servo/src')))
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'reference.json';path.write_text(json.dumps(config()))
            def run(*args):return subprocess.run([sys.executable,'-B',str(script),*args,'--reference',str(path)],env=env,capture_output=True,text=True)
            r=run('check');self.assertEqual(r.returncode,0,r.stderr)
            r=run('run');self.assertIn('--confirm-base-immobilized',r.stderr)
            r=run('run','--real','--confirm-base-immobilized','--confirm-fixed-path-clear')
            self.assertIn('real_motion_enabled is false',r.stderr)
            self.assertNotIn('No module named',r.stderr)


if __name__=='__main__':unittest.main()
