"""Actual taught adapter boundary checks, without ROS master or actuators."""
import copy
import os
from pathlib import Path
import tempfile
import sys
import types
import threading
import subprocess
import unittest
from unittest.mock import patch
from types import SimpleNamespace as M
from test_stage_c import Harness
from test_taught import template
from jzzj_grasp.core import Failure
from jzzj_grasp.taught import split_sequence
from jzzj_grasp.taught_ros import RosAdapter,TaughtArm,BaseClient


class TaughtAdapterTests(unittest.TestCase):
    def adapter(self):
        a=RosAdapter.__new__(RosAdapter);a.cfg=copy.deepcopy(Harness().cfg);a.cfg['active_profile']='tabletop'
        a.cfg['workspace_min']=[-1,-1,.005];a.cfg['workspace_max']=[1,1,1]
        a.cfg['profiles']['tabletop']['table_plane_offset_m']=0
        a.guard=lambda:None;self.q=template()['start_rad'];self.calls=[]
        a.rospy=M(get_param=lambda k:[-2.]*6 if 'lower' in k else [2.]*6,
                  set_param=lambda *a:None,Time=M(now=lambda:M(to_sec=lambda:1.)))
        def solve(p,r,g,forward=False,seed=None):
            self.assertTrue(forward);self.assertIsNone(p);self.assertIsNone(r);self.calls.append(seed.position)
            x,y,z=[v*.5 for v in seed.position[:3]]
            return M(tcp_pose=M(pose=M(position=M(x=x,y=y,z=z),orientation=M(x=0,y=0,z=0,w=1))))
        a.arm=M(measured=lambda:tuple(self.q),fresh_joints=lambda:M(name=list('123456'),position=list(self.q)),solve=solve,
                trajectory=lambda q,g,s:self.q.__setitem__(slice(None),q))
        a.base=M(guard=lambda:None,fresh_status=lambda:M(phase='LOCKED'))
        return a

    def test_full_preflight_and_execution_use_forward_only_and_keep_gripper_separate(self):
        a=self.adapter();steps=split_sequence(template());a.preflight(steps)
        self.assertGreater(len(self.calls),len(steps))
        for step in steps:a.execute(step)
        self.assertEqual(self.q[5],.5)

    def test_out_of_range_point_rejected_before_any_command(self):
        a=self.adapter();steps=split_sequence(template());steps[0]['positions_rad']=(0,0,.2,0,0,4)
        with self.assertRaises(Failure):a.preflight(steps)
        self.assertEqual(self.q,template()['start_rad'])

    def test_inverse_call_is_explicitly_forbidden(self):
        b=TaughtArm.__new__(TaughtArm)
        for forward,endpoint in ((False,False),(True,True)):
            with self.assertRaisesRegex(Failure,'inverse kinematics forbidden'):
                b.solve((.1,0,0),(0,0,0,1),lambda:None,forward=forward,endpoint=endpoint)

    def test_arm_only_executes_with_no_base_client_and_records_recovery_before_sending(self):
        a=self.adapter();a.arm_only=True;a.base=None;a.session='arm-trial';events=[]
        a.guard=lambda:events.append('guard')
        a.rospy.set_param=lambda k,v:events.append((k,v))
        steps=split_sequence(template());a.preflight(steps)
        for step in steps:a.execute(step)
        self.assertEqual(self.q[5],.5)
        self.assertEqual(events.count(('/stage_c/taught/recovery_required',True)),len(steps))
        self.assertTrue(self.calls)  # adapter() asserts forward_only on every solve.

    def test_arm_only_bindings_do_not_require_camera_or_fruit(self):
        a=RosAdapter.__new__(RosAdapter);a.arm_only=True;a.cfg=Harness().cfg
        with tempfile.TemporaryDirectory() as tmp:
            model=Path(tmp)/'arm.urdf';model.write_text('<robot/>')
            params={'/stage_c_kinematics/urdf':str(model),'/stage_c_kinematics/tcp_offset_m':[0.,0.,.1]}
            for key,value in dict(servo_signs=[1]*6,servo_offsets_deg=[0]*6,
                                  servo_position_min=96,servo_position_max=4000).items():
                params['/arm_left/arm_trajectory/'+key]=value
            a.rospy=M(get_param=lambda k:params[k])
            self.assertEqual(set(a.bindings()),{'table','tcp_model','servo'})

    def test_base_unlock_or_partial_trajectory_failure_never_advances(self):
        a=self.adapter();step=split_sequence(template())[0];a.preflight([step])
        a.base.fresh_status=lambda:M(phase='ALIGN')
        with self.assertRaisesRegex(Failure,'base not locked'):a.execute(step)
        a.base.fresh_status=lambda:M(phase='LOCKED')
        def partial(*args):raise Failure('MOTION_FAILED','partial command: skipped (5,)')
        a.arm.trajectory=partial
        with self.assertRaisesRegex(Failure,'partial command'):a.execute(step)

    def test_stale_visual_frames_during_pulse_immediately_issue_hold(self):
        a=RosAdapter.__new__(RosAdapter);events=[]
        a.base=M(switch=lambda *args:events.append('move'))
        a.guard=lambda:None;a.hold_base=lambda:events.append('hold')
        a.frame_lock=threading.RLock();a.last_frame=(1.,(1.,'s','ok',[]))
        a.rospy=M(Time=M(now=lambda:M(to_sec=lambda:10.)))
        a.session='s';a.cfg={'arm_frame':'arm'};a.template=template()
        with patch('jzzj_grasp.taught_ros.time.monotonic',return_value=1.):
            with self.assertRaises(Failure):a.pulse(0,.02,.5)
        self.assertEqual(events,['move','hold'])


class BaseTransportTests(unittest.TestCase):
    def test_actual_client_protocol_acknowledges_fault_stop_and_explicit_release(self):
        from test_taught_base import lease
        state=lease();clock=[1.];holder={}
        class Stamp:
            def __init__(self,value):self.value=value
            def to_sec(self):return self.value
            def __sub__(self,other):return Stamp(self.value-other.value)
        def publish(msg):
            try:state.accept(msg.session_id,msg.command_id,msg.operation,msg.vx,msg.vy,clock[0],True,msg.operator_confirmed)
            except ValueError as exc:state.fault(str(exc))
            holder['client'].callback(M(header=M(stamp=Stamp(clock[0])),session_id=state.owner,
                 command_id=state.command_id,phase=state.phase,serial_stop_written=state.velocity==(0.,0.),diagnostic=state.diagnostic))
        rospy=M(Publisher=lambda *a,**k:M(publish=publish,get_num_connections=lambda:1),
                Subscriber=lambda *a,**k:None,Duration=lambda v:v,Time=M(now=lambda:Stamp(clock[0])),
                Timer=lambda *a:M(shutdown=lambda:None))
        messages=types.ModuleType('user_msgs.msg')
        messages.StageCBaseCommand=lambda:M(header=M());messages.StageCBaseStatus=M
        with patch.dict(sys.modules,{'user_msgs.msg':messages}), \
             patch('jzzj_grasp.taught_ros.time.monotonic',side_effect=lambda:clock[0]):
            client=BaseClient(rospy,'s',threading.Event());holder['client']=client
            client.acquire();client.switch(1,(.02,0));self.assertEqual(state.velocity,(.02,0))
            clock[0]=1.5;state.tick(clock[0]);self.assertEqual(state.phase,'FAULT')
            self.assertTrue(client.stop());self.assertEqual(state.phase,'FAULT')
            client.release();self.assertEqual(state.phase,'IDLE');self.assertEqual(state.velocity,(0,0))
            client.close()


class LeaseAndPackagingTests(unittest.TestCase):
    def test_arm_only_teach_records_measured_start_and_explicit_jog_without_base_client(self):
        from stage_c_taught import interactive
        t=template();t.update(execution_mode='arm_only',start_rad=[],steps=[],bindings={})
        q=[0.,0.,.2,0.,0.,.1];events=[]
        def execute(step):q[:]=step['positions_rad'];events.append('execute')
        io=M(arm_only=True,ready=lambda:None,wait_arm_still=lambda:None,
             arm=M(measured=lambda:tuple(q),fresh_joints=lambda:M(header=M(stamp=M(to_sec=lambda:123.)))),
             check_start=lambda q:None,select=lambda:None,guard=lambda:None,
             bindings=lambda:{'servo':'measured'},check_base_isolated=lambda **kw:events.append('isolated'),
             preflight=lambda steps:events.append('preflight'),execute=execute)
        with tempfile.TemporaryDirectory() as tmp, patch('builtins.input',side_effect=[
                'start','jog 6 1','record open 0.8','quit']),patch('stage_c_taught.time.sleep'):
            interactive(io,t,str(Path(tmp)/'trial.json'))
        self.assertEqual(t['bindings'],{'servo':'measured'})
        self.assertEqual(t['start_rad'],[0.,0.,.2,0.,0.,.1])
        self.assertEqual(t['steps'][0]['positions_rad'],q)
        self.assertEqual(events,['isolated','preflight','execute'])

    def test_arm_only_release_requires_own_session_and_measured_hold(self):
        from stage_c_taught import release_arm_only
        for owner,hold in (('',True),('other',True),('trial',False),('trial',True)):
            params={'/stage_c/taught/arm_only_session':owner,'/stage_c/taught/recovery_required':True}
            events=[]
            io=M(rospy=M(get_param=lambda k,d:params.get(k,d),set_param=lambda k,v:params.__setitem__(k,v)),
                 wait_arm_still=lambda:events.append('still'),hold_arm=lambda:hold)
            if owner!='trial' or not hold:
                with self.assertRaises(RuntimeError):release_arm_only(io,'trial')
                self.assertTrue(params['/stage_c/taught/recovery_required'])
            else:
                release_arm_only(io,'trial')
                self.assertFalse(params['/stage_c/taught/recovery_required'])
                self.assertEqual(events,['still','still'])

    def test_arm_only_cli_requires_matching_template_physical_confirmation_and_no_base_calibration(self):
        import json
        from test_stage_c import ROOT
        script=ROOT/'mission_ws/src/mission/jzzj_grasp/scripts/stage_c_taught.py'
        env=dict(os.environ,PYTHONPATH=os.pathsep.join(str(ROOT/p) for p in
                 ('mission_ws/src/mission/jzzj_grasp/src','mission_ws/src/mission/jzzj_visual_servo/src')))
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'arm-only.yaml'
            def run(*args):return subprocess.run([sys.executable,'-B',str(script),*args,'--template',str(path)],
                                               env=env,capture_output=True,text=True)
            r=run('init','--arm-only','--example',str(ROOT/'deploy/stage_c_taught.example.yaml'))
            self.assertEqual(r.returncode,0,r.stderr)
            t=json.loads(path.read_text());self.assertEqual(t['execution_mode'],'arm_only')
            self.assertFalse(t['base_calibrated']);self.assertFalse(t['real_motion_enabled'])
            self.assertEqual(t['steps'],[])
            r=run('run','--real','--config','unused')
            self.assertIn('mode mismatch',r.stderr)
            r=run('calibrate-base','--arm-only')
            self.assertIn('forbidden',r.stderr)
            r=run('teach','--arm-only','--real','--config','unused')
            self.assertIn('confirm-base-immobilized',r.stderr)
            r=run('teach','--arm-only','--real','--config','unused','--confirm-base-immobilized')
            self.assertIn('real_motion_enabled',r.stderr)
            self.assertNotIn('No module named',r.stderr)

    def test_arm_only_graph_guard_rejects_running_or_reappearing_base_controllers(self):
        a=RosAdapter.__new__(RosAdapter);a.isolation_checked=None
        a.rospy=M(get_name=lambda:'/arm_only_test')
        clean=[('/arm_left/joint_states',['/arm_left/arm_trajectory'])]
        state=[clean,[],[]]
        with patch.dict(sys.modules,{'rosgraph':M(Master=lambda n:M(getSystemState=lambda:state))}):
            a.check_base_isolated(force=True)
            for topic in ('/cmd_vel','/car_cmd','/stage_c/base/command'):
                state[:]=[clean,[(topic,['/chassis'])],[]]
                with self.assertRaisesRegex(Failure,'controllers stopped'):a.check_base_isolated(force=True)
                state[:]=[clean+[(topic,['/teleop'])],[],[]]
                with self.assertRaises(Failure):a.check_base_isolated(force=True)
            state[:]=[clean+[('/stage_c/base/status',['/chassis'])],[],[]]
            with self.assertRaises(Failure):a.check_base_isolated(force=True)
            state[:]=[[('/arm_left/joint_states',['/arm_left/arm_trajectory','/legacy_arm'])],[],[]]
            with self.assertRaises(Failure):a.check_base_isolated(force=True)

    def test_arm_only_cannot_acquire_or_move_base(self):
        a=RosAdapter.__new__(RosAdapter);a.arm_only=True;a.base=None
        with self.assertRaisesRegex(Failure,'forbidden'):a.acquire_base()
        with self.assertRaisesRegex(Failure,'forbidden'):a.pulse(0,.02,.5)

    @unittest.skipIf(os.name=='nt','Linux flock exercised in WSL')
    def test_old_and_taught_entry_cannot_hold_execution_lease_together(self):
        from jzzj_grasp.execution_lease import ExecutionLease
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'lease');first=ExecutionLease(path)
            try:
                with self.assertRaises(RuntimeError):ExecutionLease(path)
            finally:first.close()
            second=ExecutionLease(path);second.close()

    def test_disabled_example_contains_no_invented_pose_or_matrix(self):
        import json
        from test_stage_c import ROOT
        t=json.loads((ROOT/'deploy/stage_c_taught.example.yaml').read_text())
        self.assertFalse(t['real_motion_enabled']);self.assertFalse(t['base_calibrated'])
        self.assertEqual(t['steps'],[]);self.assertEqual(t['start_rad'],[]);self.assertIsNone(t['base_matrix'])
        self.assertIsNone(t['reference_position_m'])

    def test_cli_initialization_and_disabled_real_entry_do_not_import_ros_or_move(self):
        from test_stage_c import ROOT
        script=ROOT/'mission_ws/src/mission/jzzj_grasp/scripts/stage_c_taught.py'
        env=dict(os.environ,PYTHONPATH=os.pathsep.join(str(ROOT/p) for p in
                 ('mission_ws/src/mission/jzzj_grasp/src','mission_ws/src/mission/jzzj_visual_servo/src')))
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'taught.yaml')
            result=subprocess.run([sys.executable,'-B',str(script),'init','--template',path,'--example',
                    str(ROOT/'deploy/stage_c_taught.example.yaml')],env=env,capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            result=subprocess.run([sys.executable,'-B',str(script),'run','--template',path,'--real','--config',
                    str(ROOT/'deploy/stage_c.yaml')],env=env,capture_output=True,text=True)
            self.assertEqual(result.returncode,2);self.assertIn('real_motion_enabled',result.stderr)


if __name__=='__main__':unittest.main()
