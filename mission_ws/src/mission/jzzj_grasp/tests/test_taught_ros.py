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
