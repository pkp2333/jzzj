"""Exercise ROS adapter callbacks with in-memory transport/message doubles only."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'catkin_ws/src/actuator/arm/scripts'))
for p in ('mission_ws/src/mission/jzzj_grasp/src','mission_ws/src/mission/jzzj_visual_servo/src'):
    sys.path.insert(0,str(ROOT/p))
from jzzj_grasp.core import Store,Context,Outcome

class Message:
    def __init__(self,**kwargs):self.__dict__.update(kwargs)
class Duration:
    def __init__(self,value):self.value=value
    def to_sec(self):return self.value
class Publisher:
    def __init__(self):self.sent=[]
    def publish(self,m):self.sent.append(m)
class Handle:
    def __init__(self,goal=None,id='goal'):
        self.goal=goal;self.id=id;self.status=None;self.result=None
    def get_goal(self):return self.goal
    def get_goal_id(self):return types.SimpleNamespace(id=self.id)
    def set_rejected(self,result,*args):self.status='rejected';self.result=result
    def set_accepted(self,*args):self.status='accepted'
    def set_succeeded(self,result):self.status='succeeded';self.result=result
    def set_canceled(self,result):self.status='canceled';self.result=result
    def set_aborted(self,result):self.status='aborted';self.result=result
    def publish_feedback(self,f):pass

rospy=types.ModuleType('rospy');rospy.Duration=Duration
rospy.Time=types.SimpleNamespace(now=lambda:Duration(100.))
rospy.logerr=lambda *args:None;rospy.logwarn=lambda *args:None
rospy.get_param=lambda name,default=None:default
rospy.get_time=lambda:100.
actionlib=types.ModuleType('actionlib')
messages=types.ModuleType('jzzj_grasp_interfaces.msg')
for n in ('VisualGraspAction','VisualGraspGoal','VisualGraspResult','VisualGraspFeedback','StageCContext','TargetSelection','TargetObservations'):
    setattr(messages,n,Message)
mods={'rospy':rospy,'actionlib':actionlib,'jzzj_grasp_interfaces':types.ModuleType('jzzj_grasp_interfaces'),
      'jzzj_grasp_interfaces.msg':messages}
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,str(ROOT/path));m=importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,mods):spec.loader.exec_module(m)
    return m
server=load('server_under_test','mission_ws/src/mission/jzzj_grasp/scripts/stage_c_server.py')
adapter=load('adapter_under_test','mission_ws/src/mission/jzzj_misson/scripts/stage_c_adapter.py')
CFG=json.loads((ROOT/'mission_ws/src/mission/jzzj_grasp/params/stage_c.yaml').read_text())['execution']

class DummyBackend:
    def ready(self,guard):guard()
    def select(self,*a):pass

class ServerTests(unittest.TestCase):
    def setUp(self):
        self.s=server.Server.__new__(server.Server);self.s.lock=threading.RLock();self.s.active=None
        self.s.poisoned=False;self.s.used_sessions=set();self.s.cfg=CFG;self.s.backend=DummyBackend()
        self.s.store=Store();self.s.store.context=Context(100.,'session',True,True,False,time.monotonic())
        self.s.selection=Publisher();self.s.engine=None
        self.goal=Message(session_id='session',target_color='red',grasp_profile='tabletop',timeout=Duration(30.))
    def accept(self,handle):
        with patch.object(server.threading,'Thread') as thread:self.s.goal_cb(handle)
    def test_valid_accepts_once(self):
        h=Handle(self.goal);self.accept(h);self.assertEqual(h.status,'accepted')
        self.assertTrue(self.s.selection.sent[-1].enabled)
        second=Handle(self.goal,'other');self.accept(second);self.assertEqual(second.status,'rejected')
    def test_replay_rejected(self):
        self.s.used_sessions.add('session');h=Handle(self.goal);self.accept(h)
        self.assertEqual(h.status,'rejected')
    def test_no_context_rejects_without_accepting(self):
        self.s.store.context=None;h=Handle(self.goal);self.accept(h);self.assertEqual(h.status,'rejected')
    def test_bad_goal_rejected(self):
        for attr,value in [('target_color','unknown'),('timeout',Duration(float('nan'))),('grasp_profile','none')]:
            original=getattr(self.goal,attr);setattr(self.goal,attr,value);h=Handle(self.goal);self.accept(h)
            self.assertEqual(h.status,'rejected');setattr(self.goal,attr,original)
    def test_cancel_matches_id_not_handle_identity(self):
        h=Handle(self.goal,'same');self.accept(h)
        other=Handle(self.goal,'same');self.s.cancel_cb(other);self.assertTrue(self.s.engine.cancel.is_set())
    def test_worker_stop_failed_poison(self):
        h=Handle(self.goal);self.accept(h)
        self.s.engine.execute=lambda *args:Outcome('STOP_FAILED','uncertain')
        self.s.run(h,self.s.engine,self.goal)
        self.assertTrue(self.s.poisoned);self.assertEqual(h.status,'aborted')
        self.assertFalse(self.s.selection.sent[-1].enabled)
    def test_worker_success_fields(self):
        h=Handle(self.goal);self.accept(h)
        self.s.engine.execute=lambda *args:Outcome('SUCCESS','ok',True,True,True,True,True)
        self.s.run(h,self.s.engine,self.goal)
        self.assertEqual(h.status,'succeeded');self.assertEqual(h.result.code,0)
        self.assertTrue(h.result.ready_for_transport)
    def test_no_second_open_after_success(self):
        h=Handle(self.goal);self.accept(h)
        self.s.engine.execute=lambda *args:Outcome('SUCCESS','ok',True,True,True,True,True)
        self.s.run(h,self.s.engine,self.goal)
        self.s.used_sessions.clear();second=Handle(self.goal,'new')
        self.accept(second);self.assertEqual(second.status,'rejected')
    def test_empty_detection_clears_snapshot(self):
        self.s.store.observations=[1]
        self.s.observation_cb(Message(targets=[],session_id='session',diagnostic='ok'))
        self.assertEqual(self.s.store.observations,[])

class Node:
    PICK_ALIGN='PICK_ALIGN';SEARCH_OBJECT='SEARCH_OBJECT';VISUAL_APPROACH='VISUAL_APPROACH'
    GRASP='GRASP';VERIFY_GRASP='VERIFY_GRASP';MOVE_TO_DROP='MOVE_TO_DROP'
    stages=['PICK_ALIGN','SEARCH_OBJECT','VISUAL_APPROACH','GRASP','VERIFY_GRASP','MOVE_TO_DROP']
    def __init__(self):self.state='RUNNING';self.stage_index=0;self.grasp_confirmed=False;self.fault=None;self.commands=0
    def _fault(self,text):self.fault=text;self.state='FAULT'
    def _publish_state(self):pass
    def _publish_stage_command(self):self.commands+=1

class TaskAdapterTests(unittest.TestCase):
    def setUp(self):
        self.a=adapter.StageCAdapter.__new__(adapter.StageCAdapter)
        self.a.lock=threading.RLock();self.a.active=False;self.a.done=None;self.a.latest=None
        self.a.started=100.;self.a.cancel_at=None;self.a.serial=1;self.a.stop_uncertain=False
        self.node=Node()
    def test_success_only_enters_drop_after_full_proof(self):
        self.a.done=(3,Message(code=0,grasp_confirmed=True,ready_for_transport=True,holding_known=True,holding_object=True))
        self.a.tick(self.node);self.assertEqual(self.node.stage_index,5);self.assertTrue(self.node.grasp_confirmed)
    def test_missing_grasp_proof_is_fault(self):
        self.a.done=(3,Message(code=0,grasp_confirmed=False,ready_for_transport=False,holding_known=False,holding_object=False,message='unconfirmed'))
        self.a.tick(self.node);self.assertEqual(self.node.state,'FAULT')
    def test_stopped_mission_never_advances_on_late_success(self):
        self.node.state='STOPPED';self.a.done=(3,Message(stop_confirmed=True))
        self.a.tick(self.node);self.assertEqual(self.node.stage_index,0)
    def test_uncertain_cancel_blocks_reset(self):
        self.a.completed(1,4,Message(code=9));self.assertTrue(self.a.stop_uncertain)
        with self.assertRaises(RuntimeError):self.a.reset()
    def test_old_result_not_accepted(self):
        self.a.completed(0,3,Message(code=0));self.assertIsNone(self.a.done)
    def test_cancel_timeout_does_not_grant_transport(self):
        self.a.active=True;self.a.cancel_at=time.monotonic()-10
        self.a.tick(self.node);self.assertEqual(self.node.state,'FAULT');self.assertEqual(self.node.stage_index,0)
    def test_cancel_wait_covers_server_stop_budget(self):
        self.a.active=True;self.a.cancel_at=time.monotonic()-7
        self.a.tick(self.node)
        self.assertIsNone(self.node.fault)
        self.assertEqual(self.node.stage_index,0)
    def test_feedback_updates_display_only(self):
        self.a.active=True;self.a.latest='DESCEND';self.a.tick(self.node)
        self.assertEqual(self.node.stage_index,3);self.assertEqual(self.node.commands,0)


class DriverContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sensor=types.ModuleType('sensor_msgs.msg');sensor.JointState=Message
        control=types.ModuleType('control_msgs.msg')
        for n in ('FollowJointTrajectoryAction','FollowJointTrajectoryResult','FollowJointTrajectoryFeedback'):
            setattr(control,n,Message)
        with patch.dict(mods,{'sensor_msgs.msg':sensor,'control_msgs.msg':control}):
            cls.module=load('driver_under_test','catkin_ws/src/actuator/arm/scripts/arm_trajectory_node.py')
    def setUp(self):
        self.d=self.module.TrajectoryDriver.__new__(self.module.TrajectoryDriver)
        self.d.lock=threading.RLock();self.d.active=None;self.d.poisoned=False;self.d.cancel=threading.Event()
        self.d.names=['joint'+str(i) for i in range(1,7)];self.d.lower=[-2]*6;self.d.upper=[2]*6
        self.d.tolerance=.035;self.d.stop_timeout=1.;self.d.settle_delta=.01
        self.d.sample=lambda:([0.]*6,0)
        point=Message(positions=[0.]*6,velocities=[],accelerations=[],effort=[],time_from_start=Duration(.8))
        trajectory=Message(points=[point],joint_names=self.d.names,header=Message(stamp=Duration(0.)))
        self.g=Message(trajectory=trajectory,path_tolerance=[],goal_tolerance=[],goal_time_tolerance=Duration(1.))
    def test_accepts_only_one_point(self):
        h=Handle(self.g)
        with patch.object(self.module.threading,'Thread'):self.d.goal(h)
        self.assertEqual(h.status,'accepted')
        self.d.active=None;self.g.trajectory.points*=2;h=Handle(self.g);self.d.goal(h)
        self.assertEqual(h.status,'rejected')
    def test_nan_or_duplicate_joint_rejects(self):
        self.g.trajectory.points[0].positions[0]=float('nan');h=Handle(self.g);self.d.goal(h)
        self.assertEqual(h.status,'rejected')
    def test_unsupported_velocity_does_not_silently_ignore(self):
        self.g.trajectory.points[0].velocities=[0]*6;h=Handle(self.g);self.d.goal(h)
        self.assertEqual(h.status,'rejected')
    def test_busy_rejects_without_preemption(self):
        self.d.active=Handle(id='running');h=Handle(self.g);self.d.goal(h)
        self.assertEqual(h.status,'rejected');self.assertFalse(self.d.cancel.is_set())
    def test_cancel_needs_confirmed_hold(self):
        self.d.write=lambda *args:None;self.d.converged=lambda *a:False;self.d.hold=lambda:True
        self.d.cancel.set();h=Handle(self.g);self.d.active=h
        self.d.execute(h,[0]*6,.8,1.,[.035]*6)
        self.assertEqual(h.status,'canceled');self.assertFalse(self.d.poisoned)
    def test_cancel_failed_hold_is_aborted_and_latched(self):
        self.d.write=lambda *args:None;self.d.converged=lambda *a:False;self.d.hold=lambda:False
        self.d.cancel.set();h=Handle(self.g);self.d.active=h
        self.d.execute(h,[0]*6,.8,1.,[.035]*6)
        self.assertEqual(h.status,'aborted');self.assertTrue(self.d.poisoned)
        self.assertIn('STOP_FAILED',h.result.error_string)
    def test_frozen_feedback_cannot_count_three_arrivals(self):
        self.assertFalse(self.d.converged([0]*6,time.monotonic()+.04,[.035]*6,False))
    def test_moving_feedback_cannot_count_as_settled(self):
        value=[0]
        def sample():
            value[0]+=1;return ([.02 if value[0]%2 else -.02]*6,value[0])
        self.d.sample=sample
        self.assertFalse(self.d.converged([0]*6,time.monotonic()+.08,[.035]*6,False))
    def test_feedback_timestamp_covers_oldest_sequential_read(self):
        self.d.io=threading.RLock();self.d.signs=[1]*6;self.d.offsets=[0]*6
        self.d.raw_lower=[-100]*6;self.d.raw_upper=[100]*6;self.d.sequence=0
        self.d.read_period=.1;self.d.pub=Publisher()
        self.d.device=Message(Arm_serial_servo_read=lambda _:0.)
        with patch.object(self.module,'JointState',lambda:Message(header=Message())), \
             patch.object(self.module.time,'monotonic',side_effect=[10.,10.64]), \
             patch.object(self.module.rospy,'logwarn_throttle',lambda *a:None,create=True):
            self.assertTrue(self.d.read())
        self.assertEqual(self.d.received,10.)
        self.assertAlmostEqual(self.d.read_duration,.64)
        self.assertEqual(self.d.sequence,1)
        self.assertEqual(self.d.pub.sent[0].header.stamp.to_sec(),100.)
    def test_failed_feedback_does_not_refresh_old_state(self):
        self.d.io=threading.RLock();self.d.raw_lower=[-100]*6;self.d.raw_upper=[100]*6
        self.d.sequence=7;self.d.received=1.;self.d.current=[.1]*6
        self.d.device=Message(Arm_serial_servo_read=lambda _:None)
        with patch.object(self.module.rospy,'logerr_throttle',lambda *a:None,create=True):
            self.assertFalse(self.d.read())
        self.assertEqual(self.d.sequence,7)
        self.assertEqual(self.d.received,1.)
        self.assertEqual(self.d.current,[.1]*6)

if __name__=='__main__':unittest.main()
