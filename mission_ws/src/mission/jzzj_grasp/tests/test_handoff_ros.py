"""Actual ROS-stage callbacks/execution driven by in-memory transports; never hardware."""
import copy
import json
from pathlib import Path
import tempfile
import threading
import types
import unittest
from unittest.mock import patch
from test_fixed_grasp import config,Adapter
from jzzj_grasp.fruit_stage_ros import FruitStage
from jzzj_grasp.handoff import Handoff
from jzzj_grasp.fixed_grasp import REAR_DEG,OPEN_DEG,CLOSE_DEG
M=types.SimpleNamespace


def request(operation=0,session='s',seq=1):
    return M(operation=operation,session_id=session,request_id=seq,issued_at=M(to_sec=lambda:10.),
             expires_at=M(to_sec=lambda:13.),next_owner='tag_navigation',confirmed_stopped=True,
             confirmed_empty_and_clear=True)


def stage():
    s=FruitStage.__new__(FruitStage);s.lock=threading.RLock();s.machine=Handoff();s.pending=None
    s.running=False;s.cancel=threading.Event();s.io=None;s.external_empty=False
    s.r=M(Time=M(now=lambda:M(to_sec=lambda:10.1)))
    s.response=lambda accepted,state,code,diagnostic:M(accepted=accepted,state=state,code=code,diagnostic=diagnostic)
    s.publish=lambda:None;s.check_upstream_released=lambda **kw:None
    return s


class CallbackTests(unittest.TestCase):
    def test_takeover_ack_is_pending_not_permission_to_skip_actual_stop_check(self):
        s=stage();reply=s.control(request())
        self.assertTrue(reply.accepted);self.assertEqual(reply.state,'TAKEOVER_CHECK')
        self.assertEqual(s.pending,'RUN');self.assertEqual(s.machine.owner,'stage_c_pending')
        self.assertIsNone(s.io)

    def test_foreign_owner_expired_and_replayed_requests_do_not_acquire(self):
        s=stage();s.check_upstream_released=lambda:(_ for _ in ()).throw(ValueError('old navigation still publishing'))
        reply=s.control(request());self.assertFalse(reply.accepted);self.assertEqual(s.machine.state,'WAIT_TAKEOVER')
        s=stage();expired=request();expired.expires_at=M(to_sec=lambda:9.)
        self.assertFalse(s.control(expired).accepted)
        self.assertTrue(s.control(request()).accepted);s.pending=None
        self.assertFalse(s.control(request(1,seq=1)).accepted)
        self.assertFalse(s.control(request(1,session='other',seq=2)).accepted)

    def test_cancel_active_or_queued_run_sets_event_without_new_motion(self):
        s=stage();s.control(request());self.assertEqual(s.pending,'RUN')
        reply=s.control(request(1,seq=2));self.assertTrue(reply.accepted)
        self.assertTrue(s.cancel.is_set());self.assertIsNone(s.io)

    def test_success_is_published_immediately_but_handoff_requires_explicit_ack(self):
        s=stage();s.control(request());s.pending=None
        s.machine.finish(dict(status='SUCCESS',success=True,motion_complete=True))
        self.assertEqual(s.machine.state,'DONE_WAIT_HANDOFF')
        bad=request(2,seq=2);bad.confirmed_stopped=False
        self.assertFalse(s.control(bad).accepted);self.assertIsNone(s.pending)
        self.assertTrue(s.control(request(2,seq=2)).accepted);self.assertEqual(s.pending,'RELEASE')
        self.assertEqual(s.machine.state,'DONE_WAIT_HANDOFF')

    def test_real_graph_check_rejects_old_controller_publisher_or_subscriber(self):
        s=stage();s.r.get_name=lambda:'/stage_c_fixed_grasp'
        for pubs,subs in ([('/cmd_vel',['/old'])],[]),([],[('/car_target',['/old_nav'])]):
            master=M(getSystemState=lambda:(pubs,subs,[]))
            with patch.dict('sys.modules',{'rosgraph':M(Master=lambda _:master)}):
                with self.assertRaises(ValueError):FruitStage.check_upstream_released(s)


class ExecutionTests(unittest.TestCase):
    def test_actual_stage_runs_fixed_sequence_without_tag_and_stays_owned_after_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            s=stage();s.path=Path(tmp)/'reference.json';s.reference=config();s.base_cfg={}
            s.control(request());s.pending=None;published=[]
            s.string=lambda **kw:kw;s.result_pub=M(publish=published.append)
            class IO(Adapter):
                def __init__(self,*args,**kw):
                    super().__init__();self.base=M(trace=[],stop_evidence='serial_zero_only_after_grasp',pregrasp_visual_stop=True)
                def ready(self):self.stage_state('OBSERVE','visual-only input available');super().ready()
                def prepare_grasp(self):
                    self.stage_state('STOP_CONFIRM','fruit stable');self.stage_state('ARM_GRASP','fixed action')
                def close(self):pass
            with patch('jzzj_grasp.fruit_stage_ros.wait_startup'),patch('jzzj_grasp.fruit_stage_ros.FixedRos',IO):s.run()
            self.assertEqual(s.machine.state,'DONE_WAIT_HANDOFF');self.assertEqual(s.machine.result['status'],'SUCCESS')
            result=json.loads(published[-1]['data']);self.assertTrue(result['success'])
            self.assertFalse(result['independent_postgrasp_stop_confirmed'])
            self.assertNotIn('grasp_confirmed',result);self.assertNotIn('human_verdict',result)
            self.assertEqual([e[1] for e in s.io.events if isinstance(e,tuple) and e[0]=='move'],[REAR_DEG,OPEN_DEG,CLOSE_DEG])
            self.assertTrue((Path(tmp)/'runs/s.json').exists())

    def test_failure_is_not_reported_as_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            s=stage();s.path=Path(tmp)/'reference.json';s.reference=config();s.base_cfg={}
            s.control(request());s.pending=None;published=[];s.string=lambda **kw:kw;s.result_pub=M(publish=published.append)
            class IO(Adapter):
                def __init__(self,*args,**kw):
                    super().__init__();self.fail='vision';self.base=M(trace=[],stop_evidence='visual_stop',pregrasp_visual_stop=False)
                def ready(self):self.stage_state('OBSERVE','visual');super().ready()
                def close(self):pass
            with patch('jzzj_grasp.fruit_stage_ros.wait_startup'),patch('jzzj_grasp.fruit_stage_ros.FixedRos',IO):s.run()
            self.assertEqual(s.machine.state,'FAULT');self.assertFalse(s.machine.result['success'])
            self.assertNotIn(('move',CLOSE_DEG),s.io.events)

    def test_release_only_after_ack_does_not_issue_navigation_goal(self):
        s=stage();s.control(request());s.pending=None
        s.machine.finish(dict(status='SUCCESS',success=True,motion_complete=True))
        s.control(request(2,seq=2));events=[]
        s.io=M(deadline=0.,release=lambda session:events.append(('arm_idle_hold_checked',session)),
               base=M(release=lambda **kw:events.append(('base_publishers_unregistered',kw))))
        s.dispose=lambda:events.append(('dispose',))
        s.release()
        self.assertEqual(s.machine.state,'RELEASED');self.assertEqual(s.machine.owner,'tag_navigation')
        self.assertEqual(events,[('arm_idle_hold_checked','s'),('base_publishers_unregistered',{'empty':False}),('dispose',)])


if __name__=='__main__':unittest.main()
