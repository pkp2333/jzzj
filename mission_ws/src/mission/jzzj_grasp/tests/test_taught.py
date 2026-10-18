"""Taught grasp and planar correction regressions, no ROS/hardware."""
import copy
import math
import unittest
import test_stage_c
from jzzj_grasp.core import Observation,Failure
from jzzj_grasp.taught import (Coordinator,Snapshot,fit_base_matrix,plane_basis,
                              template_hash,validate_template,split_sequence)
import numpy as np


def template():
    q=[0.,0.,.2,0.,0.,.1]
    t=dict(version=1,units='rad',real_motion_enabled=True,workspace_clear_confirmed=True,
           table_height_confirmed=True,base_calibrated=True,bindings={'camera':'field'},
           arm_frame='arm',table_normal=[0.,0.,1.],reference_position_m=[.1,.05,-.05],
           identity=dict(class_id=1,label='lemon',color='yellow'),base_matrix=[[-1.,0.],[0.,-1.]],
           start_rad=list(q),steps=[])
    for phase in ('open','approach','descend','close','lift','transport'):
        if phase=='open':q[5]=1.
        if phase=='approach':q[0]=.25
        if phase=='descend':q[2]=.1
        if phase=='close':q[5]=.5
        if phase=='lift':q[2]=.3
        if phase=='transport':q[0]=0.
        t['steps'].append(dict(phase=phase,positions_rad=list(q),seconds=.8))
    t['empty_validation']={'template_hash':template_hash(t)}
    return t


class Adapter:
    def __init__(self,t,offset=(.025,-.018,0.)):
        self.t=t;self.point=np.array(t['reference_position_m'])+offset
        self.clock=0.;self.events=[];self.locked=False;self.q=t['start_rad'];self.moves=[]
        self.arm_ok=True;self.base_ok=True;self.fail_execute=False;self.slip=1.
    def bindings(self):return self.t['bindings']
    def ready(self):self.events.append('ready')
    def guard(self):pass
    def wait_arm_still(self):self.events.append('still')
    def check_start(self,q):pass
    def preflight(self,steps):self.events.append('preflight')
    def acquire_base(self):self.events.append('acquire')
    def hold_base(self,locked=False):self.events.append('hold_base')
    def lock_base(self):self.locked=True;self.events.append('lock')
    def observe(self,anchor,deadline):self.clock+=1.05;return tuple(self.point)
    def pulse(self,axis,speed,seconds):
        if self.locked:raise AssertionError('base moved during arm execution')
        d=np.zeros(2);d[axis]=speed*seconds*self.slip
        self.point+=plane_basis(self.t['table_normal'])@(np.array(self.t['base_matrix'])@d)
        self.moves.append((axis,speed,seconds));self.events.append('pulse')
    def execute(self,step):
        if not self.locked:raise AssertionError('arm before base lock')
        self.events.append('arm')
        if self.fail_execute:raise Failure('MOTION_FAILED','arrival timeout; hold confirmed')
        self.q=step['positions_rad']
    def hold_arm(self):self.events.append('hold_arm');return self.arm_ok
    def stop_base(self):self.events.append('stop_base');return self.base_ok


class TaughtTests(unittest.TestCase):
    def test_reversed_and_rotated_base_axes_converge_without_inverse_calls(self):
        for matrix in ([[-1.,0.],[0.,-1.]],[[0.,1.],[-1.,0.]]):
            t=template();t['base_matrix']=matrix;t['empty_validation']={'template_hash':template_hash(t)}
            io=Adapter(t);result=Coordinator(io,t,lambda:io.clock).run()
            self.assertTrue(result['motion_complete'],result)
            self.assertEqual(result['status'],'MOTION_COMPLETE_AWAITING_OPERATOR')
            self.assertEqual(result['human_verdict'],'pending')
            self.assertNotIn('grasp_confirmed',result)
            self.assertLessEqual(np.linalg.norm(io.point-np.array(t['reference_position_m'])),.005)
            self.assertLess(io.events.index('preflight'),io.events.index('pulse'))
            self.assertLess(io.events.index('lock'),io.events.index('arm'))
            for _,speed,seconds in io.moves:
                self.assertLessEqual(abs(speed),.02);self.assertLessEqual(abs(speed*seconds),.01)

    def test_invalid_and_changed_templates_never_acquire_or_move(self):
        for change in (lambda t:t.update(real_motion_enabled=False),lambda t:t.update(start_rad=[]),
                       lambda t:t.update(empty_validation={}),lambda t:t.update(base_matrix=[[0,0],[0,0]]),
                       lambda t:t['steps'][1].update(phase='lift')):
            t=template();change(t);io=Adapter(t);r=Coordinator(io,t,lambda:io.clock).run()
            self.assertFalse(r['motion_complete']);self.assertNotIn('acquire',io.events)
        t=template();io=Adapter(t);io.bindings=lambda:{'camera':'changed'}
        self.assertFalse(Coordinator(io,t,lambda:io.clock).run()['motion_complete'])
        self.assertEqual(io.events,[])

    def test_motion_steps_preserve_gripper_and_gripper_phases_preserve_arm(self):
        t=template();last=t['start_rad']
        for step in split_sequence(t):
            q=step['positions_rad']
            if step['phase'] in ('open','close'):self.assertEqual(tuple(last[:5]),q[:5])
            else:
                self.assertEqual(last[5],q[5]);self.assertLessEqual(max(abs(a-b) for a,b in zip(last[:5],q[:5])),.1+1e-12)
            last=q

    def test_targets_beyond_travel_budget_and_excessive_slip_never_reach_arm(self):
        for offset,slip in [((.2,0,0),1.),((.04,0,0),3.)]:
            t=template();io=Adapter(t,offset);io.slip=slip
            r=Coordinator(io,t,lambda:io.clock).run()
            self.assertFalse(r['motion_complete']);self.assertNotIn('arm',io.events)
            self.assertEqual(io.events[-1],'stop_base')

    def test_target_drift_after_lock_refuses_grasp(self):
        t=template();io=Adapter(t,offset=(0,0,0));original=io.observe
        io.observe=lambda a,d:tuple(io.point+(.02 if io.locked else 0))
        r=Coordinator(io,t,lambda:io.clock).run()
        self.assertEqual(r['status'],'NEEDS_REPOSITION');self.assertNotIn('arm',io.events)

    def test_motion_failure_stops_once_and_hold_failure_latches_result(self):
        t=template();io=Adapter(t);io.fail_execute=True;io.arm_ok=False
        r=Coordinator(io,t,lambda:io.clock).run()
        self.assertEqual(r['status'],'STOP_FAILED');self.assertFalse(r['stop_confirmed'])
        self.assertEqual(io.events.count('arm'),1);self.assertEqual(io.events[-2:],['hold_arm','stop_base'])

    def test_cancellation_and_alignment_timeout_stop_without_arm_commands(self):
        t=template();io=Adapter(t)
        def guard():
            if io.moves:raise Failure('CANCELED','operator')
        io.guard=guard;r=Coordinator(io,t,lambda:io.clock).run()
        self.assertEqual(r['status'],'CANCELED');self.assertNotIn('arm',io.events)
        io=Adapter(t);r=Coordinator(io,t,lambda:io.clock,dict(timeout=.5)).run()
        self.assertEqual(r['status'],'TIMEOUT');self.assertNotIn('arm',io.events)

    def test_no_empty_approval_needed_only_for_explicit_empty_trial(self):
        t=template();t['empty_validation']={};io=Adapter(t)
        r=Coordinator(io,t,lambda:io.clock).run(empty=True)
        self.assertTrue(r['motion_complete']);self.assertFalse(io.moves)

    def test_calibration_fits_measured_directions_and_rejects_degenerate_or_bad_scale(self):
        samples=[];a=np.array([[0.,1.],[-1.,0.]])
        for d in ([.01,0],[-.01,0],[0,.01],[0,-.01]):
            e=a@d;samples.append(dict(base_delta_m=d,before_m=[0,0,0],after_m=[*e,0]))
        matrix,residual=fit_base_matrix(samples,[0,0,1]);np.testing.assert_allclose(matrix,a,atol=1e-12)
        self.assertLess(residual,1e-12)
        with self.assertRaises(ValueError):fit_base_matrix(samples[:2]*2,[0,0,1])
        bad=copy.deepcopy(samples)
        for s in bad:s['after_m']=[v*3 for v in s['after_m']]
        with self.assertRaises(ValueError):fit_base_matrix(bad,[0,0,1])


class SnapshotTests(unittest.TestCase):
    def snapshot(self):return Snapshot('s','arm',(1,'lemon','yellow'),1.,anchor=(.1,0,-.05))
    def frame(self,s,stamp,targets=True,session='s',diag='ok'):
        obs=Observation(stamp,session,'arm',(.1,0,-.05),'yellow','lemon',1)
        values=[obs] if targets is True else targets
        return s.add(stamp,session,diag,values,stamp+.01)
    def test_three_distinct_post_stop_frames_over_one_second_required(self):
        s=self.snapshot();self.assertIsNone(self.frame(s,1.1));self.assertIsNone(self.frame(s,1.1))
        self.assertIsNone(self.frame(s,1.6));self.assertIsNotNone(self.frame(s,2.2))
    def test_empty_frame_clears_stability_and_old_session_never_counts(self):
        s=self.snapshot();self.frame(s,1.1);self.frame(s,1.6)
        self.frame(s,1.8,session='old');self.frame(s,2.,targets=[])
        self.assertIsNone(self.frame(s,2.2));self.assertIsNone(self.frame(s,2.6));self.assertIsNotNone(self.frame(s,3.3))
    def test_ambiguous_failed_stale_and_jumped_targets_cannot_confirm(self):
        s=self.snapshot();o=Observation(2.,'s','arm',(.1,0,-.05),'yellow','lemon',1)
        with self.assertRaises(Failure):self.frame(s,2.,targets=[o,o])
        with self.assertRaises(Failure):self.frame(self.snapshot(),2.,diag='camera failed')
        self.assertIsNone(s.add(10.,'s','ok',[o],12.))
        far=Observation(2.,'s','arm',(.3,0,-.05),'yellow','lemon',1)
        with self.assertRaises(Failure):self.frame(self.snapshot(),2.,targets=[far])


if __name__=='__main__':unittest.main()
