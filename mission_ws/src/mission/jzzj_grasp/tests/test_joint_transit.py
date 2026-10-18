"""Five-DOF transitions: endpoints are never direct actuator targets."""
import math
import unittest
import test_grasp_planning as fixtures
from jzzj_grasp.core import Failure
from jzzj_grasp.planning import Pose

class JointTransitTests(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.PlannerTests();self.f.setUp()
        self.f.p.update(transit_mode='joint',transport_orientation_xyzw=[0.,0.,0.,1.],transit_clearance_height_m=.15)
        self.planner=self.f.planner();self.planner.endpoint_inverse=self.f.inverse
    def test_complete_route_preserves_gripper_and_bounds_every_joint_edge(self):
        prefix,route=self.planner.plan(self.f.start,self.f.surface)
        self.assertTrue(prefix);self.assertEqual(prefix[0].step,'PREPARE_TRANSITION')
        self.assertIn('LIFT',[x.step for x in route]);last=self.f.start
        for point in route:
            self.assertEqual(point.joints[5],last[5])
            self.assertLessEqual(max(abs(a-b) for a,b in zip(last,point.joints)),.35+1e-12);last=point.joints
        final=self.f.forward(last)
        self.assertLess(math.dist(final.position,self.f.p['transport_position']),1e-8)
        self.assertEqual(final.orientation,(0.,0.,0.,1.))
    def test_missing_solver_out_of_limit_and_changed_gripper_do_not_make_a_route(self):
        p=self.planner;p.endpoint_inverse=None
        with self.assertRaisesRegex(Failure,'solver absent'):p.plan(self.f.start,self.f.surface)
        for end in ((5.,0.,.2,0.,0.,1.),(.1,0.,.2,0.,0.,.5)):
            p.endpoint_inverse=lambda *a:end
            with self.assertRaises(Failure):p.joint_transit(self.f.start,(.1,0.,.2),(0,0,0,1),'TEST',.005)
    def test_joint_sweep_checks_midpoint_not_only_endpoint(self):
        p=self.planner;p.forward=lambda q:Pose((.1,0.,.2-.4*math.sin(math.pi*q[0]/.1)),(0,0,0,1))
        p.endpoint_inverse=lambda *a:(.1,0.,0.,0.,0.,1.)
        with self.assertRaisesRegex(Failure,'swept TCP'):
            p.joint_transit((0,0,0,0,0,1),(.1,0,.2),(0,0,0,1),'TEST',.005)
    def test_large_cartesian_step_is_refined_without_increasing_joint_limit(self):
        p=self.planner;original=self.f.inverse
        def inverse(point,rotation,seed):
            if math.dist(point,seed[:3])>.004:raise Failure('NEEDS_REPOSITION','local solve needs a smaller step')
            return original(point,rotation,seed)
        p.inverse=inverse
        points,_=p.segment(self.f.start,(.11,0,.2),(0,0,0,1),'TEST')
        self.assertGreater(len(points),2)
        self.assertTrue(all(math.dist(a.joints[:3],b.joints[:3])<=.004 for a,b in zip(points,points[1:])))
    def test_motion_failure_and_cancellation_are_never_retried(self):
        for code in ('MOTION_FAILED','CANCELED','STOP_FAILED'):
            calls=[]
            def inverse(*a):calls.append(a);raise Failure(code,'stop')
            self.planner.inverse=inverse
            with self.assertRaises(Failure):self.planner.segment(self.f.start,(.11,0,.2),(0,0,0,1),'TEST')
            self.assertEqual(len(calls),1)
    def test_transition_obeys_the_existing_total_waypoint_budget(self):
        self.f.cfg['max_motion_steps']=1
        with self.assertRaisesRegex(Failure,'motion budget'):self.planner.plan(self.f.start,self.f.surface)
    def test_simulation_completes_independent_transport_phase(self):
        from test_stage_c import Harness
        h=Harness();h.cfg['profiles']['tabletop'].update(transit_mode='joint',transport_orientation_xyzw=[0,0,0,1])
        result=h.run();self.assertEqual(result.code,'SUCCESS',result.message)
    def test_new_profile_keeps_real_motion_and_calibration_gates_disabled(self):
        import json
        from test_stage_c import ROOT
        from jzzj_grasp.core import validate_config
        c=json.loads((ROOT/'mission_ws/src/mission/jzzj_grasp/params/stage_c_downward.example.yaml').read_text())
        e=c['execution'];validate_config(e)
        self.assertFalse(any(e[k] for k in ('real_motion_enabled','robot_calibration_confirmed','workspace_clear_confirmed','model_validated')))
        self.assertFalse(e['profiles']['tabletop']['calibrated'])
        self.assertFalse(c['perception']['calibration_confirmed'])
        self.assertEqual(e['profiles']['tabletop']['transit_mode'],'joint')

if __name__=='__main__':unittest.main()
