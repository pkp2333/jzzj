"""Free grasp heading must preserve approach direction and complete-route checks."""
import copy
import math
import unittest
from unittest.mock import patch
from types import SimpleNamespace
from test_stage_c import Harness
from jzzj_grasp.core import Failure, validate_config
from jzzj_grasp.backends import RosArmBackend
from jzzj_grasp.planning import PathPlanner, Waypoint, tool_x_rotation


class OrientationSearchTests(unittest.TestCase):
    def setUp(self):
        self.cfg=copy.deepcopy(Harness().cfg);self.cfg['active_profile']='tabletop'
        self.p=self.cfg['profiles']['tabletop']
        self.p.update(transit_mode='joint',transport_orientation_xyzw=[0,0,0,1],
                      orientation_xyzw=[0,math.sqrt(.5),0,math.sqrt(.5)],
                      grasp_yaw_search_enabled=True,grasp_yaw_samples=4)
        self.start=(0.,)*5+(1.,)
        self.planner=PathPlanner(self.cfg,[-2.]*6,[2.]*6,None,None,lambda:None)

    def test_rotation_preserves_downward_axis_and_normalization(self):
        for i in range(24):
            x,y,z,w=tool_x_rotation(self.p['orientation_xyzw'],i*math.pi/12)
            self.assertAlmostEqual(x*x+y*y+z*z+w*w,1.)
            self.assertAlmostEqual(1-2*(y*y+z*z),0.)
            self.assertAlmostEqual(2*(x*y+w*z),0.)
            self.assertAlmostEqual(2*(x*z-w*y),-1.)

    def test_full_route_required_and_shorter_valid_route_selected_without_mutation(self):
        calls=[];original=copy.deepcopy(self.cfg)
        def full_route(trial,start,surface,remaining=False):
            rotation=trial.profile['orientation_xyzw'];calls.append(rotation)
            # Nominal orientation has an endpoint but fails its later transport.
            if abs(rotation[0])<1e-8:raise Failure('NEEDS_REPOSITION','transport infeasible')
            travel=.2 if rotation[0]<0 else .1
            point=Waypoint('MOVE_TO_TRANSPORT_POSE',(travel,0,0,0,0,1),.005)
            return [point],[point]
        with patch.object(PathPlanner,'plan',full_route):
            prefix,route=self.planner.search_grasp_orientation(self.start,(.1,0,.1))
        self.assertEqual(len(calls),4);self.assertEqual(route[0].joints[0],.1)
        self.assertIsNotNone(self.planner.selected_orientation)
        self.assertEqual(self.cfg,original)
        self.assertEqual(self.planner.orientation_search['route_points'],1)

    def test_cancellation_stop_failure_and_stale_feedback_are_never_swallowed(self):
        for code in ('CANCELED','STOP_FAILED','MOTION_FAILED','TIMEOUT'):
            calls=[]
            def fail(*a,**k):calls.append(1);raise Failure(code,'stop')
            with patch.object(PathPlanner,'plan',fail):
                with self.assertRaises(Failure) as ctx:self.planner.search_grasp_orientation(self.start,(0,0,.1))
            self.assertEqual(ctx.exception.code,code);self.assertEqual(len(calls),1)

    def test_all_candidates_rejected_has_no_selected_orientation(self):
        with patch.object(PathPlanner,'plan',side_effect=Failure('NEEDS_REPOSITION','limits')):
            with self.assertRaisesRegex(Failure,'no complete validated route'):
                self.planner.search_grasp_orientation(self.start,(0,0,.1))
        self.assertIsNone(self.planner.selected_orientation)

    def test_budget_is_bounded_and_never_returns_an_unfinished_route(self):
        self.p['grasp_yaw_search_timeout']=.1
        with patch('jzzj_grasp.planning.time.monotonic',side_effect=[0.,1.]):
            with self.assertRaises(Failure) as ctx:self.planner.search_grasp_orientation(self.start,(0,0,.1))
        self.assertEqual(ctx.exception.code,'TIMEOUT')

    def test_remaining_route_cannot_reselect_an_orientation(self):
        with self.assertRaisesRegex(Failure,'before descent'):self.planner.plan(self.start,(0,0,.1),True)

    def test_backend_freezes_only_this_goal_profile_after_complete_success(self):
        b=RosArmBackend.__new__(RosArmBackend);selected=tool_x_rotation(self.p['orientation_xyzw'],math.pi/2)
        trial=SimpleNamespace(selected_orientation=selected,orientation_search={'route_points':4},plan=lambda *a:None)
        b.planner=lambda g:trial;b.measured=lambda:self.start;b.rospy=SimpleNamespace(loginfo=lambda *a:None)
        transport=list(self.p['transport_orientation_xyzw'])
        b.preflight((0,0,.1),self.p,lambda:None)
        self.assertEqual(self.p['orientation_xyzw'],list(selected));self.assertTrue(self.p['_grasp_yaw_selected'])
        self.assertEqual(self.p['transport_orientation_xyzw'],transport)
        # A new goal starts from the untouched server defaults, without a frozen heading.
        fresh=copy.deepcopy(Harness().cfg)
        self.assertNotIn('_grasp_yaw_selected',fresh['profiles']['tabletop'])

    def test_backend_never_freezes_a_failed_preflight(self):
        b=RosArmBackend.__new__(RosArmBackend);before=copy.deepcopy(self.p)
        trial=SimpleNamespace(plan=lambda *a:(_ for _ in ()).throw(Failure('NEEDS_REPOSITION','transport')))
        b.planner=lambda g:trial;b.measured=lambda:self.start
        with self.assertRaises(Failure):b.preflight((0,0,.1),self.p,lambda:None)
        self.assertEqual(self.p,before)

    def test_invalid_search_configuration_is_rejected(self):
        validate_config(self.cfg)
        for key,value in [('grasp_yaw_samples',100),('grasp_yaw_samples',True),
                          ('grasp_yaw_search_timeout',float('nan')),('grasp_yaw_search_enabled',1),
                          ('orientation_xyzw',[0,0,0,1]),('transit_mode','cartesian')]:
            cfg=copy.deepcopy(self.cfg);cfg['profiles']['tabletop'][key]=value
            with self.assertRaises(ValueError):validate_config(cfg)


if __name__=='__main__':unittest.main()
