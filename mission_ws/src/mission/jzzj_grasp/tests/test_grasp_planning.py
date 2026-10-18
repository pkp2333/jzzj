"""Field regressions: no SDK, ROS master, camera or physical movement."""
import copy
import math
import unittest
from test_stage_c import Harness
from jzzj_grasp.core import Failure, validate_config
from jzzj_grasp.backends import RosArmBackend
from jzzj_grasp.planning import PathPlanner, Pose, quaternion_angle


class BeforeOpeningTests(unittest.TestCase):
    def test_unreachable_route_does_not_open_gripper(self):
        h = Harness()
        def reject(*args): raise Failure('NEEDS_REPOSITION', 'preflight: first lift unreachable')
        h.backend.preflight = reject
        result = h.run()
        self.assertEqual(result.code, 'NEEDS_REPOSITION')
        self.assertTrue(result.stop_confirmed)
        self.assertFalse(any(x[0] in ('grip','move') for x in h.backend.trace))

    def test_moved_target_is_checked_again_before_descent(self):
        h=Harness();calls=[]
        def preflight(surface,profile,guard,remaining=False):
            calls.append(remaining)
            if remaining:raise Failure('NEEDS_REPOSITION','transport from updated target impossible')
        h.backend.preflight=preflight
        result=h.run()
        self.assertEqual(calls,[False,True])
        self.assertEqual(result.code,'NEEDS_REPOSITION')
        self.assertNotIn('CLOSE_GRIPPER',h.steps)

    def test_unverified_or_nonfinite_transition_is_configuration_error(self):
        for calibrated,joints in [(False,[0.]*5),(True,[float('nan')]+[0.]*4)]:
            cfg=copy.deepcopy(Harness().cfg)
            cfg['profiles']['tabletop'].update(transition_calibrated=calibrated,
                transition_joint_waypoints_rad=[joints])
            with self.assertRaises(ValueError):validate_config(cfg)


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.cfg=copy.deepcopy(Harness().cfg);self.cfg['active_profile']='tabletop'
        self.cfg['workspace_min']=[-1.,-1.,.005];self.cfg['workspace_max']=[1.,1.,1.]
        self.p=self.cfg['profiles']['tabletop'];self.p['table_plane_offset_m']=0.
        self.p['orientation_xyzw']=[0.,math.sin(.2),0.,math.cos(.2)]
        self.start=(.1,0.,.2,0.,0.,1.)
        self.surface=(.15,0.,.1);self.inverse_calls=[];self.fk_calls=[]
    def forward(self,q):
        self.fk_calls.append(q)
        return Pose(tuple(q[:3]),(0.,math.sin(q[3]/2),0.,math.cos(q[3]/2)))
    def inverse(self,point,rotation,seed):
        self.inverse_calls.append((point,rotation,seed))
        return tuple(point)+(2*math.atan2(rotation[1],rotation[3]),seed[4],seed[5])
    def planner(self,guard=lambda:None):
        return PathPlanner(self.cfg,[-2.]*6,[2.]*6,self.forward,self.inverse,guard)
    def test_full_route_preflight_preserves_gripper_and_interpolates_orientation(self):
        prefix,route=self.planner().plan(self.start,self.surface)
        self.assertTrue(prefix);self.assertGreater(len(route),len(prefix))
        self.assertTrue(all(q.joints[5]==1. for q in route))
        self.assertIn('DESCEND',[q.step for q in route])
        self.assertIn('LIFT',[q.step for q in route])
        self.assertIn('MOVE_TO_TRANSPORT_POSE',[q.step for q in route])
        first=self.inverse_calls[0][1]
        self.assertLessEqual(quaternion_angle(self.forward(self.start).orientation,first),.1+1e-9)
        self.assertLess(quaternion_angle(self.forward(self.start).orientation,first),.4)
        self.assertEqual(tuple(route[-1].joints[:3]),tuple(self.p['transport_position']))
    def test_transport_infeasible_rejects_entire_preflight(self):
        original=self.inverse
        def inverse(point,rotation,seed):
            if point[0]<.09:raise Failure('NEEDS_REPOSITION','transport unreachable')
            return original(point,rotation,seed)
        planner=self.planner();planner.inverse=inverse
        with self.assertRaisesRegex(Failure,'MOVE_TO_TRANSPORT_POSE'):
            planner.plan(self.start,self.surface)
    def test_sparse_joint_transition_checks_midpoint_table_collision(self):
        planner=self.planner()
        def forward(q):
            z=.2-.4*math.sin(math.pi*q[0]/.1)
            return Pose((.1,0.,z),(0.,0.,0.,1.))
        planner.forward=forward
        with self.assertRaisesRegex(Failure,'swept TCP'):
            planner.edge((0.,0.,0.,0.,0.,1.),(.1,0.,0.,0.,0.,1.))
    def test_joint_transition_requires_calibration_and_limits(self):
        self.p['transition_joint_waypoints_rad']=[[.12,0.,.21,.2,0.]]
        with self.assertRaisesRegex(Failure,'not calibrated'):
            self.planner().plan(self.start,self.surface)
        self.p['transition_calibrated']=True
        prefix,_=self.planner().plan(self.start,self.surface)
        self.assertEqual(prefix[0].step,'PREPARE_TRANSITION')
        self.p['transition_joint_waypoints_rad']=[[5.,0.,.21,.2,0.]]
        with self.assertRaisesRegex(Failure,'out-of-limit'):
            self.planner().plan(self.start,self.surface)
    def test_out_of_workspace_transport_and_budget_block_route(self):
        self.p['transport_position']=[2.,0.,.2]
        with self.assertRaisesRegex(Failure,'swept TCP'):
            self.planner().plan(self.start,self.surface)
        self.p['transport_position']=[.08,0.,.24];self.cfg['max_motion_steps']=2
        with self.assertRaisesRegex(Failure,'motion budget'):
            self.planner().plan(self.start,self.surface)
    def test_cancel_during_planning_never_finishes(self):
        count=[0]
        def guard():
            count[0]+=1
            if count[0]>8:raise Failure('CANCELED','cancel during preflight')
        with self.assertRaisesRegex(Failure,'cancel during preflight'):
            self.planner(guard).plan(self.start,self.surface)
    def test_frozen_position_with_wrong_orientation_is_not_ready(self):
        planner=self.planner();planner.inverse=lambda point,rotation,seed:tuple(point)+seed[3:]
        with self.assertRaisesRegex(Failure,'FK residual'):
            planner.plan(self.start,self.surface)
    def test_joint_jump_cannot_hide_behind_close_tcp_points(self):
        planner=self.planner()
        planner.inverse=lambda point,rotation,seed:tuple(point)+(1.,0.,1.)
        with self.assertRaisesRegex(Failure,'continuity limit'):
            planner.plan(self.start,self.surface)
    def test_negative_quaternion_does_not_rotate(self):
        q=self.forward(self.start).orientation
        self.assertAlmostEqual(quaternion_angle(q,tuple(-x for x in q)),0.)

    def test_backend_never_executes_prefix_if_later_transport_fails(self):
        self.p['transport_position']=[2.,0.,.2]
        b=RosArmBackend.__new__(RosArmBackend)
        b.planner=lambda guard:self.planner(guard)
        b.measured=lambda:self.start
        sent=[];b.trajectory=lambda *a:sent.append(a)
        with self.assertRaises(Failure):b.prepare(self.surface,self.p,lambda:None,lambda *a:None)
        self.assertEqual(sent,[])

    def test_backend_rejects_changed_arm_state_before_replaying_path(self):
        planner=self.planner();prefix,_=planner.plan(self.start,self.surface)
        b=RosArmBackend.__new__(RosArmBackend)
        b.measured=lambda:(self.start[0]+.1,)+self.start[1:]
        sent=[];b.trajectory=lambda *a:sent.append(a)
        with self.assertRaisesRegex(Failure,'deviated'):
            b.execute_plan(planner,prefix,self.start,lambda:None,lambda *a:None)
        self.assertEqual(sent,[])


if __name__ == '__main__': unittest.main()
