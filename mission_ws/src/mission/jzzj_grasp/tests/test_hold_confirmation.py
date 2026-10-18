"""Slow-feedback stop regressions; actual driver convergence, no hardware."""
import math
import sys
import types
import unittest
from unittest.mock import patch
import test_ros_contracts as contracts
from test_ros_contracts import Duration, Message
from jzzj_grasp.backends import RosArmBackend
from jzzj_grasp.core import Failure
import jzzj_grasp.backends as backends


class Clock:
    def __init__(self): self.t = 0.
    def monotonic(self): return self.t
    def sleep(self, seconds): self.t += seconds


class HoldBudgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        contracts.DriverContractTests.setUpClass()
        cls.driver_module = contracts.DriverContractTests.module

    def test_stationary_slow_feedback_has_time_to_confirm_hold(self):
        clock = Clock()
        driver = self.driver_module.TrajectoryDriver.__new__(self.driver_module.TrajectoryDriver)
        driver.settle_delta = .01
        driver.sample = lambda: ([0.] * 6, math.floor(clock.t / .645))
        driver.cancel = types.SimpleNamespace(is_set=lambda: False)
        result = Message(error_code=0, error_string='')
        state = [0]
        goals = []
        def send(goal):
            goals.append(goal)
            duration = goal.trajectory.points[0].time_from_start.to_sec()
            extra = max(1., goal.goal_time_tolerance.to_sec())
            ok = driver.converged([0.]*6, clock.t+duration+extra, [.035]*6, False)
            state[0] = 3 if ok else 4
            result.error_code = 0 if ok else -5
        b = RosArmBackend.__new__(RosArmBackend)
        b.cfg = {'joint_names': ['joint'+str(i) for i in range(1,7)], 'arrival_timeout': 4.}
        b.sent = False
        b.rospy = Message(Duration=Duration)
        b.fresh_joints = lambda: Message(name=b.cfg['joint_names'], position=[0.]*6)
        b.client = Message(send_goal=send, get_state=lambda: state[0], get_result=lambda: result)
        control = types.ModuleType('control_msgs.msg')
        def goal():
            return Message(trajectory=Message(), goal_time_tolerance=Duration(0.))
        control.FollowJointTrajectoryGoal = goal
        trajectory = types.ModuleType('trajectory_msgs.msg'); trajectory.JointTrajectoryPoint = Message
        with patch.dict(sys.modules, {'control_msgs.msg':control, 'trajectory_msgs.msg':trajectory}), \
             patch.object(backends.time, 'monotonic', clock.monotonic), \
             patch.object(backends.time, 'sleep', clock.sleep):
            self.assertTrue(b.hold(12.))
        self.assertEqual(len(goals), 1)
        self.assertGreaterEqual(clock.t, 1.9)

    def test_latched_stop_failure_cannot_send_another_goal(self):
        b=RosArmBackend.__new__(RosArmBackend);b.sent=True
        b.client=Message(get_state=lambda:4,get_result=lambda:Message(error_code=-5,error_string='STOP_FAILED'))
        control=types.ModuleType('control_msgs.msg');control.FollowJointTrajectoryGoal=Message
        trajectory=types.ModuleType('trajectory_msgs.msg');trajectory.JointTrajectoryPoint=Message
        with patch.dict(sys.modules,{'control_msgs.msg':control,'trajectory_msgs.msg':trajectory}):
            self.assertFalse(b.hold(12.))  # A send_goal attempt would raise.


class TrajectoryResultBudgetTests(unittest.TestCase):
    def test_waits_for_driver_arrival_failure_and_measured_hold_without_masking_error(self):
        clock=Clock();sent=[]
        control=types.ModuleType('control_msgs.msg')
        control.FollowJointTrajectoryGoal=lambda:Message(trajectory=Message())
        trajectory=types.ModuleType('trajectory_msgs.msg');trajectory.JointTrajectoryPoint=Message
        b=RosArmBackend.__new__(RosArmBackend)
        b.cfg=dict(joint_names=['joint'+str(i) for i in range(1,7)],motion_seconds=.8,arrival_timeout=4.)
        b.rospy=Message(Duration=Duration,get_param=lambda k,d:5. if k.endswith('stop_timeout') else 1.5)
        b.fresh_joints=lambda:None
        b.client=Message(send_goal=sent.append,get_state=lambda:1 if clock.t<9.8 else 4,
                         get_result=lambda:Message(error_code=-5,error_string='arrival timeout; hold confirmed'))
        with patch.dict(sys.modules,{'control_msgs.msg':control,'trajectory_msgs.msg':trajectory}), \
             patch.object(backends.time,'monotonic',clock.monotonic),patch.object(backends.time,'sleep',clock.sleep):
            with self.assertRaisesRegex(Failure,'arrival timeout; hold confirmed'):b.trajectory([0.]*6,lambda:None)
        self.assertGreaterEqual(clock.t,9.8);self.assertEqual(len(sent),1)

    def test_cancellation_still_interrupts_extended_wait(self):
        clock=Clock();control=types.ModuleType('control_msgs.msg')
        control.FollowJointTrajectoryGoal=lambda:Message(trajectory=Message())
        trajectory=types.ModuleType('trajectory_msgs.msg');trajectory.JointTrajectoryPoint=Message
        b=RosArmBackend.__new__(RosArmBackend);b.cfg=dict(joint_names=list('123456'),motion_seconds=.8,arrival_timeout=4.)
        b.rospy=Message(Duration=Duration,get_param=lambda k,d:d);b.client=Message(send_goal=lambda g:None,get_state=lambda:1)
        def guard():
            if clock.t>2:raise Failure('CANCELED','operator')
        with patch.dict(sys.modules,{'control_msgs.msg':control,'trajectory_msgs.msg':trajectory}), \
             patch.object(backends.time,'monotonic',clock.monotonic),patch.object(backends.time,'sleep',clock.sleep):
            with self.assertRaisesRegex(Failure,'operator'):b.trajectory([0.]*6,guard)
        self.assertLess(clock.t,2.1)


if __name__ == '__main__': unittest.main()
