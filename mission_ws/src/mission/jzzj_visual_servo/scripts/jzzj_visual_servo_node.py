#!/usr/bin/env python3
from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool, Empty, String


class JzzjVisualServoNode(object):
    SEARCHING = "SEARCHING"
    TRACKING = "TRACKING"
    ALIGNED = "ALIGNED"
    STOPPED = "STOPPED"

    def __init__(self):
        rospy.init_node("jzzj_visual_servo_node")

        self.loop_rate = max(1.0, float(rospy.get_param("~loop_rate", 20.0)))
        self.target_color = str(rospy.get_param("~target_color", ""))
        self.horizontal_gain = float(rospy.get_param("~horizontal_gain", 0.002))
        self.vertical_gain = float(rospy.get_param("~vertical_gain", 0.002))
        self.depth_gain = float(rospy.get_param("~depth_gain", 0.25))
        self.horizontal_deadband = abs(float(rospy.get_param("~horizontal_deadband", 0.03)))
        self.vertical_deadband = abs(float(rospy.get_param("~vertical_deadband", 0.03)))
        self.depth_deadband = abs(float(rospy.get_param("~depth_deadband", 0.05)))
        self.max_linear_x = abs(float(rospy.get_param("~max_linear_x", 0.08)))
        self.max_linear_z = abs(float(rospy.get_param("~max_linear_z", 0.08)))
        self.max_angular_z = abs(float(rospy.get_param("~max_angular_z", 0.25)))
        self.require_target_visible = rospy.get_param("~require_target_visible", True)

        self.visible = False
        self.error = Vector3()
        self.state = self.SEARCHING
        self.enabled = False

        self.cmd_pub = rospy.Publisher("~cmd_vel", Twist, queue_size=1)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
        self.ready_pub = rospy.Publisher("~grasp_ready", Bool, queue_size=1, latch=True)

        rospy.Subscriber("~target_visible", Bool, self._visible_callback, queue_size=1)
        rospy.Subscriber("~target_error", Vector3, self._error_callback, queue_size=1)
        rospy.Subscriber("~enable", Bool, self._enable_callback, queue_size=1)
        rospy.Subscriber("~stop", Empty, self._stop_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.loop_rate), self._tick)

        self._publish_status()

    def _visible_callback(self, message):
        self.visible = bool(message.data)

    def _error_callback(self, message):
        self.error = message

    def _enable_callback(self, message):
        self.enabled = bool(message.data)
        if not self.enabled:
            self._stop_output()

    def _stop_callback(self, _message):
        self.enabled = False
        self.state = self.STOPPED
        self._stop_output()
        self._publish_status()

    def _tick(self, _event):
        if not self.enabled:
            self._stop_output()
            return

        if self.require_target_visible and not self.visible:
            self.state = self.SEARCHING
            self._stop_output()
            self._publish_status()
            return

        horizontal = self._outside_deadband(self.error.x, self.horizontal_deadband)
        vertical = self._outside_deadband(self.error.y, self.vertical_deadband)
        depth = self._outside_deadband(self.error.z, self.depth_deadband)

        if horizontal == 0.0 and vertical == 0.0 and depth == 0.0:
            self.state = self.ALIGNED
            self._stop_output()
            self.ready_pub.publish(Bool(data=True))
            self._publish_status()
            return

        self.state = self.TRACKING
        command = Twist()
        command.angular.z = self._clamp(
            -self.horizontal_gain * horizontal,
            -self.max_angular_z,
            self.max_angular_z)
        command.linear.z = self._clamp(
            -self.vertical_gain * vertical,
            -self.max_linear_z,
            self.max_linear_z)
        command.linear.x = self._clamp(
            self.depth_gain * depth,
            -self.max_linear_x,
            self.max_linear_x)
        self.cmd_pub.publish(command)
        self.ready_pub.publish(Bool(data=False))
        self._publish_status()

    @staticmethod
    def _outside_deadband(value, deadband):
        if abs(value) <= deadband:
            return 0.0
        return value

    @staticmethod
    def _clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def _stop_output(self):
        self.cmd_pub.publish(Twist())
        self.ready_pub.publish(Bool(data=False))

    def _publish_status(self):
        payload = "state={};enabled={};visible={};target_color={}".format(
            self.state, self.enabled, self.visible, self.target_color)
        self.state_pub.publish(String(data=payload))


if __name__ == "__main__":
    try:
        JzzjVisualServoNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
