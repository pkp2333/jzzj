#!/usr/bin/env python3
from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool, Empty, String

try:
    from user_msgs.msg import ObjDets
except ImportError:
    ObjDets = None


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


class JzzjVisualServoNode(object):
    SEARCHING = "SEARCHING"
    TRACKING = "TRACKING"
    ALIGNED = "ALIGNED"
    STOPPED = "STOPPED"

    def __init__(self):
        rospy.init_node("jzzj_visual_servo_node")

        self.loop_rate = max(1.0, float(rospy.get_param("~loop_rate", 20.0)))
        self.target_color = str(rospy.get_param("~target_color", "")).strip().lower()
        self.horizontal_gain = float(rospy.get_param("~horizontal_gain", 1.2))
        self.vertical_gain = float(rospy.get_param("~vertical_gain", 0.8))
        self.depth_gain = float(rospy.get_param("~depth_gain", 0.5))
        self.horizontal_deadband = abs(float(rospy.get_param("~horizontal_deadband", 0.03)))
        self.vertical_deadband = abs(float(rospy.get_param("~vertical_deadband", 0.03)))
        self.depth_deadband = abs(float(rospy.get_param("~depth_deadband", 0.05)))
        self.max_linear_x = abs(float(rospy.get_param("~max_linear_x", 0.12)))
        self.max_linear_y = abs(float(rospy.get_param("~max_linear_y", 0.12)))
        self.max_angular_z = abs(float(rospy.get_param("~max_angular_z", 0.35)))
        self.require_target_visible = bool(rospy.get_param("~require_target_visible", True))
        self.target_timeout = max(0.05, float(rospy.get_param("~target_timeout", 0.5)))
        self.use_detector = bool(rospy.get_param("~use_detector", True))
        self.image_width = max(1.0, float(rospy.get_param("~image_width", 640.0)))
        self.image_height = max(1.0, float(rospy.get_param("~image_height", 480.0)))
        self.reference_bbox_height = max(1.0, float(
            rospy.get_param("~reference_bbox_height", 150.0)))
        self.min_score = float(rospy.get_param("~min_score", 0.35))
        self.target_class_ids = [int(v) for v in rospy.get_param("~target_class_ids", [])]
        self.detector_topic = str(rospy.get_param(
            "~detector_topic", "/ascamera_hp60c/rgb0/yolov5_detections"))

        self.visible = False
        self.error = Vector3()
        self.state = self.SEARCHING
        self.enabled = False
        self.last_target_time = rospy.Time(0)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.state_pub = rospy.Publisher("/jzzj_visual_servo/state", String,
                                          queue_size=1, latch=True)
        self.ready_pub = rospy.Publisher("/jzzj_visual_servo/grasp_ready", Bool,
                                         queue_size=1, latch=True)
        self.visible_pub = rospy.Publisher(
            "/jzzj_visual_servo/target_visible_state", Bool,
            queue_size=1, latch=True)

        rospy.Subscriber("/jzzj_visual_servo/target_visible", Bool,
                         self._visible_callback, queue_size=1)
        rospy.Subscriber("/jzzj_visual_servo/target_error", Vector3,
                         self._error_callback, queue_size=1)
        rospy.Subscriber("/jzzj_visual_servo/enable", Bool,
                         self._enable_callback, queue_size=1)
        rospy.Subscriber("/jzzj_visual_servo/stop", Empty,
                         self._stop_callback, queue_size=1)
        if ObjDets is not None and self.use_detector:
            rospy.Subscriber(self.detector_topic, ObjDets,
                             self._detections_callback, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.loop_rate), self._tick)
        self._publish_status()

    def _visible_callback(self, message):
        self.visible = bool(message.data)
        self.last_target_time = rospy.Time.now()

    def _error_callback(self, message):
        self.error = message
        self.visible = True
        self.last_target_time = rospy.Time.now()

    def _detections_callback(self, message):
        candidates = []
        for detection in message.dets:
            if float(detection.score) < self.min_score:
                continue
            label = str(detection.label).strip().lower()
            if self.target_color and label and self.target_color not in label:
                if not self.target_class_ids or int(detection.class_id) not in self.target_class_ids:
                    continue
            if self.target_class_ids and int(detection.class_id) not in self.target_class_ids:
                continue
            candidates.append(detection)

        if not candidates:
            self.visible = False
            return

        detection = max(candidates, key=lambda value: float(value.score))
        center_x = float(detection.bbox.x)
        center_y = float(detection.bbox.y)
        bbox_height = max(1.0, float(detection.bbox.h))
        self.error.x = (center_x - self.image_width * 0.5) / (self.image_width * 0.5)
        self.error.y = (center_y - self.image_height * 0.5) / (self.image_height * 0.5)
        self.error.z = self.reference_bbox_height / bbox_height - 1.0
        self.visible = True
        self.last_target_time = rospy.Time.now()

    def _enable_callback(self, message):
        self.enabled = bool(message.data)
        if not self.enabled:
            self.state = self.STOPPED
            self._stop_output()
            self._publish_status()

    def _stop_callback(self, _message):
        self.enabled = False
        self.state = self.STOPPED
        self._stop_output()
        self._publish_status()

    def _tick(self, _event):
        now = rospy.Time.now()
        if self.visible and (now - self.last_target_time).to_sec() > self.target_timeout:
            self.visible = False

        self.visible_pub.publish(Bool(data=self.visible))
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
        command.angular.z = clamp(
            -self.horizontal_gain * horizontal,
            -self.max_angular_z, self.max_angular_z)
        command.linear.y = clamp(
            -self.vertical_gain * vertical,
            -self.max_linear_y, self.max_linear_y)
        command.linear.x = clamp(
            self.depth_gain * depth,
            -self.max_linear_x, self.max_linear_x)
        self.cmd_pub.publish(command)
        self.ready_pub.publish(Bool(data=False))
        self._publish_status()

    @staticmethod
    def _outside_deadband(value, deadband):
        return 0.0 if abs(value) <= deadband else value

    def _stop_output(self):
        self.cmd_pub.publish(Twist())
        self.ready_pub.publish(Bool(data=False))

    def _publish_status(self):
        payload = "state={};enabled={};visible={};target_color={};error=({:.3f},{:.3f},{:.3f})".format(
            self.state, self.enabled, self.visible, self.target_color,
            self.error.x, self.error.y, self.error.z)
        self.state_pub.publish(String(data=payload))


if __name__ == "__main__":
    try:
        JzzjVisualServoNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
