#!/usr/bin/env python3
from __future__ import print_function

import math

import rospy
from geometry_msgs.msg import JointState, PoseStamped, Twist
from std_msgs.msg import Bool, Empty, String

try:
    from user_msgs.msg import CarCmd
except ImportError:
    CarCmd = None


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def pose_yaw(pose):
    q = pose.pose.orientation
    sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
    cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sin_yaw, cos_yaw)


class JzzjMissonNode(object):
    WAIT_TAG = "WAIT_TAG"
    LOCALIZE = "LOCALIZE"
    MOVE_TO_PICK = "MOVE_TO_PICK"
    PICK_ALIGN = "PICK_ALIGN"
    SEARCH_OBJECT = "SEARCH_OBJECT"
    VISUAL_APPROACH = "VISUAL_APPROACH"
    GRASP = "GRASP"
    VERIFY_GRASP = "VERIFY_GRASP"
    MOVE_TO_DROP = "MOVE_TO_DROP"
    DROP_ALIGN = "DROP_ALIGN"
    RELEASE = "RELEASE"
    FINISH = "FINISH"

    def __init__(self):
        rospy.init_node("jzzj_misson_node")

        self.loop_rate = max(1.0, float(rospy.get_param("~loop_rate", 10.0)))
        self.stage_timeout = max(0.0, float(rospy.get_param("~stage_timeout", 60.0)))
        self.pose_timeout = max(0.1, float(rospy.get_param("~pose_timeout", 0.8)))
        self.require_both_tags = bool(rospy.get_param("~require_both_tags", False))
        self.require_tag_before_motion = bool(rospy.get_param("~require_tag_before_motion", True))
        self.require_grasp_confirmation = bool(rospy.get_param("~require_grasp_confirmation", True))
        self.require_drop_confirmation = bool(rospy.get_param("~require_drop_confirmation", False))
        self.target_color = str(rospy.get_param("~target_color", ""))

        self.pick_pose = self._read_target_pose("~pick_pose")
        self.drop_pose = self._read_target_pose("~drop_pose")
        self.drop_align_pose = self._read_target_pose("~drop_align_pose", self.drop_pose)
        self.position_tolerance = float(rospy.get_param("~position_tolerance", 0.06))
        self.yaw_tolerance = math.radians(float(rospy.get_param("~yaw_tolerance_deg", 6.0)))
        self.kp_position = float(rospy.get_param("~kp_position", 0.9))
        self.kp_yaw = float(rospy.get_param("~kp_yaw", 1.5))
        self.max_linear = abs(float(rospy.get_param("~max_linear", 0.35)))
        self.max_angular = abs(float(rospy.get_param("~max_angular", 0.8)))
        self.release_settle_time = max(0.0, float(rospy.get_param("~release_settle_time", 2.0)))
        self.release_pose_deg = list(rospy.get_param(
            "~release_pose_deg", [0.0, 0.0, 90.0, 90.0, 0.0, 60.0]))
        self.grasp_pre_pose_deg = list(rospy.get_param(
            "~grasp_pre_pose_deg", [-60.0, 35.0, 90.0, 40.0, -70.0, 60.0]))
        self.grasp_close_pose_deg = list(rospy.get_param(
            "~grasp_close_pose_deg", [-60.0, 35.0, 90.0, 40.0, -70.0, 30.0]))
        self.grasp_lift_pose_deg = list(rospy.get_param(
            "~grasp_lift_pose_deg", [0.0, 0.0, 70.0, 0.0, 0.0, 30.0]))
        self.grasp_pre_settle_time = max(0.0, float(
            rospy.get_param("~grasp_pre_settle_time", 1.0)))
        self.grasp_close_settle_time = max(0.0, float(
            rospy.get_param("~grasp_close_settle_time", 2.0)))
        self.assume_grasp_success_after_sequence = bool(rospy.get_param(
            "~assume_grasp_success_after_sequence", False))

        default_stages = [
            self.WAIT_TAG, self.LOCALIZE, self.MOVE_TO_PICK,
            self.PICK_ALIGN, self.SEARCH_OBJECT, self.VISUAL_APPROACH,
            self.GRASP, self.VERIFY_GRASP, self.MOVE_TO_DROP,
            self.DROP_ALIGN, self.RELEASE, self.FINISH]
        self.stages = [str(value) for value in rospy.get_param("~stages", default_stages)]
        if not self.stages:
            self.stages = default_stages

        self.state = "READY"
        self.stage_index = -1
        self.stage_started_at = rospy.Time.now()
        self.tag_ready_input = False
        self.visual_ready = False
        self.target_visible = False
        self.grasp_confirmed = False
        self.drop_confirmed = False
        self.grasp_step = 0
        self.grasp_sequence_done = False
        self.release_sent = False

        self.front_pose_filtered = None
        self.front_pose_orig = None
        self.back_pose = None
        self.front_pose_filtered_time = rospy.Time(0)
        self.front_pose_orig_time = rospy.Time(0)
        self.back_pose_time = rospy.Time(0)

        self.state_pub = rospy.Publisher("/jzzj_misson/state", String, queue_size=1, latch=True)
        self.finish_pub = rospy.Publisher("/jzzj_misson/finish", Bool, queue_size=1, latch=True)
        self.tag_state_pub = rospy.Publisher("/jzzj_misson/tag_ready_state", Bool,
                                              queue_size=1, latch=True)
        self.pose_pub = rospy.Publisher("/jzzj_misson/car_pose", PoseStamped, queue_size=1)
        self.motion_status_pub = rospy.Publisher("/jzzj_misson/motion_command", String, queue_size=1)
        self.arm_status_pub = rospy.Publisher("/jzzj_misson/left_arm_command", String, queue_size=1)
        self.visual_enable_pub = rospy.Publisher(
            "/jzzj_visual_servo/enable", Bool, queue_size=1, latch=True)
        self.cmd_vel_pub = rospy.Publisher(
            rospy.get_param("~cmd_vel_topic", "/cmd_vel"), Twist, queue_size=1)

        self.car_cmd_pub = None
        self.release_pub = rospy.Publisher("/arm_left/arm_joint_states", JointState, queue_size=1)
        if CarCmd is not None:
            self.car_cmd_pub = rospy.Publisher(
                rospy.get_param("~car_cmd_topic", "/car_cmd"), CarCmd, queue_size=1)

        rospy.Subscriber("/jzzj_misson/start", Empty, self._start_callback, queue_size=1)
        rospy.Subscriber("/jzzj_misson/stop", Empty, self._stop_callback, queue_size=1)
        rospy.Subscriber("/jzzj_misson/advance", Empty, self._advance_callback, queue_size=1)
        rospy.Subscriber("/jzzj_misson/tag_ready", Bool, self._tag_callback, queue_size=1)
        rospy.Subscriber("/jzzj_visual_servo/grasp_ready", Bool,
                         self._visual_callback, queue_size=1)
        rospy.Subscriber("/jzzj_visual_servo/target_visible_state", Bool,
                         self._target_visible_callback, queue_size=1)
        rospy.Subscriber("/jzzj_misson/grasp_confirmed", Bool,
                         self._grasp_callback, queue_size=1)
        rospy.Subscriber("/jzzj_misson/drop_confirmed", Bool,
                         self._drop_callback, queue_size=1)
        rospy.Subscriber("/front_car_pose_filt_map", PoseStamped,
                         self._front_pose_filtered_callback, queue_size=1)
        rospy.Subscriber("/front_car_pose_orig_map", PoseStamped,
                         self._front_pose_orig_callback, queue_size=1)
        rospy.Subscriber("/back_car_pose_filt_map", PoseStamped,
                         self._back_pose_callback, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.loop_rate), self._tick)
        self._publish_state()
        self.finish_pub.publish(Bool(data=False))
        self.visual_enable_pub.publish(Bool(data=False))
        if rospy.get_param("~auto_start", False):
            rospy.Timer(rospy.Duration(0.2), self._auto_start, oneshot=True)

    @staticmethod
    def _read_target_pose(name, fallback=None):
        value = rospy.get_param(name, fallback)
        if value is None and fallback is not None:
            value = fallback
        if value is None or len(value) < 3:
            return None
        return (float(value[0]), float(value[1]), float(value[2]))

    def _auto_start(self, _event):
        self._start_callback(Empty())

    def _start_callback(self, _message):
        if self.state == "RUNNING":
            return
        if self.state == "COMPLETED":
            rospy.logwarn("mission is already completed")
            return
        self.stage_index = 0
        self.stage_started_at = rospy.Time.now()
        self.tag_ready_input = False
        self.visual_ready = False
        self.target_visible = False
        self.grasp_confirmed = False
        self.drop_confirmed = False
        self.grasp_step = 0
        self.grasp_sequence_done = False
        self.release_sent = False
        self.state = "RUNNING"
        self.finish_pub.publish(Bool(data=False))
        self._publish_state()
        self._publish_stage_command()

    def _stop_callback(self, _message):
        if self.state in ("RUNNING", "READY"):
            self.state = "STOPPED"
            self._publish_state()
            self._publish_safe_stop()

    def _advance_callback(self, _message):
        if self.state == "RUNNING":
            self._advance_stage(force=True)

    def _tag_callback(self, message):
        self.tag_ready_input = bool(message.data)

    def _visual_callback(self, message):
        self.visual_ready = bool(message.data)

    def _target_visible_callback(self, message):
        self.target_visible = bool(message.data)

    def _grasp_callback(self, message):
        self.grasp_confirmed = bool(message.data)

    def _drop_callback(self, message):
        self.drop_confirmed = bool(message.data)

    def _front_pose_filtered_callback(self, message):
        self.front_pose_filtered = message
        self.front_pose_filtered_time = rospy.Time.now()

    def _front_pose_orig_callback(self, message):
        self.front_pose_orig = message
        self.front_pose_orig_time = rospy.Time.now()

    def _back_pose_callback(self, message):
        self.back_pose = message
        self.back_pose_time = rospy.Time.now()

    def _tick(self, _event):
        self._publish_fused_pose()
        if self.state != "RUNNING":
            return

        if self.stage_timeout > 0.0:
            elapsed = (rospy.Time.now() - self.stage_started_at).to_sec()
            if elapsed >= self.stage_timeout:
                self._fault("stage timeout: {}".format(self._current_stage()))
                return

        stage = self._current_stage()
        tag_ready = self._tag_ready()
        self.tag_state_pub.publish(Bool(data=tag_ready))

        if stage == self.WAIT_TAG:
            if tag_ready or self.tag_ready_input:
                self._advance_stage()
        elif stage == self.LOCALIZE:
            if tag_ready or self.tag_ready_input:
                self._advance_stage()
        elif stage == self.MOVE_TO_PICK:
            if self._drive_to(self.pick_pose):
                self._advance_stage()
        elif stage == self.PICK_ALIGN:
            self._advance_stage()
        elif stage == self.SEARCH_OBJECT:
            if self.target_visible:
                self._advance_stage()
        elif stage == self.VISUAL_APPROACH:
            if self.visual_ready:
                self._advance_stage()
        elif stage == self.GRASP:
            self._run_grasp_sequence()
            if self.grasp_sequence_done and (
                    self.grasp_confirmed or self.assume_grasp_success_after_sequence):
                self.grasp_confirmed = True
                self._advance_stage()
        elif stage == self.VERIFY_GRASP:
            if self.grasp_confirmed or not self.require_grasp_confirmation:
                self._advance_stage()
        elif stage == self.MOVE_TO_DROP:
            if self._drive_to(self.drop_pose):
                self._advance_stage()
        elif stage == self.DROP_ALIGN:
            if self._drive_to(self.drop_align_pose):
                self._advance_stage()
        elif stage == self.RELEASE:
            self._publish_release_once()
            if self.require_drop_confirmation:
                if self.drop_confirmed:
                    self._advance_stage()
            elif (rospy.Time.now() - self.stage_started_at).to_sec() >= self.release_settle_time:
                self._advance_stage()
        elif stage == self.FINISH:
            self._advance_stage()

    def _advance_stage(self, force=False):
        current = self._current_stage()
        if not force:
            if current == self.MOVE_TO_PICK and self.require_tag_before_motion and not self._tag_ready():
                return
            if current == self.VERIFY_GRASP and self.require_grasp_confirmation and not self.grasp_confirmed:
                return
            if current == self.RELEASE and self.require_drop_confirmation and not self.drop_confirmed:
                return

        if self.stage_index + 1 >= len(self.stages):
            self.state = "COMPLETED"
            self._publish_state()
            self._publish_safe_stop()
            self.finish_pub.publish(Bool(data=True))
            rospy.loginfo("JZZJ mission completed")
            return

        self.stage_index += 1
        self.stage_started_at = rospy.Time.now()
        self.release_sent = False
        self._publish_state()
        self._publish_stage_command()

    def _current_stage(self):
        if 0 <= self.stage_index < len(self.stages):
            return self.stages[self.stage_index]
        return "IDLE"

    def _tag_ready(self):
        now = rospy.Time.now()
        front_valid = self._current_front_pose(now) is not None
        back_valid = self.back_pose is not None and (now - self.back_pose_time).to_sec() <= self.pose_timeout
        if self.require_both_tags:
            return front_valid and back_valid
        return front_valid or back_valid

    def _fused_pose(self):
        now = rospy.Time.now()
        front_pose = self._current_front_pose(now)
        front_valid = front_pose is not None
        back_valid = self.back_pose is not None and (now - self.back_pose_time).to_sec() <= self.pose_timeout
        poses = []
        if front_valid:
            poses.append(front_pose)
        if back_valid:
            poses.append(self.back_pose)
        if not poses:
            return None

        result = PoseStamped()
        result.header.stamp = rospy.Time.now()
        result.header.frame_id = "map"
        result.pose.position.x = sum(p.pose.position.x for p in poses) / len(poses)
        result.pose.position.y = sum(p.pose.position.y for p in poses) / len(poses)
        result.pose.position.z = sum(p.pose.position.z for p in poses) / len(poses)
        yaws = [pose_yaw(p) for p in poses]
        result_yaw = math.atan2(sum(math.sin(yaw) for yaw in yaws),
                                sum(math.cos(yaw) for yaw in yaws))
        result.pose.orientation.z = math.sin(result_yaw / 2.0)
        result.pose.orientation.w = math.cos(result_yaw / 2.0)
        return result

    def _current_front_pose(self, now):
        if (self.front_pose_filtered is not None and
                (now - self.front_pose_filtered_time).to_sec() <= self.pose_timeout):
            return self.front_pose_filtered
        if (self.front_pose_orig is not None and
                (now - self.front_pose_orig_time).to_sec() <= self.pose_timeout):
            return self.front_pose_orig
        return None

    def _publish_fused_pose(self):
        pose = self._fused_pose()
        if pose is not None:
            self.pose_pub.publish(pose)

    def _drive_to(self, target):
        if target is None:
            self._fault("target pose is not configured")
            return False
        pose = self._fused_pose()
        if pose is None:
            self._publish_zero_motion("localization lost")
            return False

        dx = target[0] - pose.pose.position.x
        dy = target[1] - pose.pose.position.y
        yaw = pose_yaw(pose)
        distance = math.hypot(dx, dy)
        yaw_error = wrap_angle(target[2] - yaw)
        if distance <= self.position_tolerance and abs(yaw_error) <= self.yaw_tolerance:
            self._publish_zero_motion("arrived")
            return True

        body_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        body_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        command = Twist()
        command.linear.x = clamp(self.kp_position * body_x, -self.max_linear, self.max_linear)
        command.linear.y = clamp(self.kp_position * body_y, -self.max_linear, self.max_linear)
        command.angular.z = clamp(self.kp_yaw * yaw_error, -self.max_angular, self.max_angular)
        self.cmd_vel_pub.publish(command)
        self._publish_car_cmd(CarCmd.NAVIGATION if CarCmd else None)
        self.motion_status_pub.publish(String(
            data="stage={};dx={:.3f};dy={:.3f};dyaw={:.3f}".format(
                self._current_stage(), dx, dy, yaw_error)))
        return False

    def _publish_car_cmd(self, command):
        if self.car_cmd_pub is None or command is None:
            return
        message = CarCmd()
        message.cmd = command
        self.car_cmd_pub.publish(message)

    def _publish_zero_motion(self, reason):
        self.cmd_vel_pub.publish(Twist())
        self._publish_car_cmd(CarCmd.HOLD if CarCmd else None)
        self.motion_status_pub.publish(String(data="STOP:{}".format(reason)))

    def _publish_arm_pose(self, pose_deg, label):
        command = JointState()
        command.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        command.position = [math.radians(float(value)) for value in pose_deg]
        command.header.stamp = rospy.Time.now()
        self.release_pub.publish(command)
        self.arm_status_pub.publish(String(data=label))

    def _run_grasp_sequence(self):
        elapsed = (rospy.Time.now() - self.stage_started_at).to_sec()
        if self.grasp_step == 0:
            self._publish_arm_pose(self.grasp_pre_pose_deg, "GRASP_PREPARE")
            self.grasp_step = 1
            self.stage_started_at = rospy.Time.now()
            return
        if self.grasp_step == 1 and elapsed >= self.grasp_pre_settle_time:
            self._publish_arm_pose(self.grasp_close_pose_deg, "GRASP_CLOSE")
            self.grasp_step = 2
            self.stage_started_at = rospy.Time.now()
            return
        if self.grasp_step == 2 and elapsed >= self.grasp_close_settle_time:
            self._publish_arm_pose(self.grasp_lift_pose_deg, "GRASP_LIFT")
            self.grasp_step = 3
            self.grasp_sequence_done = True
            self.stage_started_at = rospy.Time.now()

    def _publish_release_once(self):
        if self.release_sent:
            return
        self.release_sent = True
        command = JointState()
        command.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        command.position = [math.radians(float(value)) for value in self.release_pose_deg]
        command.header.stamp = rospy.Time.now()
        self.release_pub.publish(command)
        self.arm_status_pub.publish(String(data="RELEASE"))

    def _publish_stage_command(self):
        stage = self._current_stage()
        visual_enabled = stage in (self.PICK_ALIGN, self.SEARCH_OBJECT, self.VISUAL_APPROACH)
        self.visual_enable_pub.publish(Bool(data=visual_enabled))
        if stage in (self.MOVE_TO_PICK, self.MOVE_TO_DROP, self.DROP_ALIGN):
            self.motion_status_pub.publish(String(data="stage={}".format(stage)))
        elif stage in (self.PICK_ALIGN, self.SEARCH_OBJECT, self.VISUAL_APPROACH):
            self.arm_status_pub.publish(String(data="VISUAL_ALIGN;color={}".format(self.target_color)))
        elif stage == self.GRASP:
            self.arm_status_pub.publish(String(data="GRASP"))

    def _publish_safe_stop(self):
        self._publish_zero_motion("safe_stop")
        self.arm_status_pub.publish(String(data="SAFE"))
        self.visual_enable_pub.publish(Bool(data=False))

    def _publish_state(self):
        payload = "state={};stage={};stage_index={};tag_ready={};target_visible={};visual_ready={};grasp_confirmed={};drop_confirmed={};target_color={}".format(
            self.state, self._current_stage(), self.stage_index,
            self._tag_ready(), self.target_visible, self.visual_ready,
            self.grasp_confirmed, self.drop_confirmed, self.target_color)
        self.state_pub.publish(String(data=payload))

    def _fault(self, reason):
        if self.state == "FAULT":
            return
        self.state = "FAULT"
        self._publish_state()
        self._publish_safe_stop()
        self.finish_pub.publish(Bool(data=False))
        rospy.logerr("JZZJ mission fault: %s", reason)


if __name__ == "__main__":
    try:
        JzzjMissonNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
