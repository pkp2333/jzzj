#!/usr/bin/env python3
from __future__ import print_function

import rospy
from std_msgs.msg import Bool, Empty, String


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
        self.require_tag_before_motion = rospy.get_param(
            "~require_tag_before_motion", True)
        self.require_grasp_confirmation = rospy.get_param(
            "~require_grasp_confirmation", True)
        self.require_drop_confirmation = rospy.get_param(
            "~require_drop_confirmation", False)
        self.target_color = str(rospy.get_param("~target_color", ""))

        stages = rospy.get_param("~stages", [
            self.WAIT_TAG, self.LOCALIZE, self.MOVE_TO_PICK,
            self.PICK_ALIGN, self.SEARCH_OBJECT, self.VISUAL_APPROACH,
            self.GRASP, self.VERIFY_GRASP, self.MOVE_TO_DROP,
            self.DROP_ALIGN, self.RELEASE, self.FINISH])
        self.stages = [str(stage) for stage in stages] or [self.FINISH]

        self.state = "IDLE"
        self.stage_index = -1
        self.stage_started_at = None
        self.tag_ready = False
        self.visual_ready = False
        self.grasp_confirmed = False
        self.drop_confirmed = False

        self.state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
        self.motion_pub = rospy.Publisher("~motion_command", String, queue_size=1)
        self.arm_pub = rospy.Publisher("~left_arm_command", String, queue_size=1)
        self.visual_enable_pub = rospy.Publisher(
            "~visual_enable", Bool, queue_size=1, latch=True)

        rospy.Subscriber("~start", Empty, self._start_callback, queue_size=1)
        rospy.Subscriber("~stop", Empty, self._stop_callback, queue_size=1)
        rospy.Subscriber("~advance", Empty, self._advance_callback, queue_size=1)
        rospy.Subscriber("~tag_ready", Bool, self._tag_callback, queue_size=1)
        rospy.Subscriber("~visual_ready", Bool,
                         self._visual_callback, queue_size=1)
        rospy.Subscriber("~grasp_confirmed", Bool,
                         self._grasp_callback, queue_size=1)
        rospy.Subscriber("~drop_confirmed", Bool,
                         self._drop_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.loop_rate), self._tick)

        self._set_state("READY")
        if rospy.get_param("~auto_start", False):
            rospy.Timer(rospy.Duration(0.2), self._auto_start, oneshot=True)

    def _auto_start(self, _event):
        self._start_callback(Empty())

    def _start_callback(self, _message):
        if self.state in ("RUNNING", "COMPLETED"):
            rospy.logwarn("Cannot start while mission is %s", self.state)
            return
        self.stage_index = 0
        self.stage_started_at = rospy.Time.now()
        self.tag_ready = False
        self.visual_ready = False
        self.grasp_confirmed = False
        self.drop_confirmed = False
        self._set_state("RUNNING")
        self._publish_stage_command()

    def _stop_callback(self, _message):
        if self.state == "RUNNING":
            self._set_state("STOPPED")
            self._publish_safe_stop()

    def _advance_callback(self, _message):
        if self.state == "RUNNING":
            self._advance_stage()

    def _tag_callback(self, message):
        self.tag_ready = bool(message.data)

    def _grasp_callback(self, message):
        self.grasp_confirmed = bool(message.data)

    def _visual_callback(self, message):
        self.visual_ready = bool(message.data)

    def _drop_callback(self, message):
        self.drop_confirmed = bool(message.data)

    def _tick(self, _event):
        if self.state != "RUNNING":
            return

        if self.stage_timeout > 0.0:
            elapsed = (rospy.Time.now() - self.stage_started_at).to_sec()
            if elapsed >= self.stage_timeout:
                self._fault("stage timeout: {}".format(self._current_stage()))
                return

        stage = self._current_stage()
        if stage in (self.WAIT_TAG, self.LOCALIZE) and self.tag_ready:
            self._advance_stage()
        elif stage in (self.PICK_ALIGN, self.VISUAL_APPROACH) and self.visual_ready:
            self._advance_stage()
        elif stage == self.VERIFY_GRASP and self.grasp_confirmed:
            self._advance_stage()
        elif stage == self.RELEASE and self.drop_confirmed:
            self._advance_stage()

    def _advance_stage(self):
        if self.stage_index + 1 >= len(self.stages):
            self.state = "COMPLETED"
            self._publish_state()
            self._publish_safe_stop()
            rospy.loginfo("JZZJ mission completed")
            return

        current = self._current_stage()
        if current == self.MOVE_TO_PICK and self.require_tag_before_motion and not self.tag_ready:
            self._fault("cannot move to pick before AprilTag is ready")
            return
        if current == self.VERIFY_GRASP and self.require_grasp_confirmation and not self.grasp_confirmed:
            self._fault("left-arm grasp was not confirmed")
            return
        if current == self.RELEASE and self.require_drop_confirmation and not self.drop_confirmed:
            self._fault("drop was not confirmed")
            return

        self.stage_index += 1
        self.stage_started_at = rospy.Time.now()
        self._publish_state()
        self._publish_stage_command()

    def _current_stage(self):
        if 0 <= self.stage_index < len(self.stages):
            return self.stages[self.stage_index]
        return "IDLE"

    def _publish_stage_command(self):
        stage = self._current_stage()
        payload = "stage={};target_color={}".format(stage, self.target_color)
        self.visual_enable_pub.publish(Bool(data=stage in (
            self.PICK_ALIGN, self.SEARCH_OBJECT, self.VISUAL_APPROACH)))
        if stage in (self.MOVE_TO_PICK, self.MOVE_TO_DROP, self.DROP_ALIGN):
            self.motion_pub.publish(String(data=payload))
        elif stage in (self.PICK_ALIGN, self.SEARCH_OBJECT,
                       self.VISUAL_APPROACH, self.GRASP, self.RELEASE):
            self.arm_pub.publish(String(data=payload))

    def _publish_safe_stop(self):
        self.motion_pub.publish(String(data="STOP"))
        self.arm_pub.publish(String(data="SAFE"))
        self.visual_enable_pub.publish(Bool(data=False))

    def _set_state(self, state):
        self.state = state
        self._publish_state()

    def _publish_state(self):
        payload = "state={};stage={};stage_index={};tag_ready={};visual_ready={};grasp_confirmed={};drop_confirmed={};target_color={}".format(
            self.state, self._current_stage(), self.stage_index,
            self.tag_ready, self.visual_ready, self.grasp_confirmed,
            self.drop_confirmed, self.target_color)
        self.state_pub.publish(String(data=payload))

    def _fault(self, reason):
        self.state = "FAULT"
        self._publish_state()
        self._publish_safe_stop()
        rospy.logerr("JZZJ mission fault: %s", reason)


if __name__ == "__main__":
    try:
        JzzjMissonNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
