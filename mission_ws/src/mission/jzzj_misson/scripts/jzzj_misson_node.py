#!/usr/bin/env python3
from __future__ import print_function

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
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
        """初始化任务节点。

        初始化流程:
            1. 从ROS参数服务器加载配置(循环频率、超时、守卫开关、目标颜色、阶段序列)
            2. 初始化运行时状态变量(顶层状态、阶段索引、各传感器标志位)
            3. 创建发布器: 状态广播、底盘运动命令、左臂控制命令、视觉伺服使能
            4. 创建订阅器: 启动/停止/推进控制、AprilTag/视觉/抓取/放置确认信号
            5. 启动定时器驱动主循环(_tick)
            6. 进入READY状态，若auto_start为True则延迟0.2秒自动启动任务

        ROS参数:
            ~loop_rate                  (float, 默认10.0)  主循环频率Hz
            ~stage_timeout              (float, 默认60.0)   单阶段超时秒数，0=不限
            ~require_tag_before_motion  (bool,  默认True)   运动前是否要求Tag就绪
            ~require_grasp_confirmation (bool,  默认True)   是否要求抓取确认
            ~require_drop_confirmation  (bool,  默认False)  是否要求放置确认
            ~target_color               (str,   默认"")     目标果实颜色
            ~stages                     (list,  默认完整12阶段) 任务阶段序列
            ~auto_start                 (bool,  默认False)  是否自动启动任务
        """
        # 初始化ROS节点
        rospy.init_node("jzzj_misson_node")

        # ---- 参数加载 ----
        # 控制循环频率(Hz)，下限为1.0防止除零
        self.loop_rate = max(1.0, float(rospy.get_param("~loop_rate", 10.0)))
        # 单阶段超时时间(秒)，超时则进入FAULT状态，0表示不超时
        self.stage_timeout = max(0.0, float(rospy.get_param("~stage_timeout", 60.0)))
        # 运动前是否要求AprilTag已就绪
        self.require_tag_before_motion = rospy.get_param(
            "~require_tag_before_motion", True)
        # 抓取后是否要求确认抓取成功
        self.require_grasp_confirmation = rospy.get_param(
            "~require_grasp_confirmation", True)
        # 放置后是否要求确认放置成功
        self.require_drop_confirmation = rospy.get_param(
            "~require_drop_confirmation", False)
        # 目标果实颜色，用于视觉伺服筛选
        self.target_color = str(rospy.get_param("~target_color", ""))
        # 是否启用外部阶段C(视觉抓取)适配器
        self.external_stage_c = rospy.get_param("~external_stage_c", False)
        self.stage_c = None
        if self.external_stage_c:
            from stage_c_adapter import StageCAdapter
            self.stage_c = StageCAdapter()
            rospy.on_shutdown(self.stage_c.cancel)

        # 任务阶段序列，可通过参数自定义，默认为完整采摘流程
        stages = rospy.get_param("~stages", [
            self.WAIT_TAG, self.LOCALIZE, self.MOVE_TO_PICK,
            self.PICK_ALIGN, self.SEARCH_OBJECT, self.VISUAL_APPROACH,
            self.GRASP, self.VERIFY_GRASP, self.MOVE_TO_DROP,
            self.DROP_ALIGN, self.RELEASE, self.FINISH])
        self.stages = [str(stage) for stage in stages] or [self.FINISH]

        # ---- 运行时状态 ----
        self.state = "IDLE"             # 顶层状态机: IDLE/READY/RUNNING/STOPPED/COMPLETED/FAULT
        self.stage_index = -1           # 当前阶段索引，-1表示未开始
        self.stage_started_at = None    # 当前阶段开始时间戳，用于超时检测
        self.tag_ready = False          # AprilTag定位是否就绪
        self.visual_ready = False       # 视觉伺服是否就绪(对齐/接近完成)
        self.grasp_confirmed = False    # 机械臂抓取是否已确认成功
        self.drop_confirmed = False     # 放置是否已确认完成

        # ---- 发布器 ----
        # 状态广播(latched，新订阅者可立即获取最新状态)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
        # 底盘运动命令(导航/对齐)
        self.motion_pub = rospy.Publisher("~motion_command", String, queue_size=1)
        # 左臂控制命令(对齐/搜索/接近/抓取/释放)
        self.arm_pub = rospy.Publisher("~left_arm_command", String, queue_size=1)
        # 视觉伺服使能开关(latched)
        self.visual_enable_pub = rospy.Publisher(
            "~visual_enable", Bool, queue_size=1, latch=True)

        # ---- 订阅器 ----
        rospy.Subscriber("~start", Empty, self._start_callback, queue_size=1)       # 启动任务
        rospy.Subscriber("~stop", Empty, self._stop_callback, queue_size=1)         # 停止任务
        rospy.Subscriber("~advance", Empty, self._advance_callback, queue_size=1)   # 手动推进阶段
        rospy.Subscriber("~tag_ready", Bool, self._tag_callback, queue_size=1)      # AprilTag就绪信号
        rospy.Subscriber("~visual_ready", Bool,
                         self._visual_callback, queue_size=1)                       # 视觉伺服就绪信号
        rospy.Subscriber("~grasp_confirmed", Bool,
                         self._grasp_callback, queue_size=1)                        # 抓取确认信号
        rospy.Subscriber("~drop_confirmed", Bool,
                         self._drop_callback, queue_size=1)                         # 放置确认信号

        # 定时器驱动主循环_tick，频率为loop_rate
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.loop_rate), self._tick)

        # 初始化完成，进入READY状态
        self._set_state("READY")
        # 若配置了自动启动，延迟0.2秒后触发_start_callback
        if rospy.get_param("~auto_start", False):
            rospy.Timer(rospy.Duration(0.2), self._auto_start, oneshot=True)

    def _auto_start(self, _event):
        self._start_callback(Empty())

    def _start_callback(self, _message):
        """启动任务回调，由~start话题触发。

        将状态切换为RUNNING，重置所有阶段和标志位，并发布首阶段命令。
        若当前已在RUNNING或COMPLETED状态则拒绝启动并打印警告。
        若启用了外部阶段C(external_stage_c)，需等待其取消完成并先重置适配器。
        """
        # 外部阶段C未完全取消前禁止启动
        if self.stage_c and self.stage_c.active:
            rospy.logwarn("Cannot start before stage C cancellation completes")
            return
        # 防止重复启动：任务运行中或已完成时拒绝
        if self.state in ("RUNNING", "COMPLETED"):
            rospy.logwarn("Cannot start while mission is %s", self.state)
            return
        # 重置外部阶段C适配器状态
        if self.stage_c:
            try:
                self.stage_c.reset()
            except Exception as exc:
                rospy.logerr(str(exc))
                return
        # 重置阶段索引到第一个阶段
        self.stage_index = 0
        # 记录当前阶段开始时间，用于超时检测
        self.stage_started_at = rospy.Time.now()
        # 清除所有传感器标志位，等待新一轮反馈
        self.tag_ready = False
        self.visual_ready = False
        self.grasp_confirmed = False
        self.drop_confirmed = False
        # 切换顶层状态为RUNNING
        self._set_state("RUNNING")
        # 发布第一个阶段的控制命令
        self._publish_stage_command()

    def _stop_callback(self, _message):
        if self.state == "RUNNING":
            self._set_state("STOPPED")
            self._publish_safe_stop()

    def _advance_callback(self, _message):
        if self.stage_c and self._current_stage() in (self.PICK_ALIGN,self.SEARCH_OBJECT,self.VISUAL_APPROACH,self.GRASP,self.VERIFY_GRASP):
            rospy.logwarn("Manual advance disabled while external stage C owns grasp stages")
            return
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
        if self.stage_c and (self.stage_c.active or self.stage_c.done):
            self.stage_c.tick(self)
            return
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
        if self.stage_c and stage in (self.PICK_ALIGN,self.SEARCH_OBJECT,self.VISUAL_APPROACH,self.GRASP,self.VERIFY_GRASP):
            self.visual_enable_pub.publish(Bool(data=False))
            if stage == self.PICK_ALIGN and not self.stage_c.active:
                try:
                    self.stage_c.start(self.target_color)
                except Exception as exc:
                    self._fault(str(exc))
            return
        self.visual_enable_pub.publish(Bool(data=stage in (
            self.PICK_ALIGN, self.SEARCH_OBJECT, self.VISUAL_APPROACH)))
        if stage in (self.MOVE_TO_PICK, self.MOVE_TO_DROP, self.DROP_ALIGN):
            self.motion_pub.publish(String(data=payload))
        elif stage in (self.PICK_ALIGN, self.SEARCH_OBJECT,
                       self.VISUAL_APPROACH, self.GRASP, self.RELEASE):
            self.arm_pub.publish(String(data=payload))

    def _publish_safe_stop(self):
        if self.stage_c:
            self.stage_c.cancel()
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