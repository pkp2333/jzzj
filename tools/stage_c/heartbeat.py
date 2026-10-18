#!/usr/bin/env python3
"""阶段C交接心跳: 每条消息刷新header.stamp(2Hz). 用法: heartbeat.py 会话ID"""
import sys, rospy
from jzzj_grasp_interfaces.msg import StageCContext
sid=sys.argv[1] if len(sys.argv)>1 else 'grasp-001'
rospy.init_node('stage_c_heartbeat')
pub=rospy.Publisher('/stage_c/context',StageCContext,queue_size=1)
rate=rospy.Rate(2)
while not rospy.is_shutdown():
    m=StageCContext()
    m.header.stamp=rospy.Time.now()
    m.session_id=sid
    m.base_locked=True
    m.arm_control_granted=True
    m.stop_requested=False
    pub.publish(m)
    rate.sleep()
