#!/usr/bin/env python3
"""通过轨迹Action的jog(与arm_trajectory节点协同,无需租约):
用法: python3 action_jog.py 关节号 弧度增量   (0.1rad≈5.7°, 0.05rad≈3°)
示例: python3 action_jog.py 2 0.1   python3 action_jog.py 1 -0.15"""
import sys, rospy, math
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
jid=int(sys.argv[1]);delta=float(sys.argv[2])
rospy.init_node('action_jog')
js=rospy.wait_for_message('/arm_left/joint_states',JointState,timeout=5)
cur=dict(zip(js.name,js.position))
names=['joint%d'%i for i in range(1,7)]
q=[cur[n] for n in names]
q[jid-1]+=delta
client=actionlib.SimpleActionClient('/arm_left/follow_joint_trajectory',FollowJointTrajectoryAction)
if not client.wait_for_server(rospy.Duration(3)):raise SystemExit('轨迹服务器不可用')
g=FollowJointTrajectoryGoal()
g.trajectory.joint_names=names
pt=JointTrajectoryPoint();pt.positions=q;pt.time_from_start=rospy.Duration(0.8)
g.trajectory.points=[pt]
client.send_goal(g)
if client.wait_for_result(rospy.Duration(6)):
    print('完成: joint%d %+.3frad -> %.3frad (%s)'%(jid,cur[names[jid-1]],q[jid-1],client.get_goal_status_text()))
else:
    print('失败:',client.get_goal_status_text())
