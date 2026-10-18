#!/usr/bin/env python3
import time
import rospy
import actionlib
from jzzj_grasp_interfaces.msg import VisualGraspAction, VisualGraspGoal
if __name__=='__main__':
    rospy.init_node('stage_c_client')
    client=actionlib.SimpleActionClient('/stage_c/grasp',VisualGraspAction)
    if not client.wait_for_server(rospy.Duration(5.)):raise RuntimeError('stage C server unavailable')
    goal=VisualGraspGoal(session_id=rospy.get_param('~session_id','stage-c-demo'),
        target_color=rospy.get_param('~target_color','red'),grasp_profile=rospy.get_param('~grasp_profile','tabletop'),
        timeout=rospy.Duration(rospy.get_param('~timeout',60.)))
    rospy.on_shutdown(client.cancel_goal)
    client.send_goal(goal,feedback_cb=lambda f:rospy.loginfo_throttle(1.,'stage C: %s',f.step))
    stop_wait=max(5.,float(rospy.get_param('/stage_c_server/execution/stop_timeout',8.))+1.)
    deadline=time.monotonic()+goal.timeout.to_sec()+stop_wait
    while client.get_state() in (0,1,6,7) and time.monotonic()<deadline and not rospy.is_shutdown():
        time.sleep(.02)
    if client.get_state() in (0,1,6,7):
        client.cancel_goal();end=time.monotonic()+stop_wait
        while client.get_state() in (0,1,6,7) and time.monotonic()<end:time.sleep(.02)
    result=client.get_result()
    if result is None:raise RuntimeError('no final result / stop not confirmed')
    rospy.loginfo('stage C result: %s',result)
