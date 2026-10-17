#!/usr/bin/env python3
"""Demo-only handoff generator. An explicit private backend gate prevents real-mode use."""
import rospy
from std_msgs.msg import String
from jzzj_grasp_interfaces.msg import StageCContext
if __name__=='__main__':
    rospy.init_node('sim_stage_c_context')
    rospy.wait_for_message('/stage_c/backend',String,timeout=10.)
    if rospy.get_param('/stage_c_server/backend','')!='sim':raise RuntimeError('sim context cannot target real backend')
    p=rospy.Publisher('/stage_c/context',StageCContext,queue_size=1)
    rate=rospy.Rate(2.)
    while not rospy.is_shutdown():
        if rospy.get_param('/stage_c_server/backend','')!='sim':raise RuntimeError('backend changed; refusing handoff')
        m=StageCContext();m.header.stamp=rospy.Time.now();m.session_id=rospy.get_param('~session_id','stage-c-demo')
        m.base_locked=m.arm_control_granted=True;p.publish(m);rate.sleep()
