#!/usr/bin/python3
# -*- coding: utf-8 -*-
import re
import ast
import cv2
import time
import copy
import numpy as np
from utils import *
from arm_motions import *
from grab_test import grab_fruit_pos
import rospy
import tf2_ros
from std_msgs.msg import Bool, String, Int16 
from sensor_msgs.msg import CameraInfo, Image, JointState
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped
from user_msgs.msg import CarCmd, ExitCondition, ObjDet, ObjDets, GrabCmd

class LeftGrabNode:
    def __init__(self):
        self.grab_flag = GrabCmd()
        self.yolov5_dets = ObjDets()
        self.assign_left = False
        
        rospy.init_node('grab_test_left', anonymous=True)

        self.left_arm_tight = rospy.get_param('/arm_left/claw_tight', 30)
        self.left_arm_loose = rospy.get_param('/arm_left/claw_loose', 60)
        # self.left_arm_tight = 30
        # self.left_arm_loose = 60

        self.grab_flag_sub = rospy.Subscriber('/left_grab_flag', GrabCmd, self.grab_flag_cb)
        self.yolov5_dets_sub = rospy.Subscriber('/ascamera_hp60c/rgb0/yolov5_detections_left', ObjDets, self.yolov5_dets_cb)
        
        self.left_final_pub = rospy.Publisher('/left_final_flag', GrabCmd, queue_size=1)
        self.left_arm_target_pose_pub = rospy.Publisher('/arm_left/arm_target', PoseStamped, queue_size=3)
        self.left_arm_servo_degree_pub = rospy.Publisher('/arm_left/arm_joint_states', JointState, queue_size=3)
        
        # 初始化抓取参数
        self.grab_param_l = GrabParam()
        self.arm_servo_degree = JointState()
        self.arm_servo_degree.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    def yolov5_dets_cb(self, msg):
        self.yolov5_dets = msg

    def grab_flag_cb(self, msg:GrabCmd):
        self.grab_flag = msg.cmd

    def process_grab(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.grab_flag == GrabCmd.START:
                rospy.loginfo("左边开抓！")
                t1 = time.time()

                self.grab_flag = GrabCmd.BUZY

                while time.time() - t1 < 5:
                    if self.yolov5_dets.num > 0:
                        self.grab_flag = GrabCmd.TIMEOUT
                        break
                    rate.sleep()

                if self.yolov5_dets.num == 0:
                    rospy.loginfo("左边果子全部抓完")
                    self.grab_flag = GrabCmd.FIANL
                else:
                    yolov5_dets_ = copy.copy(self.yolov5_dets)
                    self.left_final_pub.publish(self.grab_flag)

                    for det in yolov5_dets_.dets:
                        det: ObjDet
                        target_center_pix = [det.bbox.x, det.bbox.y]
                        
                        pose, cur_fruit_pos = calc_j5_from_claw(True, target_center_pix, 0, self.grab_param_l)
                        
                        if pose is None or cur_fruit_pos == 0:
                            rospy.logwarn("无法计算抓取位置")
                            continue
                        
                        grab_degree_solid = grab_fruit_pos[cur_fruit_pos]
                        degree_test_left = [0, 0, 0, 0, 0, 0]
                        for i in range(6):
                            degree_test_left[i] = grab_degree_solid[i]
                        
                        # 抓取果子
                        grab(degree_test_left, self.left_arm_loose, self.left_arm_tight, True, 
                             self.arm_servo_degree, self.left_arm_servo_degree_pub)
                        
                        class_id_test = det.class_id if det.class_id < 4 else det.class_id - 2
                        # 将果子放进弹仓
                        put_to_basket(class_id_test, self.left_arm_loose, self.left_arm_tight, True, 
                                     self.arm_servo_degree, self.left_arm_servo_degree_pub)


                self.left_final_pub.publish(self.grab_flag)
                
            rate.sleep()

def main():
    try:
        node = LeftGrabNode()
        node.process_grab()
    except rospy.ROSInterruptException:
        rospy.loginfo("左抓取节点退出")
    except Exception as e:
        rospy.logerr(f"左抓取节点错误: {e}")

if __name__ == "__main__":
    main()