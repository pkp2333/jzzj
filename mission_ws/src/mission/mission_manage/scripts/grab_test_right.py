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

class RightGrabNode:
    def __init__(self):
        self.grab_flag = GrabCmd()
        self.yolov5_dets = ObjDets()
        self.assign_right = False
        
        # ROS初始化
        rospy.init_node('grab_test_right', anonymous=True)

        self.right_arm_tight = rospy.get_param('/arm_right/claw_tight', 30)
        self.right_arm_loose = rospy.get_param('/arm_right/claw_loose', 60)
        self.right_arm_tight = 150
        self.right_arm_loose = 130

        self.grab_flag_sub = rospy.Subscriber('/right_grab_flag', GrabCmd, self.grab_flag_cb)
        self.yolov5_dets_sub = rospy.Subscriber('/ascamera_hp60c/rgb0/yolov5_detections_right', ObjDets, self.yolov5_dets_cb)
        
        self.right_final_pub = rospy.Publisher('/right_final_flag', GrabCmd, queue_size=1)
        self.right_arm_target_pose_pub = rospy.Publisher('/arm_right/arm_target', PoseStamped, queue_size=3)
        self.right_arm_servo_degree_pub = rospy.Publisher('/arm_right/arm_joint_states', JointState, queue_size=3)
        
        self.grab_param_r = GrabParam()
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
                rospy.loginfo("右边开抓！")
                t1 = time.time()

                self.grab_flag = GrabCmd.BUZY

                # 等待检测结果或超时
                while time.time() - t1 < 5:  # 5秒超时
                    if self.yolov5_dets.num > 0:
                        self.grab_flag = GrabCmd.TIMEOUT
                        break
                    rate.sleep()

                if self.yolov5_dets.num == 0:
                    rospy.loginfo("右边果子全部抓完")
                    self.grab_flag = GrabCmd.FIANL
                else:
                    yolov5_dets_ = copy.copy(self.yolov5_dets)
                    self.right_final_pub.publish(self.grab_flag)

                    for det in yolov5_dets_.dets:
                        det: ObjDet
                        target_center_pix = [det.bbox.x, det.bbox.y]
                        pose, cur_fruit_pos = calc_j5_from_claw(False, target_center_pix, 0, self.grab_param_r)
                        
                        if pose is None or cur_fruit_pos == 0:
                            rospy.logwarn("无法计算抓取位置")
                            continue
                        
                        grab_degree_solid = grab_fruit_pos[cur_fruit_pos]
                        degree_test_right = [0, 0, 0, 0, 0, 0]
                        for i in range(6):
                            degree_test_right[i] = grab_degree_solid[i]
                        
                        # 抓取果子
                        grab(degree_test_right, self.right_arm_loose, self.right_arm_tight, False, 
                             self.arm_servo_degree, self.right_arm_servo_degree_pub)
                        
                        class_id_test = det.class_id if det.class_id < 4 else det.class_id - 2
                        # 将果子放进弹仓
                        put_to_basket(class_id_test, self.right_arm_loose, self.right_arm_tight, False, 
                                     self.arm_servo_degree, self.right_arm_servo_degree_pub)

                # 发布抓取完成标志
                self.right_final_pub.publish(self.grab_flag)

            rate.sleep()

def main():
    try:
        node = RightGrabNode()
        node.process_grab()
    except rospy.ROSInterruptException:
        rospy.loginfo("右抓取节点退出")
    except Exception as e:
        rospy.logerr(f"右抓取节点错误: {e}")

if __name__ == "__main__":
    main()