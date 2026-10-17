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
        """初始化左臂抓取节点。

        初始化流程:
            1. 初始化状态变量(抓取命令、检测结果、分配标志)
            2. 加载夹爪参数(夹紧/松开角度)
            3. 订阅抓取命令和YOLOv5检测结果话题
            4. 创建左臂控制发布器(完成状态、目标位姿、舵机角度)
            5. 初始化抓取参数和舵机角度消息模板
        """
        # ---- 状态变量 ----
        self.grab_flag = GrabCmd()       # 当前抓取命令状态(IDLE/START/BUZY/TIMEOUT/FINAL)
        self.yolov5_dets = ObjDets()     # YOLOv5最新检测结果(目标数量+边界框列表)
        self.assign_left = False         # 左臂是否被分配了抓取任务

        rospy.init_node('grab_test_left', anonymous=True)

        # ---- 夹爪参数 ----
        self.left_arm_tight = rospy.get_param('/arm_left/claw_tight', 30)   # 夹爪夹紧角度(舵机PWM值，越小越紧)
        self.left_arm_loose = rospy.get_param('/arm_left/claw_loose', 60)   # 夹爪松开角度(舵机PWM值，越大越松)

        # ---- 订阅器 ----
        # 抓取命令: 外部节点(如mission_manage)下发 START 触发抓取流程
        self.grab_flag_sub = rospy.Subscriber('/left_grab_flag', GrabCmd, self.grab_flag_cb)
        # YOLOv5检测结果: 前置相机检测到的果实位置(像素坐标+类别)
        self.yolov5_dets_sub = rospy.Subscriber('/ascamera_hp60c/rgb0/yolov5_detections_left', ObjDets, self.yolov5_dets_cb)
        
        # ---- 发布器 ----
        # 抓取完成状态: 通知外部节点当前抓取结果(BUZY/TIMEOUT/FINAL)
        self.left_final_pub = rospy.Publisher('/left_final_flag', GrabCmd, queue_size=1)
        # 左臂目标位姿: 发布IK求解的目标位姿(当前流程中未直接使用，预留接口)
        self.left_arm_target_pose_pub = rospy.Publisher('/arm_left/arm_target', PoseStamped, queue_size=3)
        # 左臂舵机角度: 发布6个关节的舵机角度，由arm_control_node串口下发
        self.left_arm_servo_degree_pub = rospy.Publisher('/arm_left/arm_joint_states', JointState, queue_size=3)
        
        # ---- 抓取参数与舵机消息模板 ----
        self.grab_param_l = GrabParam()  # 抓取计算参数(相机内参、手眼标定等)
        # 舵机角度消息模板，6个关节名对应6个舵机
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