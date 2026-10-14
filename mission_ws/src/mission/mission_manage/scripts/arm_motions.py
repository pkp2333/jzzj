#!/usr/bin/python3
# -*- coding: utf-8 -*-
import time
import rospy
import numpy as np
from sensor_msgs.msg import JointState

fruit_pos_right = {
    0: [84, 61, 81, 40, 82, 59],
    1: [82, 45, 84, 40, 80, 59],
    2: [73, 12, 97, 59, 73, 30],
    3: [70, 24, 104, 64, 74, 59],
}

fruit_pos_left = {
    0: [-61, 17, 95, 62, -65, 60],
    1: [-60, 35, 90, 40, -70, 60],
    2: [-60, 50, 90, 35, -62, 60],
    3: [-60, 30, 100, 57, -62, 60],
}

def radians_list(degree):
    """
    将list中的角度全部转换为弧度制
    """
    return [np.radians(deg) for deg in degree]


def arm_wait(arm_dir, claw, arm_servo_degree:JointState, arm_servo_degree_pub:rospy.Publisher):
    arm_servo_degree.position = radians_list([arm_dir, 0, 90, 90, 0, claw])
    arm_servo_degree.header.stamp = rospy.Time.now()
    arm_servo_degree_pub.publish(arm_servo_degree)
    time.sleep(.5)

def arm_up(arm_dir, claw, arm_servo_degree:JointState, arm_servo_degree_pub:rospy.Publisher):
    arm_servo_degree.position = radians_list([arm_dir, 0, 90, 0, 0, claw])
    arm_servo_degree.header.stamp = rospy.Time.now()
    arm_servo_degree_pub.publish(arm_servo_degree)
    time.sleep(.5)

def grab(grab_degree, loose, tight, lr:bool, arm_servo_degree:JointState, arm_servo_degree_pub:rospy.Publisher):
    lr = lr * 2 - 1

    loose = 130
    tight = 150

    arm_wait(grab_degree[0]+10*lr, loose, arm_servo_degree, arm_servo_degree_pub)
    time.sleep(.5)

    if grab_degree[0]+10*lr > 0:
        arm_servo_degree.position = radians_list([grab_degree[0]+10*lr, 0, 55, 55, -90, loose])
        print(f"loose:{loose}")
        arm_servo_degree.header.stamp = rospy.Time.now()
        arm_servo_degree_pub.publish(arm_servo_degree)
        time.sleep(1)

    arm_servo_degree.position = radians_list([grab_degree[0]+10*lr, grab_degree[1], grab_degree[2], grab_degree[3], grab_degree[4], loose])
    arm_servo_degree.header.stamp = rospy.Time.now()
    arm_servo_degree_pub.publish(arm_servo_degree)
    time.sleep(1)

    arm_servo_degree.position = radians_list([grab_degree[0], grab_degree[1], grab_degree[2], grab_degree[3], grab_degree[4], loose])
    arm_servo_degree.header.stamp = rospy.Time.now()
    arm_servo_degree_pub.publish(arm_servo_degree)
    print("\到位\n")
    time.sleep(1)
    
    arm_servo_degree.position = radians_list([grab_degree[0], grab_degree[1], grab_degree[2], grab_degree[3], grab_degree[4], tight])
    # print(f"tight:{tight}")
    # time.sleep(10000000000000)
    # arm_servo_degree.header.stamp = rospy.Time.now()
    arm_servo_degree_pub.publish(arm_servo_degree)
    print(f"arm_servo_degree:{arm_servo_degree}")
    time.sleep(10)
    print("\n夹紧\n")
    time.sleep(1)

    arm_servo_degree.position = radians_list([grab_degree[0]+10*lr, grab_degree[1], grab_degree[2], grab_degree[3], grab_degree[4], tight])
    arm_servo_degree.header.stamp = rospy.Time.now()
    arm_servo_degree_pub.publish(arm_servo_degree)
    print("\n收臂\n")
    time.sleep(1)

    if (lr >= 0):
        arm_servo_degree.position = radians_list([grab_degree[0]-10*lr, grab_degree[1], grab_degree[2]-20*lr, grab_degree[3], grab_degree[4], tight])
    else:
        arm_servo_degree.position = radians_list([grab_degree[0]-10*lr, grab_degree[1], grab_degree[2]+20*lr, grab_degree[3], grab_degree[4], tight])
    arm_servo_degree.header.stamp = rospy.Time.now()
    arm_servo_degree_pub.publish(arm_servo_degree)
    print("\n收臂  往上\n")
    time.sleep(1)

    if grab_degree[0]+10*lr > 0:
        arm_servo_degree.position = radians_list([grab_degree[0]-30*lr, grab_degree[1]-30*lr, grab_degree[2]-20*lr, grab_degree[3], grab_degree[4], tight])
        arm_servo_degree.header.stamp = rospy.Time.now()
        arm_servo_degree_pub.publish(arm_servo_degree)
        time.sleep(1)
        print("\n get in 111 \n")
        arm_servo_degree.position = radians_list([0, 0, 70, 0, 0, tight])
        arm_servo_degree.header.stamp = rospy.Time.now()
        arm_servo_degree_pub.publish(arm_servo_degree)
        time.sleep(1)

    arm_up(0, tight, arm_servo_degree, arm_servo_degree_pub)
    time.sleep(.5)

def put_to_basket(fruit_id, loose, tight, lr:bool, arm_servo_degree:JointState, arm_servo_degree_pub:rospy.Publisher):
    lr = lr * 2 - 1

    # 判断机械臂方向，左右机械臂的存矿角度有些许差异
    if (lr > 0):
        put_degree = fruit_pos_left[fruit_id]
    elif (lr < 0):
        put_degree = fruit_pos_right[fruit_id]

    # 到达放果子姿态
    arm_servo_degree.position = radians_list([put_degree[0]+10*lr, put_degree[1], put_degree[2], put_degree[3], put_degree[4], tight])
    arm_servo_degree.header.stamp = rospy.Time.now()    
    arm_servo_degree_pub.publish(arm_servo_degree)
    print("到达上方")
    time.sleep(1.5)

    # 下移
    arm_servo_degree.position = radians_list([put_degree[0], put_degree[1], put_degree[2], put_degree[3], put_degree[4], loose])
    arm_servo_degree.header.stamp = rospy.Time.now()
    arm_servo_degree_pub.publish(arm_servo_degree)
    print("下移放果子")
    time.sleep(1.5)

    # # 放果子
    # arm_wait(put_degree[0], loose, arm_servo_degree, arm_servo_degree_pub)
    # print("放果子")
    # time.sleep(3)

    # 后续看看要不要再加个恢复原来姿态的动作
    arm_servo_degree.position = radians_list([0, 0, 90, 0, 0, 60])
    arm_servo_degree.header.stamp = rospy.Time.now()
    arm_servo_degree_pub.publish(arm_servo_degree)
    print("回位")
    time.sleep(.5)


def put(arm_front, loose, tight, lr:bool, arm_servo_degree:JointState, arm_servo_degree_pub:rospy.Publisher):
    lr = lr * 2 - 1

    arm_wait(0, tight, arm_servo_degree, arm_servo_degree_pub)
    time.sleep(.5)

    arm_wait(arm_front, tight, arm_servo_degree, arm_servo_degree_pub)
    time.sleep(.5)
    
    arm_servo_degree.position = radians_list([arm_front, 67, 41, 32, -90*lr, tight])
    arm_servo_degree.header.stamp = rospy.Time.now()
    arm_servo_degree_pub.publish(arm_servo_degree)
    time.sleep(1)

    arm_servo_degree.position = radians_list([arm_front, 67, 41, 32, -90*lr, loose])
    arm_servo_degree.header.stamp = rospy.Time.now()
    arm_servo_degree_pub.publish(arm_servo_degree)
    time.sleep(1)

    arm_wait(arm_front, loose, arm_servo_degree, arm_servo_degree_pub)
    time.sleep(.5)

    arm_wait(0, loose, arm_servo_degree, arm_servo_degree_pub)
    time.sleep(.5)