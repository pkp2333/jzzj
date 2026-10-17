#!/usr/bin/python3
# -*- coding: utf-8 -*-
import time
import rospy
import numpy as np
from sensor_msgs.msg import JointState


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


def grab(grab_degree, loose, tight, lr:bool, arm_servo_degree:JointState, arm_servo_degree_pub:rospy.Publisher):
    lr = lr * 2 - 1

    arm_wait(grab_degree[0]+10*lr, loose, arm_servo_degree, arm_servo_degree_pub)
    time.sleep(.5)

    if grab_degree[0]+10*lr > 0:
        arm_servo_degree.position = radians_list([grab_degree[0]+10*lr, 0, 55, 55, -90, loose])
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
    time.sleep(1)

    arm_servo_degree.position = radians_list([grab_degree[0], grab_degree[1], grab_degree[2], grab_degree[3], grab_degree[4], tight])
    arm_servo_degree.header.stamp = rospy.Time.now()
    arm_servo_degree_pub.publish(arm_servo_degree)
    time.sleep(1)

    arm_servo_degree.position = radians_list([grab_degree[0]+10*lr, grab_degree[1], grab_degree[2], grab_degree[3], grab_degree[4], tight])
    arm_servo_degree.header.stamp = rospy.Time.now()
    arm_servo_degree_pub.publish(arm_servo_degree)
    time.sleep(1)

    if grab_degree[0]+10*lr > 0:
        arm_servo_degree.position = radians_list([grab_degree[0]+10*lr, grab_degree[1]-30*lr, grab_degree[2]-20*lr, grab_degree[3], grab_degree[4], tight])
        arm_servo_degree.header.stamp = rospy.Time.now()
        arm_servo_degree_pub.publish(arm_servo_degree)
        time.sleep(1)

        arm_servo_degree.position = radians_list([0, 0, 70, 0, 0, tight])
        arm_servo_degree.header.stamp = rospy.Time.now()
        arm_servo_degree_pub.publish(arm_servo_degree)
        time.sleep(1)

    arm_wait(0, tight, arm_servo_degree, arm_servo_degree_pub)
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