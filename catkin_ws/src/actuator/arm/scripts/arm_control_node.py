#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import time
import numpy as np
from threading import Lock
from Arm_Lib import Arm_Device
from arm_motions import radians_list

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf.transformations import quaternion_from_euler


class ArmNode:
    def __init__(self) -> None:
        arm_ns = rospy.get_namespace()
        arm_dir = arm_ns[5:-1]

        self.left = rospy.get_param("turn_left", 90)
        self.middle = rospy.get_param("turn_middle", 0)
        self.right = rospy.get_param("turn_right", -90)
        self.tight = rospy.get_param("claw_tight", 30)
        self.loose = rospy.get_param("claw_loose", 60)
        t_car_to_arm = rospy.get_param("translation", [0, 0, 0])  # cm
        r_car_to_arm = rospy.get_param("rotation", [0, 0, 0])
        # t_car_to_arm = [ -0.075, 0.305, 0.0]
        # r_car_to_arm = [0, 90, -90]
        self.delta_servo = []
        for i in range(1, 7):
            self.delta_servo.append(rospy.get_param(f"delta_servo_{i}", 0))
        self.arm_degree_init = rospy.get_param("arm_degree_init", [0, 0, 90, 90, 0])

        rospy.loginfo(f"机械臂控制节点初始化中...\n")

        # Share an OS lease with the opt-in stage-C trajectory driver.
        from hardware_lease import HardwareLease
        self.hardware_lease = HardwareLease(arm_dir)
        self.Arm = Arm_Device(lr=arm_dir)
        time.sleep(.01)

        ang = [0, 0, 0, 0, 0, 0]
        for i in range(6):
            ang[i] = self.Arm.Arm_serial_servo_read(i + 1)
            if ang[i] is None:
                continue
            self.Arm.Arm_serial_servo_write(i + 1, ang[i], 800)
            time.sleep(.1)
        time.sleep(.5)
        self.Arm.Arm_serial_set_torque(1)
        time.sleep(.5)

        self.Arm.Arm_serial_servo_write6(self.arm_degree_init[0] + self.delta_servo[0], self.arm_degree_init[1] + self.delta_servo[1],
                                         self.arm_degree_init[2] + self.delta_servo[2], self.arm_degree_init[3] + self.delta_servo[3],
                                         self.arm_degree_init[4] + self.delta_servo[4], self.loose + self.delta_servo[5], 800)
        time.sleep(.1)
        self.Arm.Arm_serial_servo_write6(self.arm_degree_init[0] + self.delta_servo[0], self.arm_degree_init[1] + self.delta_servo[1],
                                         self.arm_degree_init[2] + self.delta_servo[2], self.arm_degree_init[3] + self.delta_servo[3],
                                         self.arm_degree_init[4] + self.delta_servo[4], self.loose + self.delta_servo[5], 800)
        time.sleep(1)

        self.task = True
        
        broadcaster = StaticTransformBroadcaster()

        static_tf_car_2_arm = TransformStamped()
        static_tf_car_2_arm.header.stamp = rospy.Time.now()
        static_tf_car_2_arm.header.frame_id = "car_base_link"
        static_tf_car_2_arm.child_frame_id = f"{arm_ns}arm_base_link"
        static_tf_car_2_arm.transform.translation.x = t_car_to_arm[0]
        static_tf_car_2_arm.transform.translation.y = t_car_to_arm[1]
        static_tf_car_2_arm.transform.translation.z = t_car_to_arm[2]
        quaternion = quaternion_from_euler(np.radians(r_car_to_arm[0]), 
                                           np.radians(r_car_to_arm[1]), 
                                           np.radians(r_car_to_arm[2]))
        static_tf_car_2_arm.transform.rotation.x = quaternion[0]
        static_tf_car_2_arm.transform.rotation.y = quaternion[1]
        static_tf_car_2_arm.transform.rotation.z = quaternion[2]
        static_tf_car_2_arm.transform.rotation.w = quaternion[3]

        broadcaster.sendTransform(static_tf_car_2_arm)

        self.servo_degree = JointState()
        self.servo_degree.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.servo_degree.position = self.arm_degree_init + [self.loose]

        self.servo_degree_lists = []
        self.last_servo_degree = None
        self.last_same_cnt = 0

        self.lock = Lock()
        self.torque_on = True

        self.arm_servo_degree_sub = rospy.Subscriber("arm_joint_states", JointState, self.arm_servo_degree_cb, queue_size=1)
        self.arm_torque_sub = rospy.Subscriber("arm_torque_state", Bool, self.arm_torque_cb, queue_size=1)
        self.arm_servo_degree_pub = rospy.Publisher("joint_states", JointState, queue_size=3)
        
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.controller)

        self.read_deg_timer = rospy.Timer(rospy.Duration(.1), self.read_deg)

        rospy.loginfo(f"机械臂控制节点成功初始化\n")
    
    def __del__(self):
        print("\n机械臂控制节点成功结束\n")

    def arm_torque_cb(self, msg:Bool):
        self.torque_on = msg.data

    def arm_servo_degree_cb(self, msg:JointState):
        with self.lock:
            servo_deg = list(msg.position)
            
            num = len(servo_deg)
            for i in range(num):
                servo_deg[i] = int(np.round(np.rad2deg(servo_deg[i])))
            if num == 5:
                servo_deg = servo_deg + [self.loose]
            elif num < 5:
                return

            if servo_deg == self.last_servo_degree:
                self.last_same_cnt += 1
            else:
                self.last_same_cnt = 0

            if self.last_same_cnt < 3:
                self.servo_degree_lists.append(servo_deg)
                self.last_servo_degree = servo_deg
            

    def controller(self, *args):
        if not self.task or not self.torque_on or not len(self.servo_degree_lists):
            return
                        
        servo_degree = None

        with self.lock:
            servo_degree = self.servo_degree_lists[0]
            self.servo_degree_lists.pop(0)

        if servo_degree is not None:
            rospy.loginfo(f"机械臂执行: {list(servo_degree)}\n")

            self.Arm.Arm_serial_servo_write6(servo_degree[0] + self.delta_servo[0], 
                                             -servo_degree[1] + self.delta_servo[1],
                                             servo_degree[2] + self.delta_servo[2], 
                                             servo_degree[3] + self.delta_servo[3],
                                             servo_degree[4] + self.delta_servo[4], 
                                             -servo_degree[5], 800)
            time.sleep(.05)
        
    def read_deg(self, *args):
        if self.torque_on:
            degree = [0, 0, 0, 0, 0, 0]
            for i in range(6):
                ang = self.Arm.Arm_serial_servo_read(i + 1)
                if ang is not None:
                    degree[i] = ang - self.delta_servo[i]
                else:
                    print("第" + str(i + 1) + "号舵机角度超限")
                    return
                degree[1] *= -1
                time.sleep(.005)

            self.servo_degree.position = radians_list(degree)
            self.servo_degree.header.stamp = rospy.Time.now()
            self.arm_servo_degree_pub.publish(self.servo_degree)
            time.sleep(.05)


if __name__ == '__main__':
    rospy.init_node('arm_ctrl_node', anonymous=True)
    mode = rospy.get_param("~control_mode", "legacy")
    if mode == "trajectory":
        from arm_trajectory_node import TrajectoryDriver
        arm = TrajectoryDriver()
    elif mode == "legacy":
        arm = ArmNode()
    else:
        raise ValueError("control_mode must be legacy or trajectory")
    rospy.spin()
    
