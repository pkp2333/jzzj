#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import time
import signal
import numpy as np
from arm_motions import *
from Arm_Lib import Arm_Device

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


arm_ik_servo_degree = JointState()
get_cmd_ik = False

def dh_matrix(theta, d, a, alpha):
    """计算单个关节的DH变换矩阵"""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

def fkine_5dof(q, dh_params):
    """6自由度机械臂正运动学计算"""
    T = np.eye(4)
    for i in range(6):
        theta, d, a, alpha = dh_params[i]
        T_i = dh_matrix(q[i] + theta, d, a, alpha)
        T = T @ T_i  # 矩阵连乘
    return T

def radians_list(degrees):
    """将角度列表转换为弧度"""
    return [np.radians(deg) for deg in degrees]

# 根据URDF文件精确修正的DH参数 (单位：米和弧度)
dh_params = [
    [0, 0.305, 0, -np.pi/2],
    [0, 0.085, 0.0, np.pi/2],  # 关节1rp
    [0, 0.0, 0.085, 0],  # 关节2
    [0, 0.0, 0.085, 0],  # 关节3
    [0, 0.0, 0.075, 0],  # 关节4
    [0, 0.0, 0.0, np.pi/2],  # 关节5
]

def print_pose(T):
    """打印位姿矩阵并提取位置和欧拉角"""
    position = T[:3, 3]
    rotation = T[:3, :3]

    # 计算欧拉角(ZYX顺序)
    sy = np.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2]) * 180 / np.pi
        pitch = np.arctan2(-rotation[2, 0], sy)* 180 / np.pi
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])* 180 / np.pi
    else:
        roll = np.arctan2(-rotation[1, 2], rotation[1, 1])
        pitch = np.arctan2(-rotation[2, 0], sy)
        yaw = 0

    print("\n末端位置 (m):", np.round(position*100, 4))
    print("欧拉角 (rad, ZYX):", np.round([yaw, pitch, roll], 4))
    # print("变换矩阵:\n", np.round(T, 4))




def signal_handler(sig, frame):
    rospy.signal_shutdown('User interrupted')
    sys.exit(0)


def arm_ik_servo_degree_cb(msg:JointState):
    global arm_ik_servo_degree, get_cmd_ik
    arm_ik_servo_degree = msg
    get_cmd_ik = True


def main():
    global arm_ik_servo_degree, get_cmd_ik

    rospy.init_node('arm_test_node', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)
    
    arm_ns = rospy.get_namespace()
    arm_dir = arm_ns[5:-1]

    if arm_dir in ['l', 'L', 'left', 'Left', 'LEFT']:
        lr = 1
    elif arm_dir in ['r', 'R', 'right', 'Right', 'RIGHT']:
        lr = 0
    else:
        lr = 1

    arm_front = rospy.get_param('turn_front', 90)
    arm_middle = rospy.get_param('turn_middle', 0)
    arm_back = rospy.get_param('turn_back', -90)
    claw_tight = rospy.get_param('claw_tight', 30)
    claw_loose = rospy.get_param('claw_loose', 60)
    delta_servo = []
    for i in range(1, 7):
        delta_servo.append(rospy.get_param(f'delta_servo_{i}', 0))

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rospy.loginfo(f"机械臂测试节点初始化中...\n")

    Arm = Arm_Device(lr=arm_dir)
    time.sleep(.01)

    arm_servo_degree = JointState()
    arm_servo_degree.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    arm_servo_degree_sub = rospy.Subscriber("arm_ik_servo_degree", JointState, arm_ik_servo_degree_cb)
    
    arm_torque_pub = rospy.Publisher('arm_torque_state', Bool, queue_size=3)
    arm_servo_degree_pub = rospy.Publisher('arm_joint_states', JointState, queue_size=3)
    tar_pose_car_pub = rospy.Publisher('tar_pose_car', PoseStamped, queue_size=3)
    arm_target_pose_pub = rospy.Publisher('arm_target', PoseStamped, queue_size=3)

    time.sleep(6)

    print("arm_servo_degree:",arm_servo_degree)
    arm_wait(arm_middle, claw_loose, arm_servo_degree, arm_servo_degree_pub)

    rospy.loginfo(f"机械臂测试节点成功初始化!\n")
    
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        command = input('\n代号       指令'
                        '\ntoff      关闭力矩'
                        '\nton       打开力矩'
                        '\nrd        读取角度'
                        '\nrp        读取末端位姿'
                        '\nstd       设定角度'
                        '\nstp       设定末端位姿'
                        '\nwf        前转待命'
                        '\nwm        居中待命'
                        '\nwb        后转待命'
                        '\ng         抓取'
                        '\np         放置'
                        '\nq         退出'
                        '\n>>>')
        if command == "toff":
            Arm.Arm_serial_set_torque(0)
            time.sleep(.5)
            print("\n机械臂舵机力矩已关闭, 可以掰动\n")
        elif command == "ton":
            Arm.Arm_serial_set_torque(1)
            time.sleep(.5)
            print("\n机械臂舵机力矩已启动\n")

        elif command == "rd":
            print("\n当前指令  读取角度\n")
            Arm.Arm_serial_set_torque(0)
            time.sleep(.5)
            print("\n机械臂舵机力矩已关闭, 可以掰动\n")
            input("\n当机械臂掰动到位后请输按回车\n")
            arm_torque_pub.publish(Bool(False))
            time.sleep(.5)
            ang = [0, 0, 0, 0, 0, 0]
            for i in range(6):
                ang[i] = Arm.Arm_serial_servo_read(i + 1)
                if ang[i] is None:
                    continue
                print("第" + str(i + 1) + "号舵机角度为 ", ang[i] - delta_servo[i], " °")
                Arm.Arm_serial_servo_write(i + 1, ang[i], 800)
                time.sleep(.1)
            print(f"\n[{ang[0]}, {ang[1]}, {ang[2]}, {ang[3]}, {ang[4]}, {ang[5]}]\n")
            Arm.Arm_serial_set_torque(1)
            time.sleep(.5)
            arm_torque_pub.publish(Bool(True))
            print("\n机械臂舵机力矩已启动\n")
            trans = tf_buffer.lookup_transform('car_base_link', f'{arm_ns[1:]}link5', rospy.Time(0))
            position = trans.transform.translation
            x, y, z = position.x, position.y, position.z
            orientation = trans.transform.rotation
            quat = [orientation.x, orientation.y, orientation.z, orientation.w]
            roll, pitch, yaw = euler_from_quaternion(quat)
            print("\n末端相对于 car_base_link 的位姿\n[x, y, z, R, P, Y]  cm °\n%.1f %.1f %.1f %.1f %.1f %.1f" 
                  % (x*100, y*100, z*100, np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)))
            
        elif command == "rp":
            print("\n当前指令  读取末端位姿\n")
            trans = tf_buffer.lookup_transform('car_base_link', f'{arm_ns[1:]}link5', rospy.Time(0))
            position = trans.transform.translation
            print("Translation:", trans.transform.translation)
            print("Rotation:", trans.transform.rotation)
            x, y, z = position.x, position.y, position.z
            orientation = trans.transform.rotation
            quat = [orientation.x, orientation.y, orientation.z, orientation.w]
            roll, pitch, yaw = euler_from_quaternion(quat)
            print("\n末端相对于 car_base_link 的位姿\n[x, y, z, R, P, Y]  cm °\n%.1f %.1f %.1f %.1f %.1f %.1f" 
                  % (x*100, y*100, z*100, np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)))
            ang_test = [0, 0, 0, 0, 0, 0]
            ang_test_ = [0, 0, 0, 0, 0, 0, 0]
            for i in range(6):
                ang_test[i] = Arm.Arm_serial_servo_read(i+1) - delta_servo[i]
                if ang_test[i] is None:
                    continue
                print("第" + str(i + 1) + "号舵机角度为 ", ang_test[i], " °")
                time.sleep(.1)

            for m in range(6):
                ang_test_[m+1] = ang_test[m]
            ang_test_[1] =  -ang_test[0] - 90
            ang_test_[2] =  -ang_test[1] - 90

            q = radians_list(ang_test_)
            for i in range(6):
                print(str(i+1)+"号舵机角度为:", ang_test_[i])
                time.sleep(.1)
            T = fkine_5dof(q, dh_params)
            print_pose(T)

        elif command == "std":
            print("\n当前指令  设定角度\n")
            num = input("\n请输入目标角度°(六个舵机, 以空格隔开): \n>>>")
            try:
                degree = [int(n) for n in num.split(" ")]
            except:
                print("\n输入有误\n")
                continue
            if len(degree) != 6:
                rospy.logerr("目标角度数量不匹配!")
                continue
            arm_servo_degree.position = radians_list(degree)
            arm_servo_degree.header.stamp = rospy.Time.now()
            arm_servo_degree_pub.publish(arm_servo_degree)
            
            ang_test = [0,0,0,0,0,0,0]
            for m in range(6):
                ang_test[m+1] = degree[m]

            ang_test[1] *= -1
            ang_test[2] *= -1
            ang_test[1] -= 90
            ang_test[2] -= 90
            
            # for i in range(6):
            #     print(str(i)+"号舵机角度为:", ang_test[i])
            #     time.sleep(.1)

            q = radians_list(ang_test)
            T = fkine_5dof(q, dh_params)
            print_pose(T)


        elif command == "stp":
            print("\n当前指令  设定位置\n")
            num = input("\n请输入目标位置(x y z r p y, 以空格隔开)\n"
                        "x正方向指向车体前方 y正方向指向车体左侧 z正方向指向车体上方\n"
                        "xyz单位cm\nrpy单位°\n"
                        ">>>")
            try:
                pose = [float(n) for n in num.split(" ")]
            except:
                print("\n输入有误\n")
                continue
            if len(pose) != 6:
                rospy.logerr("目标位姿信息长度不匹配!")
                continue
            
            pose_in_car_base = PoseStamped()
            pose_in_car_base.header.frame_id = "car_base_link"
            pose_in_car_base.pose.position.x = pose[0]/100
            pose_in_car_base.pose.position.y = pose[1]/100
            pose_in_car_base.pose.position.z = pose[2]/100
            quaternion = quaternion_from_euler(np.radians(pose[3]), 
                                               np.radians(pose[4]), 
                                               np.radians(pose[5]))
            pose_in_car_base.pose.orientation.x = quaternion[0]
            pose_in_car_base.pose.orientation.y = quaternion[1]
            pose_in_car_base.pose.orientation.z = quaternion[2]
            pose_in_car_base.pose.orientation.w = quaternion[3]

            print("\n末端相对于 car_base_link 的位姿\n[x, y, z, R, P, Y]  cm °\n%.1f %.1f %.1f %.1f %.1f %.1f" % 
                  (pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]))

            tar_pose_car_pub.publish(pose_in_car_base)

            transform = tf_buffer.lookup_transform(
                target_frame=f"{arm_ns[1:]}arm_base_link",
                source_frame="car_base_link",
                time=rospy.Time(0),
                timeout=rospy.Duration(1.0)
            )

            print("Transform from car_base_link to arm_base_link:")
            print("Translation:", transform.transform.translation)
            print("Rotation:", transform.transform.rotation)

            pose_in_arm_base = do_transform_pose(pose_in_car_base, transform)
            
            quat = [pose_in_arm_base.pose.orientation.x, pose_in_arm_base.pose.orientation.y, 
                    pose_in_arm_base.pose.orientation.z, pose_in_arm_base.pose.orientation.w]
            euler_angles = euler_from_quaternion(quat)
            print("\n末端相对于 arm_base_link 的位姿\n[x, y, z, R, P, Y]  cm °\n%.1f %.1f %.1f %.1f %.1f %.1f" % 
                  (pose_in_arm_base.pose.position.x*100, pose_in_arm_base.pose.position.y*100, pose_in_arm_base.pose.position.z*100,
                   np.rad2deg(euler_angles[0]), np.rad2deg(euler_angles[1]), np.rad2deg(euler_angles[2])))
            arm_target_pose_pub.publish(pose_in_arm_base)
            t0 = time.time()
            while not get_cmd_ik and (time.time() - t0 < 2):
                pass
            if get_cmd_ik:
                arm_servo_degree.position = radians_list(arm_ik_servo_degree.position)
                arm_servo_degree_pub.publish(arm_servo_degree)
                get_cmd_ik = False

        elif command == "wf":
            print("\n当前指令  前转待命\n")
            arm_wait(arm_front, claw_loose, arm_servo_degree, arm_servo_degree_pub)
        elif command == "wm":
            print("\n当前指令  居中待命\n")
            arm_wait(arm_middle, claw_loose, arm_servo_degree, arm_servo_degree_pub)
        elif command == "wb":
            print("\n当前指令  后转待命\n")
            arm_wait(arm_back, claw_loose, arm_servo_degree, arm_servo_degree_pub)

        elif command == "g":
            print("\n当前指令  抓取\n")

            num = input("\n请输入目标位置(x y z r p y, 以空格隔开)\n"
                        "x正方向指向车体前方 y正方向指向车体左侧 z正方向指向车体上方\n"
                        "xyz单位cm\nrpy单位°\n"
                        ">>>")
            try:
                pose = [float(n) for n in num.split(" ")]
            except:
                print("\n输入有误\n")
                continue
            if len(pose) != 6:
                rospy.logerr("目标位姿信息长度不匹配!")
                continue

            pose_in_car_base = PoseStamped()
            pose_in_car_base.header.frame_id = "car_base_link"
            pose_in_car_base.pose.position.x = pose[0]/100
            pose_in_car_base.pose.position.y = pose[1]/100
            pose_in_car_base.pose.position.z = pose[2]/100
            quaternion = quaternion_from_euler(np.radians(pose[3]), 
                                               np.radians(pose[4]), 
                                               np.radians(pose[5]))
            pose_in_car_base.pose.orientation.x = quaternion[0]
            pose_in_car_base.pose.orientation.y = quaternion[1]
            pose_in_car_base.pose.orientation.z = quaternion[2]
            pose_in_car_base.pose.orientation.w = quaternion[3]

            tar_pose_car_pub.publish(pose_in_car_base)

            transform = tf_buffer.lookup_transform(
                target_frame=f"{arm_ns[1:]}arm_base_link",
                source_frame="car_base_link",
                time=rospy.Time(0),
                timeout=rospy.Duration(1.0)
            )

            pose_in_arm_base = do_transform_pose(pose_in_car_base, transform)
            
            quat = [pose_in_arm_base.pose.orientation.x, pose_in_arm_base.pose.orientation.y, 
                    pose_in_arm_base.pose.orientation.z, pose_in_arm_base.pose.orientation.w]
            euler_angles = euler_from_quaternion(quat)
            print("\n末端相对于 arm_base_link 的位姿\n[x, y, z, R, P, Y]  cm °\n%.1f %.1f %.1f %.1f %.1f %.1f" % 
                  (pose_in_arm_base.pose.position.x*100, pose_in_arm_base.pose.position.y*100, pose_in_arm_base.pose.position.z*100,
                   np.rad2deg(euler_angles[0]), np.rad2deg(euler_angles[1]), np.rad2deg(euler_angles[2])))
            arm_target_pose_pub.publish(pose_in_arm_base)
            t0 = time.time()
            while not get_cmd_ik and (time.time() - t0 < 2):
                pass
            if get_cmd_ik:
                grab_degree = arm_ik_servo_degree.position
                get_cmd_ik = False
                grab(grab_degree, claw_loose, claw_tight, lr, arm_servo_degree, arm_servo_degree_pub)

        elif command == "p":
            print("\n当前指令  放置\n")
            put(arm_front, claw_loose, claw_tight, lr, arm_servo_degree, arm_servo_degree_pub)

        elif command == "q":
            print("\n当前指令  退出\n")
            rospy.logwarn("Press Ctrl + c to exit")
            break
        else:
            print("\n未知指令, 请重新输入\n")
        
        r.sleep()
        time.sleep(1)
    

if __name__ == '__main__':
    main()
