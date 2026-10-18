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

saved_joint_angles = [0, 0, 90, 90, 0]
arm_ik_servo_degree = JointState()
get_cmd_ik = False


def dh_matrix(theta, d, a, alpha):
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
    T = np.eye(4)
    for i in range(6):
        theta, d, a, alpha = dh_params[i]
        T_i = dh_matrix(q[i] + theta, d, a, alpha)
        T = T @ T_i
    return T


def radians_list(degrees):
    return [np.radians(deg) for deg in degrees]


dh_params = [
    [0, 0.305, 0, -np.pi / 2],
    [0, 0.085, 0.0, np.pi / 2],
    [0, 0.0, 0.085, 0],
    [0, 0.0, 0.085, 0],
    [0, 0.0, 0.075, 0],
    [0, 0.0, 0.0, np.pi / 2],
]


def print_pose(T):
    position = T[:3, 3]
    rotation = T[:3, :3]
    sy = np.sqrt(rotation[0, 0] ** 2 + rotation[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2]) * 180 / np.pi
        pitch = np.arctan2(-rotation[2, 0], sy) * 180 / np.pi
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0]) * 180 / np.pi
    else:
        roll = np.arctan2(-rotation[1, 2], rotation[1, 1])
        pitch = np.arctan2(-rotation[2, 0], sy)
        yaw = 0

    print("\n末端位置 (m):", np.round(position * 100, 4))
    print("欧拉角 (deg):", np.round([yaw, pitch, roll], 4))


def signal_handler(sig, frame):
    rospy.signal_shutdown("User interrupted")
    sys.exit(0)


def arm_ik_servo_degree_cb(msg: JointState):
    global arm_ik_servo_degree, get_cmd_ik
    arm_ik_servo_degree = msg
    get_cmd_ik = True


def read_servo_angles(arm, delta_servo):
    """读取当前舵机角度，返回去除补偿后的六路角度。"""
    angles = []
    for i in range(6):
        value = arm.Arm_serial_servo_read(i + 1)
        if value is None:
            print("舵机%d读取失败" % (i + 1))
            return None

        angles.append(value - delta_servo[i])
        time.sleep(0.05)

    return angles


def publish_servo_angles(pub, message, angles):
    message.position = radians_list(angles)
    message.header.stamp = rospy.Time.now()
    pub.publish(message)


def main():
    global arm_ik_servo_degree, get_cmd_ik, saved_joint_angles

    rospy.init_node('arm_test_node', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)

    arm_ns = rospy.get_namespace()
    arm_dir = arm_ns[5:-1]

    if arm_dir.lower() in ['l', 'left']:
        lr = 1
    elif arm_dir.lower() in ['r', 'right']:
        lr = 0
    else:
        lr = 1

    arm_front = rospy.get_param('turn_front', 90)
    arm_middle = rospy.get_param('turn_middle', 0)
    arm_back = rospy.get_param('turn_back', -90)
    claw_tight = rospy.get_param('claw_tight', 30)
    claw_loose = rospy.get_param('claw_loose', 60)
    delta_servo = [
        rospy.get_param('delta_servo_%d' % i, 0)
        for i in range(1, 7)
    ]

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.loginfo("机械臂测试节点初始化中...")
    arm = Arm_Device(lr=arm_dir)
    time.sleep(0.01)

    arm_servo_degree = JointState()
    arm_servo_degree.name = [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    ]

    arm_servo_degree_sub = rospy.Subscriber(
        "arm_ik_servo_degree", JointState, arm_ik_servo_degree_cb
    )
    arm_torque_pub = rospy.Publisher(
        'arm_torque_state', Bool, queue_size=3
    )
    arm_servo_degree_pub = rospy.Publisher(
        'arm_joint_states', JointState, queue_size=3
    )
    tar_pose_car_pub = rospy.Publisher(
        'tar_pose_car', PoseStamped, queue_size=3
    )
    arm_target_pose_pub = rospy.Publisher(
        'arm_target', PoseStamped, queue_size=3
    )

    # arm_wait() establishes the known startup pose. This is also the
    # fallback used when the servo read operation temporarily times out.
    curr_ang = [arm_middle, 0, 90, 90, 0, claw_loose]
    time.sleep(6)
    arm_wait(arm_middle, claw_loose, arm_servo_degree, arm_servo_degree_pub)
    rospy.loginfo("机械臂测试节点启动成功!")

    measured = read_servo_angles(arm, delta_servo)
    if measured is not None:
        curr_ang = measured
        print("启动读取角度:", curr_ang)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        command = input(
            '\n代号       指令\n'
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
            '\ng1~g6     对应舵机+8度'
            '\nb1~b6     对应舵机-8度'
            '\na         手动输入角度'
            '\nb         执行抓取流程'
            '\nc         执行放置流程'
            '\n>>>'
        )

        if command == "toff":
            arm.Arm_serial_set_torque(0)
            time.sleep(0.5)
            print("\n力矩已关闭")

        elif command == "ton":
            arm.Arm_serial_set_torque(1)
            time.sleep(0.5)
            print("\n力矩已开启")

        elif command == "rd":
            print("\n读取角度")
            arm.Arm_serial_set_torque(0)
            input("\n调整到位后按回车")
            arm_torque_pub.publish(Bool(False))
            time.sleep(0.5)

            measured = read_servo_angles(arm, delta_servo)
            if measured is not None:
                curr_ang = measured
                print("\n[%s]" % ", ".join(map(str, curr_ang)))
                publish_servo_angles(
                    arm_servo_degree_pub, arm_servo_degree, curr_ang
                )
            else:
                print("读取失败，保留上一次角度缓存")

            arm.Arm_serial_set_torque(1)
            time.sleep(0.5)
            arm_torque_pub.publish(Bool(True))
            print("\n力矩已开启")

        elif command == "rp":
            trans = tf_buffer.lookup_transform(
                'car_base_link',
                '%slink5' % arm_ns[1:],
                rospy.Time(0)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            q = [
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ]
            roll, pitch, yaw = euler_from_quaternion(q)
            print(
                "\n末端位姿 x=%.1f y=%.1f z=%.1f r=%.1f p=%.1f y=%.1f"
                % (
                    x * 100, y * 100, z * 100,
                    np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)
                )
            )

        elif command == "std":
            num = input("\n输入6个角度: ")
            try:
                degree = list(map(int, num.split()))
            except ValueError:
                print("输入错误")
                continue

            if len(degree) != 6:
                print("必须输入6个角度")
                continue

            curr_ang = degree[:]
            publish_servo_angles(
                arm_servo_degree_pub, arm_servo_degree, curr_ang
            )
            time.sleep(1.2)

        elif command == "stp":
            try:
                pose = list(map(float, input(
                    "\n输入 x y z r p y: "
                ).split()))
            except ValueError:
                print("输入错误")
                continue

            if len(pose) != 6:
                print("必须输入6个数值")
                continue

            p = PoseStamped()
            p.header.frame_id = "car_base_link"
            p.pose.position.x = pose[0] / 100.0
            p.pose.position.y = pose[1] / 100.0
            p.pose.position.z = pose[2] / 100.0
            q = quaternion_from_euler(
                np.radians(pose[3]),
                np.radians(pose[4]),
                np.radians(pose[5])
            )
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            tar_pose_car_pub.publish(p)

            tf = tf_buffer.lookup_transform(
                "%sarm_base" % arm_ns[1:],
                "car_base_link",
                rospy.Time(0)
            )
            arm_target_pose_pub.publish(do_transform_pose(p, tf))

            t0 = time.time()
            while not get_cmd_ik and time.time() - t0 < 2:
                pass
            if get_cmd_ik:
                curr_ang = list(np.rad2deg(arm_ik_servo_degree.position))
                publish_servo_angles(
                    arm_servo_degree_pub, arm_servo_degree, curr_ang
                )
                get_cmd_ik = False

        elif command == "a":
            print("\n手动输入角度")
            try:
                deg = list(map(int, input(
                    "请输入6个舵机角度: "
                ).split()))
            except ValueError:
                print("输入错误")
                continue

            if len(deg) != 6:
                print("必须输入6个角度")
                continue

            curr_ang = deg[:]
            saved_joint_angles = deg[:5]
            publish_servo_angles(
                arm_servo_degree_pub, arm_servo_degree, curr_ang
            )
            print("已到达:", curr_ang)
            time.sleep(2.5)

        elif command == "b":
            if len(curr_ang) != 6:
                print("当前角度未知，请先执行 rd 或 a 命令")
                continue

            print("\nmissionb 执行抓取")
            deg2 = curr_ang[:5] + [50]
            publish_servo_angles(
                arm_servo_degree_pub, arm_servo_degree, deg2
            )
            curr_ang = deg2[:]
            print("已设置:", deg2)
            time.sleep(1.5)

            deg3 = [-1, -21, 40, 59, -1, 50]
            publish_servo_angles(
                arm_servo_degree_pub, arm_servo_degree, deg3
            )
            curr_ang = deg3[:]
            print("抬升完成")
            time.sleep(2)

            deg4 = [0, 0, 90, 90, 0, 30]
            publish_servo_angles(
                arm_servo_degree_pub, arm_servo_degree, deg4
            )
            curr_ang = deg4[:]
            time.sleep(2)

        elif command == "c":
            print("\nmissionc 执行放置")

            deg3 = [-1, -21, 40, 59, -1, 50]
            publish_servo_angles(
                arm_servo_degree_pub, arm_servo_degree, deg3
            )
            curr_ang = deg3[:]
            print("抬升完成")
            time.sleep(2)

            deg4 = [-100, -1, 50, 60, -89, 50]
            publish_servo_angles(
                arm_servo_degree_pub, arm_servo_degree, deg4
            )
            curr_ang = deg4[:]
            time.sleep(2)

            deg4 = [-100, -1, 50, 60, -89, 130]
            publish_servo_angles(
                arm_servo_degree_pub, arm_servo_degree, deg4
            )
            curr_ang = deg4[:]
            time.sleep(2)

            deg4 = [0, 0, 90, 90, 0, 130]
            publish_servo_angles(
                arm_servo_degree_pub, arm_servo_degree, deg4
            )
            curr_ang = deg4[:]
            time.sleep(2)

        elif command in [
            "g1", "g2", "g3", "g4", "g5", "g6",
            "b1", "b2", "b3", "b4", "b5", "b6"
        ]:
            if len(curr_ang) != 6:
                print("当前角度未知，请先执行 rd 或 a 命令")
                continue

            servo_limit = [
                [-130, 130], [-40, 60], [30, 120],
                [30, 120], [-90, 90], [20, 140]
            ]
            idx = int(command[1]) - 1
            step = 8 if command[0] == 'g' else -8

            # 二号舵机方向与其余舵机相反。
            if idx == 1:
                step = -step

            new_ang = curr_ang[idx] + step
            mn, mx = servo_limit[idx]
            if new_ang < mn or new_ang > mx:
                print("已到达限位")
                continue

            curr_ang[idx] = new_ang
            target = new_ang + delta_servo[idx]
            arm.Arm_serial_servo_write(idx + 1, target, 300)
            publish_servo_angles(
                arm_servo_degree_pub, arm_servo_degree, curr_ang
            )
            print(
                "舵机%d 调整%d度 当前%.1f度"
                % (idx + 1, step, curr_ang[idx])
            )
            time.sleep(0.6)

        elif command == "wf":
            arm_wait(arm_front, claw_loose, arm_servo_degree, arm_servo_degree_pub)
            curr_ang = [arm_front, 0, 90, 90, 0, claw_loose]

        elif command == "wm":
            arm_wait(arm_middle, claw_loose, arm_servo_degree, arm_servo_degree_pub)
            curr_ang = [arm_middle, 0, 90, 90, 0, claw_loose]

        elif command == "wb":
            arm_wait(arm_back, claw_loose, arm_servo_degree, arm_servo_degree_pub)
            curr_ang = [arm_back, 0, 90, 90, 0, claw_loose]

        elif command == "g":
            try:
                pose = list(map(float, input(
                    "\n输入抓取坐标 x y z r p y: "
                ).split()))
            except ValueError:
                print("输入错误")
                continue

            if len(pose) != 6:
                print("必须输入6个数值")
                continue

            p = PoseStamped()
            p.header.frame_id = "car_base_link"
            p.pose.position.x = pose[0] / 100.0
            p.pose.position.y = pose[1] / 100.0
            p.pose.position.z = pose[2] / 100.0
            q = quaternion_from_euler(
                np.radians(pose[3]),
                np.radians(pose[4]),
                np.radians(pose[5])
            )
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            tar_pose_car_pub.publish(p)

            tfm = tf_buffer.lookup_transform(
                "%sarm_base" % arm_ns[1:],
                "car_base_link",
                rospy.Time(0)
            )
            arm_target_pose_pub.publish(do_transform_pose(p, tfm))

            t0 = time.time()
            while not get_cmd_ik and time.time() - t0 < 2:
                pass
            if get_cmd_ik:
                grab(
                    arm_ik_servo_degree.position,
                    claw_loose,
                    claw_tight,
                    lr,
                    arm_servo_degree,
                    arm_servo_degree_pub
                )
                get_cmd_ik = False

        elif command == "p":
            put(
                arm_front,
                claw_loose,
                claw_tight,
                lr,
                arm_servo_degree,
                arm_servo_degree_pub
            )

        elif command == "q":
            print("\n退出程序")
            break

        else:
            print("\n未知指令")

        r.sleep()
        time.sleep(1)


if __name__ == '__main__':
    main()
