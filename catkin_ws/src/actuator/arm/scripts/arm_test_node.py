#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import time
import signal
import numpy as np
from arm_motions import *
from checked_arm_sdk import CheckedArmDevice as Arm_Device

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
    [0, 0.305, 0, -np.pi/2],
    [0, 0.085, 0.0, np.pi/2],
    [0, 0.0, 0.085, 0],
    [0, 0.0, 0.085, 0],
    [0, 0.0, 0.075, 0],
    [0, 0.0, 0.0, np.pi/2],
]

def print_pose(T):
    position = T[:3, 3]
    rotation = T[:3, :3]
    sy = np.sqrt(rotation[0, 0]**2 + rotation[1, 0]**2)
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
    print("欧拉角 (deg):", np.round([yaw, pitch, roll], 4))

def signal_handler(sig, frame):
    rospy.signal_shutdown("User interrupted")
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

    if arm_dir.lower() in ['l', 'left']:
        lr = 1
    elif arm_dir.lower() in ['r', 'right']:
        lr = 0
    else:
        lr = 1

    arm_front = rospy.get_param('turn_front', 90)
    curr_ang = [0, 0, 90, 90, 0, 60]  # 兜底初始角度，防止未 std/a 直接 b/gN 触发 UnboundLocalError
    arm_middle = rospy.get_param('turn_middle', 0)
    arm_back = rospy.get_param('turn_back', -90)
    claw_tight = rospy.get_param('claw_tight', 30)
    claw_loose = rospy.get_param('claw_loose', 60)
    delta_servo = [rospy.get_param('delta_servo_%d'%i, 0) for i in range(1,7)]

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rospy.loginfo("机械臂测试节点初始化中...")
    Arm = Arm_Device(lr=arm_dir)
    time.sleep(0.01)

    arm_servo_degree = JointState()
    arm_servo_degree.name = ["joint1","joint2","joint3","joint4","joint5","joint6"]

    arm_servo_degree_sub = rospy.Subscriber("arm_ik_servo_degree", JointState, arm_ik_servo_degree_cb)
    arm_torque_pub = rospy.Publisher('arm_torque_state', Bool, queue_size=3)
    arm_servo_degree_pub = rospy.Publisher('arm_joint_states', JointState, queue_size=3)
    tar_pose_car_pub = rospy.Publisher('tar_pose_car', PoseStamped, queue_size=3)
    arm_target_pose_pub = rospy.Publisher('arm_target', PoseStamped, queue_size=3)

    time.sleep(6)
    arm_wait(arm_middle, claw_loose, arm_servo_degree, arm_servo_degree_pub)
    rospy.loginfo("机械臂测试节点启动成功!")
    
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        command = input('\n代号       指令\n\ntoff      关闭力矩\nton       打开力矩\nrd        读取角度\nrp        读取末端位姿\nstd       设定角度\nstp       设定末端位姿\nwf        前转待命\nwm        居中待命\nwb        后转待命\ng         抓取\np         放置\nq         退出\ng1~g6     对应舵机+8度\nb1~b6     对应舵机-8度\nok        一键抓取复位\na  手动输入摆动角度\nb  收回待命\n\n>>>')
                        
        if command == "toff":
            Arm.Arm_serial_set_torque(0)
            time.sleep(0.5)
            print("\n力矩已关闭")
            
        elif command == "ton":
            Arm.Arm_serial_set_torque(1)
            time.sleep(0.5)
            print("\n力矩已开启")

        elif command == "rd":
            print("\n读取角度")
            Arm.Arm_serial_set_torque(0)
            input("\n调整到位后按回车")
            arm_torque_pub.publish(Bool(False))
            time.sleep(0.5)
            ang = [0]*6
            for i in range(6):
                ang[i] = Arm.Arm_serial_servo_read(i+1)
                if ang[i] is not None:
                    print("舵机%d: %.1f 度" % (i+1, ang[i]-delta_servo[i]))
                    #Arm.Arm_serial_servo_write(i+1, ang[i], 800)
                    time.sleep(0.1)
            print("\n[%s]" % ", ".join(map(str, ang)))
            Arm.Arm_serial_set_torque(1)
            time.sleep(0.5)
            arm_torque_pub.publish(Bool(True))
            print("\n力矩已开启")
            
        elif command == "rp":
            trans = tf_buffer.lookup_transform('car_base_link', '%slink5'%arm_ns[1:], rospy.Time(0))
            x,y,z = trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z
            q = [trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w]
            roll,pitch,yaw = euler_from_quaternion(q)
            print("\n末端位姿 x=%.1f y=%.1f z=%.1f r=%.1f p=%.1f y=%.1f" % (x*100, y*100, z*100, np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)))

        elif command == "std":
            num = input("\n输入6个角度: ")
            try:
                degree = list(map(int, num.split()))
                curr_ang=degree
            except:
                print("输入错误")
                continue
            if len(degree)!=6:
                print("必须输入6个角度")
                continue
            arm_servo_degree.position = radians_list(degree)
            arm_servo_degree.header.stamp = rospy.Time.now()
            arm_servo_degree_pub.publish(arm_servo_degree)
                   
            time.sleep(1.2)

        elif command == "stp":
            s = input("\n输入 x y z r p y: ")
            pose = list(map(float, s.split()))
            p = PoseStamped()
            p.header.frame_id = "car_base_link"
            p.pose.position.x = pose[0]/100.0
            p.pose.position.y = pose[1]/100.0
            p.pose.position.z = pose[2]/100.0
            q = quaternion_from_euler(np.radians(pose[3]), np.radians(pose[4]), np.radians(pose[5]))
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            tar_pose_car_pub.publish(p)
            tf = tf_buffer.lookup_transform("%sarm_base_link"%arm_ns[1:], "car_base_link", rospy.Time(0))
            p2 = do_transform_pose(p, tf)
            arm_target_pose_pub.publish(p2)
            t0 = time.time()
            while not get_cmd_ik and time.time()-t0 < 2:
                pass
            if get_cmd_ik:
                arm_servo_degree.position = arm_ik_servo_degree.position
                arm_servo_degree_pub.publish(arm_servo_degree)
                get_cmd_ik = False

        elif command == "a":
            print("\nmissiona 手动输入角度")
            num = input("请输入6个舵机角度: ")
            try:
                deg = list(map(int, num.split()))
                curr_ang=list(map(int, num.split()))
                
            except:
                print("输入错误")
                continue
            if len(deg)!=6:
                print("必须输入6个")
                continue
                
            #curr_ang = deg.copy()
            #saved_joint_angles = deg[:5]
                
            global saved_joint_angles
            saved_joint_angles = deg[:5]
            arm_servo_degree.position = radians_list(deg)
            arm_servo_degree.header.stamp = rospy.Time.now()
            arm_servo_degree_pub.publish(arm_servo_degree)
            
          
            
            print("已到达: ", deg)
            time.sleep(2.5)

        elif command == "b":
            print("\nmissionb 执行抓取")
            deg2 = curr_ang[:5] + [50]
            arm_servo_degree.position = radians_list(deg2)
            arm_servo_degree.header.stamp = rospy.Time.now()
            arm_servo_degree_pub.publish(arm_servo_degree)
            print("已设置: ", deg2)
            time.sleep(1.5)

            deg3 = [-1, -21, 40, 59, -1, 50]
            arm_servo_degree.position = radians_list(deg3)
            arm_servo_degree.header.stamp = rospy.Time.now()
            arm_servo_degree_pub.publish(arm_servo_degree)
            print("抬升完成")
            time.sleep(2)
            
            deg4 = [0, 0, 90, 90, 0, 30]
            arm_servo_degree.position = radians_list(deg4)
            arm_servo_degree.header.stamp = rospy.Time.now()
            arm_servo_degree_pub.publish(arm_servo_degree)
            time.sleep(2)
            
        elif command == "c":
            print("\nmissionc 执行放置")
            
            deg3 = [-1, -21, 40, 59, -1, 50]
            arm_servo_degree.position = radians_list(deg3)
            arm_servo_degree.header.stamp = rospy.Time.now()
            arm_servo_degree_pub.publish(arm_servo_degree)
            print("抬升完成")
            time.sleep(2)
            
            deg4 = [-100,-1,50, 60,-89,50]  
            arm_servo_degree.position = radians_list(deg4)
            arm_servo_degree.header.stamp = rospy.Time.now()
            arm_servo_degree_pub.publish(arm_servo_degree)
            time.sleep(2)
            
            deg4 = [-100,-1,50, 60,-89,130]
            arm_servo_degree.position = radians_list(deg4)
            arm_servo_degree.header.stamp = rospy.Time.now()
            arm_servo_degree_pub.publish(arm_servo_degree)
            time.sleep(2)
            
            deg4 = [0, 0, 90, 90, 0,130]
            arm_servo_degree.position = radians_list(deg4)
            arm_servo_degree.header.stamp = rospy.Time.now()
            arm_servo_degree_pub.publish(arm_servo_degree)
            time.sleep(2)        
            
            
            

#另粘一份进行修改
        elif command in ["g1","g2","g3","g4","g5","g6","b1","b2","b3","b4","b5","b6"]:#增加"g6"
            servo_limit = Arm.servo_degree_limits
            '''curr_ang = []
            for i in range(6):
                a = Arm.Arm_serial_servo_read(i+1) - delta_servo[i]
                curr_ang.append(a)'''
            
            idx = int(command[1])-1
            
            step = 8 if command[0]=='g' else -8

            # ✅ 只在这里修正 2 号舵机反向
            if idx == 1:
                step = -step
                
            

            new_ang = curr_ang[idx] + step

            print(f"修改前角度,{curr_ang[0]} {curr_ang[1]} {curr_ang[2]} {curr_ang[3]} {curr_ang[4]} {curr_ang[5]}。目标新角度：{new_ang}")#增加打印看信息
            mn,mx = servo_limit[idx]
            
            if new_ang < mn or new_ang > mx:
                print("已到达限位")
                continue

            # ✅ 真实写入舵机（编号完全正确）
            target = new_ang + delta_servo[idx]
            Arm.Arm_serial_servo_write(idx+1, target, 300)
            
            print(f"本次写入舵机{idx+1},加上漂移的目标值：{target}")#增加打印看信息

            # ✅ 完全保留你原来的 ROS 发布逻辑，一点不改
            curr_ang[idx] = new_ang
            arm_servo_degree.position = radians_list(curr_ang)
            arm_servo_degree.header.stamp = rospy.Time.now()
            arm_servo_degree_pub.publish(arm_servo_degree)

            
            

            
            print("舵机%d 调整%d度 当前%.1f度" % (idx+1, step, curr_ang[idx]))
            time.sleep(0.6)
            

        elif command == "wf":
            arm_wait(arm_front, claw_loose, arm_servo_degree, arm_servo_degree_pub)
        elif command == "wm":
            arm_wait(arm_middle, claw_loose, arm_servo_degree, arm_servo_degree_pub)
        elif command == "wb":
            arm_wait(arm_back, claw_loose, arm_servo_degree, arm_servo_degree_pub)

        elif command == "g":
            s = input("\n输入抓取坐标 x y z r p y: ")
            pose = list(map(float, s.split()))
            p = PoseStamped()
            p.header.frame_id = "car_base_link"
            p.pose.position.x = pose[0]/100.0
            p.pose.position.y = pose[1]/100.0
            p.pose.position.z = pose[2]/100.0
            q = quaternion_from_euler(np.radians(pose[3]), np.radians(pose[4]), np.radians(pose[5]))
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            tar_pose_car_pub.publish(p)
            tfm = tf_buffer.lookup_transform("%sarm_base_link"%arm_ns[1:], "car_base_link", rospy.Time(0))
            p_arm = do_transform_pose(p, tfm)
            arm_target_pose_pub.publish(p_arm)
            t0 = time.time()
            while not get_cmd_ik and time.time()-t0 < 2:
                pass
            if get_cmd_ik:
                grab(arm_ik_servo_degree.position, claw_loose, claw_tight, lr, arm_servo_degree, arm_servo_degree_pub)
                get_cmd_ik = False

        elif command == "p":
            put(arm_front, claw_loose, claw_tight, lr, arm_servo_degree, arm_servo_degree_pub)

        elif command == "q":
            print("\n退出程序")
            break
        else:
            print("\n未知指令")
        
        r.sleep()
        time.sleep(1)
    
if __name__ == '__main__':
    main()
