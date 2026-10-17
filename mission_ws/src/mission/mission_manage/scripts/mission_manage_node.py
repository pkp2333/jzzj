#!/usr/bin/python3
# -*- coding: utf-8 -*-
import re
import ast
import cv2
import time
import copy
import numpy as np
from scipy.spatial.transform import Rotation
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

img_raw = None
car_cmd = CarCmd()
left_grab_cmd = GrabCmd()
right_grab_cmd = GrabCmd()
arrived = False
yolov5_dets = ObjDets()
left_arm_ik_servo_degree = JointState()
left_arm_get_cmd_ik = False
right_arm_ik_servo_degree = JointState()
right_arm_get_cmd_ik = False
left_final_flag = GrabCmd.BUZY
right_final_flag = GrabCmd.BUZY

def image_raw_cb(msg:Image):
    global img_raw
    bridge = CvBridge()
    img_raw = bridge.imgmsg_to_cv2(msg, 'bgr8')


def car_cmd_cb(msg:CarCmd):
    global car_cmd, arrived
    if not arrived and (car_cmd.cmd == CarCmd.NAVIGATION or car_cmd.cmd == CarCmd.HOLD) and msg.cmd == CarCmd.ARRIVE:
        arrived = True
    elif arrived and msg.cmd == CarCmd.NAVIGATION:
        arrived = False    
    car_cmd = msg

def left_grab_final_cb(msg:GrabCmd):
    global left_final_flag
    left_final_flag = msg.cmd

def right_grab_final_cb(msg:GrabCmd):
    global right_final_flag
    right_final_flag = msg.cmd

def left_arm_ik_servo_degree_cb(msg:JointState):
    global left_arm_ik_servo_degree, left_arm_get_cmd_ik
    left_arm_ik_servo_degree = msg
    left_arm_get_cmd_ik = True


def right_arm_ik_servo_degree_cb(msg:JointState):
    global right_arm_ik_servo_degree, right_arm_get_cmd_ik
    right_arm_ik_servo_degree = msg
    right_arm_get_cmd_ik = True

# 根据机械臂选择果篮
def select_input_packet(dir,fruit_id,car_cmd_pub:rospy.Publisher):
    car_cmd_1 = CarCmd()
    
    if dir == 0:
        if fruit_id <=3:
            if fruit_id == 0:
                car_cmd_1.cmd = CarCmd.Position_1
            
            elif fruit_id == 1:
                car_cmd_1.cmd = CarCmd.Position_2
            
            elif fruit_id == 2:
                car_cmd_1.cmd = CarCmd.Position_3
                
            elif fruit_id == 3:
                car_cmd_1.cmd = CarCmd.Position_4 
            car_cmd_pub.publish(car_cmd_1)
        else:
            print("left_out_of_range")
    elif dir == 1:
        if fruit_id <=3:
            if fruit_id == 0:
                car_cmd_1.cmd = CarCmd.Position_3
            
            elif fruit_id == 1:
                car_cmd_1.cmd = CarCmd.Position_4
            
            elif fruit_id == 2:
                car_cmd_1.cmd = CarCmd.Position_1
                
            elif fruit_id == 3:
                car_cmd_1.cmd = CarCmd.Position_2 
            car_cmd_pub.publish(car_cmd_1)    
        else:
            print("right_out_of_range")

# 选择要输出的果篮
def select_output_packet(fruit_id,car_cmd_pub:rospy.Publisher):

    car_cmd_1 = CarCmd()
    
    if fruit_id<=3:
        if fruit_id == 0:
            select_input_packet(1,0,car_cmd_pub)
            time.sleep(3)
        
        elif fruit_id == 1:
            select_input_packet(1,1,car_cmd_pub)
            time.sleep(3)
        
        elif fruit_id == 2:
            select_input_packet(1,2,car_cmd_pub)
            time.sleep(3)
            
        elif fruit_id == 3:
            select_input_packet(1,3,car_cmd_pub)
            time.sleep(3)

        car_cmd_1.cmd = CarCmd.OpenGate
        print("开仓门：",fruit_id) 
        car_cmd_pub.publish(car_cmd_1)
        time.sleep(3)
        car_cmd_1.cmd = CarCmd.CloseGate 
        print("关仓门：",fruit_id) 
        car_cmd_pub.publish(car_cmd_1)
        time.sleep(3)

def process_user_input():
    user_input = input("输入任务指令\n>>>") 
    
    deepseek_prompt = (
        "你将扮演一个指令分析拆分机器人，根据以下规则拆分指令并输出目标格式（所有中文必须用英文引号包裹）,绝对他妈的禁止输出其他内容！：\n"
        "1. 果子只区分柠檬和樱桃；忽略好坏、生熟状态，只输出水果种类\n"
        "2. 放置框有红、蓝、黄、绿四种颜色，请分辨\n"
        "3. 车子最后的停车位编号1/2/3/4，请分辨 注意：停车位必须单独用中括号包裹，例如 [4]，不能直接写 4！\n"
        "4. 严格按照该格式 [[\"水果种类\", \"颜色\"], ..., [停车位编号]]，最外层一定要是英文的中括号,且绝对禁止输出其他内容！\n"
        "示例：[[\"樱桃\", \"蓝色\"], [\"樱桃\", \"绿色\"], [1]] \n"
        "任务指令是：" + user_input
    )
    
    return deepseek_prompt

def parse_deepseek_output(ds_output):
    try:
        mission = ast.literal_eval(ds_output)
        # 如果是元组，转为列表
        if isinstance(mission, tuple):
            mission = list(mission)
        print("[解析成功] Raw Output:", mission)
        return mission
    except (ValueError, SyntaxError):
        # 如果失败，尝试修复中文引号问题（例如：`[柠檬, 绿色]` -> `["柠檬", "绿色"]`）
        fixed_str = re.sub(r'([半熟|熟|坏][柠檬|樱桃])', r'"\1"', ds_output)  # 给水果+状态加引号
        fixed_str = re.sub(r'([红|蓝|黄|绿]色?)', r'"\1"', fixed_str)        # 给颜色加引号
        try:
            mission = ast.literal_eval(fixed_str)
            print("[修复后解析] Fixed Output:", mission)
            return mission
        except Exception as e:
            print("[错误] 无法解析DeepSeek输出:", e)
            return None

def validate_mission(mission):
    """检查任务结构是否合法"""
    if not isinstance(mission, list):
        return False
    if len(mission) < 5 or not isinstance(mission[4], list) or len(mission[4]) != 1:
        return False  # 必须包含4个抓取任务 + 1个停车位
    return True

def main():
    global arrived, left_arm_get_cmd_ik, right_arm_get_cmd_ik
    global left_final_flag, right_final_flag
    
    # 初始化两个标志位
    left_final_flag_ = GrabCmd.BUZY
    right_final_flag_ = GrabCmd.BUZY

    rospy.init_node('mission_manage_node', anonymous=True)

    sort_basket_rois = rospy.get_param('/mission_magage/sort_basket_rois', [[(400, 130), (480, 170)],
                                                                            [(120, 140), (160, 200)],
                                                                            [(0, 200), (50, 320)],
                                                                            [(500, 260), (640, 380)]])
    
    go_to_grab_num = rospy.get_param('/mission_magage/go_to_grab/wp_num', 0)
    go_to_grab_wp = []
    for i in range(go_to_grab_num):
        go_to_grab_wp.append(rospy.get_param(f'/mission_magage/go_to_grab/wp_{i+1}'))

    transit_wp = rospy.get_param('/mission_magage/transit_pt/wp_1')

    grab_num = rospy.get_param('/mission_magage/grab/wp_num', 0)
    grab_wp = []
    for i in range(grab_num):
        grab_wp.append(rospy.get_param(f'/mission_magage/grab/wp_{i+1}'))
        
    put_num = rospy.get_param('/mission_magage/put/wp_num', 0)
    put_wp = []
    for i in range(put_num):
        put_wp.append(rospy.get_param(f'/mission_magage/put/wp_{i+1}'))

    go_back_wp = rospy.get_param('/mission_magage/go_back/wp_1')

    park_num = rospy.get_param('/mission_magage/park/wp_num', 0)
    park_wp = []
    for i in range(park_num):
        park_wp.append(rospy.get_param(f'/mission_magage/park/wp_{i+1}'))
    
    grab_param_l = GrabParam()
    grab_param_r = GrabParam()

    t_larm_2_car = rospy.get_param('/arm_left/translation', [0, 0, 0])
    t_rarm_2_car = rospy.get_param('/arm_right/translation', [0, 0, 0])
    t_cam_2_car = rospy.get_param('/ascamera/translation', [0, 0, 0])
    rpy_cam_2_car = rospy.get_param('/ascamera/rotation', [-90, 0, -90])
    sight_middle_h = rospy.get_param('/grab_param/sight_middle_h', 300)
    sight_middle_w = rospy.get_param('/grab_param/sight_middle_w', [269, 457])
    x_obj_to_cam_close = rospy.get_param('/grab_param/x_obj_to_cam_close', 0.205)
    x_obj_to_cam_middle = rospy.get_param('/grab_param/x_obj_to_cam_middle', 0.26)
    x_obj_to_cam_far = rospy.get_param('/grab_param/x_obj_to_cam_far', 0.34)
    x_j5_2_claw = rospy.get_param('/grab_param/x_j5_2_claw', 0.085)
    z_j5_2_claw = rospy.get_param('/grab_param/z_j5_2_claw', 0.0115)
    l_j5_2_claw = np.sqrt(x_j5_2_claw**2 + z_j5_2_claw**2)
    theta_j5_2_claw = np.rad2deg(np.arctan2(z_j5_2_claw, x_j5_2_claw))
    left_arm_front = rospy.get_param('turn_front', 90)
    left_arm_middle = rospy.get_param('turn_middle', 0)
    left_arm_back = rospy.get_param('turn_back', -90)
    left_arm_tight = rospy.get_param('/arm_left/claw_tight', 30)
    left_arm_loose = rospy.get_param('/arm_left/claw_loose', 60)
    right_arm_front = rospy.get_param('/arm_right/turn_front', 90)
    right_arm_middle = rospy.get_param('/arm_right/turn_middle', 0)
    right_arm_back = rospy.get_param('/arm_right/turn_back', -90)
    right_arm_tight = rospy.get_param('/arm_right/claw_tight', 30)
    right_arm_loose = rospy.get_param('/arm_right/claw_loose', 60)

    mission_time = rospy.get_param('/mission/mission_time', 300) - 25  # 留出返回时间

    image_raw_sub = rospy.Subscriber('/ascamera_hp60c/rgb0/image', Image, image_raw_cb)
    car_cmd_sub = rospy.Subscriber('/car_cmd', CarCmd, car_cmd_cb)

    left_arm_servo_degree_sub = rospy.Subscriber("/arm_left/arm_ik_servo_degree", JointState, left_arm_ik_servo_degree_cb)
    left_final_flag_sub = rospy.Subscriber('/left_final_flag', GrabCmd, left_grab_final_cb)
    right_arm_servo_degree_sub = rospy.Subscriber("/arm_right/arm_ik_servo_degree", JointState, right_arm_ik_servo_degree_cb)
    right_final_flag_sub = rospy.Subscriber('/right_final_flag', GrabCmd, right_grab_final_cb)

    ds_task_pub = rospy.Publisher('/deepseek/task', Bool, queue_size=1)
    ds_input_pub = rospy.Publisher('/deepseek/input_text', String, queue_size=1)
    at_target_pub = rospy.Publisher('/at_target_id', Int16, queue_size=1)
    car_target_pub = rospy.Publisher('/car_target', PoseStamped, queue_size=1)
    exit_condition_pub = rospy.Publisher('exit_condition', ExitCondition, queue_size=1)
    car_cmd_pub = rospy.Publisher('/car_cmd', CarCmd, queue_size=3)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
    left_grab_flag_pub = rospy.Publisher('/left_grab_flag', GrabCmd, queue_size=1)
    right_grab_flag_pub = rospy.Publisher('/right_grab_flag', GrabCmd, queue_size=1)
    cam_orie_pub = rospy.Publisher('/orientation_bool', Bool, queue_size=1)

    camera_info_msg: CameraInfo = rospy.wait_for_message('/ascamera_hp60c/rgb0/camera_info', CameraInfo, timeout=None)

    # # # 出发前订阅相机画面，检测放置框的顺序
    r = rospy.Rate(30)
    seq_lists = []
    while not rospy.is_shutdown():
        if img_raw is not None:
            basket_color_seq_ = sort_basket_color_seq(img_raw, sort_basket_rois)
            seq_lists.append(basket_color_seq_)
            if len(seq_lists) == 10:
                break
        r.sleep()
    cv2.destroyAllWindows() 
    basket_color_seq = find_most_seq(seq_lists)

    # basket_color_seq = [0, 3, 2, 1]
    # print("放置区顺序: ", basket_color_seq)
    # select_output_packet(1, car_cmd_pub)

    # ################################################################################
    # 输入任务指令 等待deepseek输出
    
    ds_input = String()
    ds_input.data = process_user_input()
    ds_input_pub.publish(ds_input)
     
    t0 = time.time()
    rospy.loginfo("任务开始\n")

    # ################################################################################
    # 等待deepseek处理得到的结果
    
    try:
       ds_output = rospy.wait_for_message('/deepseek/output_text', String, timeout=None)
       print("[接收] DeepSeek原始输出:", ds_output.data)
    except rospy.ROSException:
       print("[错误] 等待DeepSeek响应超时！")
       return    
    
    mission = parse_deepseek_output(ds_output.data)
    if not mission or not validate_mission(mission):
       print("[错误] 任务格式无效！")
       return
            
    # ################################################################################

    # # ################################################################################
    # # 手动指定结果 仅供调试使用
    # ds_output = String()
    
    ds_output.data = "[[柠檬, 绿色], [柠檬, 蓝色], [樱桃, 红色], [樱桃, 黄色], [2]]"
    # # ################################################################################

    # 处理deepseek的输出，将字符串转换为列表
    orig_str = ds_output.data
    if "色" in orig_str:
        orig_str = orig_str.replace("色", "")
    try:
        mission = ast.literal_eval(orig_str)
        print(mission)
    except ValueError:
        processed = re.sub(r'(?<=[\[,\s])([\u4e00-\u9fa5]+)(?=[,\]\s])', r'"\1"', orig_str)
        mission = ast.literal_eval(processed)
        print(mission)
    # 将列表中的任务转换为代号，便于后续任务执行
    grab_mission = mission[:4]
    park_mission = mission[4][0]
    for item in grab_mission:
        if isinstance(item, list) and len(item) == 2:
            if item[0] in fruit_dict:
                item[0] = fruit_dict[item[0]]
            if item[1] in color_dict:
                item[1] = color_dict[item[1]]
    fruit_cmd = [grab_mission[i][0] for i in range(4)]
    basket_color_cmd = [grab_mission[i][1] for i in range(4)]
    
    print("抓取放置任务配对: ", grab_mission)  # [[4, 3], [0, 2], [1, 0], [5, 1]]
    print("泊车任务: ", park_mission)  # 2
    
    # 结束deepseek节点 释放内存
    ds_task_pub.publish(Bool(False))
    
    grab_param_l.cx = camera_info_msg.K[2]
    grab_param_l.cy = camera_info_msg.K[5]
    grab_param_l.fx = camera_info_msg.K[0]
    grab_param_l.fy = camera_info_msg.K[4]
    grab_param_l.t_arm_2_car = t_larm_2_car
    grab_param_l.t_cam_2_car = t_cam_2_car
    grab_param_l.rpy_cam_2_car = rpy_cam_2_car
    grab_param_l.sight_middle_h = sight_middle_h
    grab_param_l.sight_middle_w = sight_middle_w
    grab_param_l.x_obj_to_cam_close = x_obj_to_cam_close
    grab_param_l.x_obj_to_cam_middle = x_obj_to_cam_middle
    grab_param_l.x_obj_to_cam_far = x_obj_to_cam_far
    grab_param_l.l_j5_2_claw = l_j5_2_claw
    grab_param_l.theta_j5_2_claw = theta_j5_2_claw
    grab_param_r.cx = camera_info_msg.K[2]
    grab_param_r.cy = camera_info_msg.K[5]
    grab_param_r.fx = camera_info_msg.K[0]
    grab_param_r.fy = camera_info_msg.K[4]
    grab_param_r.t_arm_2_car = t_rarm_2_car
    grab_param_r.t_cam_2_car = t_cam_2_car
    grab_param_r.rpy_cam_2_car = rpy_cam_2_car
    grab_param_r.sight_middle_h = sight_middle_h
    grab_param_r.sight_middle_w = sight_middle_w
    grab_param_r.x_obj_to_cam_close = x_obj_to_cam_close
    grab_param_r.x_obj_to_cam_middle = x_obj_to_cam_middle
    grab_param_r.x_obj_to_cam_far = x_obj_to_cam_far
    grab_param_r.l_j5_2_claw = l_j5_2_claw
    grab_param_r.theta_j5_2_claw = theta_j5_2_claw
    arm_servo_degree = JointState()
    arm_servo_degree.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    at_target = Int16()
    car_target = PoseStamped()
    exit_condition = ExitCondition()

    # while(1):
    #     # print("test")
    #     left_grab_flag = GrabCmd.START
    #     right_grab_flag = GrabCmd.START

    #     left_grab_flag_pub.publish(left_grab_flag)
    #     right_grab_flag_pub.publish(right_grab_flag)

    #     left_final_flag_ = left_final_flag
    #     right_final_flag_ = right_final_flag

    #     detect_round_cnt = 0
        
    #     # 程序堵塞,直到两个机械臂全部抓取完成
    #     while left_final_flag_ == GrabCmd.BUZY or right_final_flag_ == GrabCmd.BUZY:
    #         # 超时检测
    #         if detect_round_cnt >= 15:
    #             print("超时哩")
    #             break
    #         # 通过回调函数读取抓取结果
    #         left_final_flag_ = left_final_flag
    #         right_final_flag_ = right_final_flag

    #         detect_round_cnt += 1
    #         rospy.sleep(1)        
    #     put_basket_count = 0
    #     for put_basket_count in range(4):
    #         bakset_color = basket_color_seq[put_basket_count]
    #         rotate_idx = fruit_cmd.index(bakset_color)
    #         rotate_num = fruit_cmd[rotate_idx]
    #         print("前往"+str(put_basket_count+1)+"号位\n")
    #         arrived = False
    #         camera_dir = True
    #         if(put_basket_count == 1):
    #             chassis_drive_y(-0.15, car_cmd_pub, cmd_vel_pub)
    #             chassis_turn(-65, car_cmd_pub, cmd_vel_pub)
    #             print("开始旋转65")
    #         elif(put_basket_count == 3):# 向后退
    #             camera_dir = False
    #             chassis_turn(150.0, car_cmd_pub, cmd_vel_pub)

    #         pub_wp(put_wp[put_basket_count], 
    #             car_cmd, at_target, car_target, exit_condition,
    #             car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub, camera_dir, cam_orie_pub)

    #         while not rospy.is_shutdown():
    #             if not arrived:
    #                 rospy.sleep(1)
    #             else:
    #                 rospy.sleep(1)
    #                 print("到达"+str(put_basket_count+1)+"号位置")
    #                 put_basket_count += 1
    #                 break
            # time.sleep(1)
            # select_output_packet(rotate_num, car_cmd_pub)

    chassis_drive_y(-0.95, car_cmd_pub, cmd_vel_pub)  

    print("\n前往中间点...\n")

    arrived = False
    pub_wp(transit_wp, 
           car_cmd, at_target, car_target, exit_condition,
           car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub, True, cam_orie_pub)
    while not rospy.is_shutdown():
        if not arrived:
            rospy.sleep(1)
        else:
            break

    # 采摘/运输
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    grab_flag = 1 # 用于判断当前位置果子是否全部抓完
    grab_wp_cnt = 0 # 用于前往下一个位置
    test_flag = 1
    detect_round_cnt = 0
    
    while not rospy.is_shutdown() and (time.time() - t0 < mission_time) and grab_wp_cnt < grab_num:
        # 前往采摘位置
        if grab_flag:
            print(f"\n前往{grab_wp_cnt+1}号采摘位置...\n")
            arrived = False
            pub_wp(grab_wp[grab_wp_cnt], 
                 car_cmd, at_target, car_target, exit_condition,
                 car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub, True, cam_orie_pub)
            while not rospy.is_shutdown():
                if not arrived:
                    time.sleep(1)
                else:
                    print("到达"+str(grab_wp_cnt+1)+"号位")
                    if(test_flag == 1):
                        chassis_drive_x(0.075, car_cmd_pub, cmd_vel_pub)
                        chassis_drive_y(0.035, car_cmd_pub, cmd_vel_pub)
                        test_flag = 0
                    elif(test_flag == 0):
                        chassis_drive_x(0.075, car_cmd_pub, cmd_vel_pub)
                        chassis_drive_y(-0.045, car_cmd_pub, cmd_vel_pub)
                    
                    left_final_flag = GrabCmd.BUZY #默认为忙碌，防止标志位更新不及时导致跳过抓取点
                    right_final_flag = GrabCmd.BUZY

                    grab_flag = 0
                    time.sleep(1)
                    break

        # 发布识别抓取标志位，左右臂开始抓取
        left_grab_flag = GrabCmd.START
        right_grab_flag = GrabCmd.START

        left_grab_flag_pub.publish(left_grab_flag)
        right_grab_flag_pub.publish(right_grab_flag)

        left_final_flag_ = left_final_flag
        right_final_flag_ = right_final_flag
        
        # 程序堵塞,直到两个机械臂全部抓取完成
        # while left_final_flag_ == GrabCmd.BUZY or right_final_flag_ == GrabCmd.BUZY:
        #     # 超时检测
        #     if detect_round_cnt >= 15:
        #         print("超时哩")
        #         break
        #     # 通过回调函数读取抓取结果
        #     left_final_flag_ = left_final_flag
        #     right_final_flag_ = right_final_flag

        #     detect_round_cnt += 1
        #     rospy.sleep(1)

        # (好像FALSE打错字了，懒的改了)如果任意边机械臂抓取出现异常则后退重新导航调整姿态
        if left_final_flag_ == GrabCmd.FALSE or right_final_flag_ == GrabCmd.FALSE:
            print("后退调整姿态")
            chassis_drive_x(-0.10, car_cmd_pub, cmd_vel_pub)
            chassis_drive_y(-0.035 , car_cmd_pub, cmd_vel_pub)
            grab_flag = 1

        # 左右两边均抓取成功则到下一个摘果子位置
        if left_final_flag_ == GrabCmd.FIANL and right_final_flag_ == GrabCmd.FIANL:
            grab_wp_cnt += 1
            detect_round_cnt = 0
            grab_flag = 1

    print("\n前往中间点...\n")
    arrived = False
    pub_wp(transit_wp, 
        car_cmd, at_target, car_target, exit_condition,
        car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub, True, cam_orie_pub)
    while not rospy.is_shutdown():
        if not arrived:
            rospy.sleep(1)
        else:
            break

    chassis_turn(70, car_cmd_pub, cmd_vel_pub)

    camera_dir = True
    put_basket_count = 0
    while(put_basket_count < 4):
        bakset_color = basket_color_seq[put_basket_count]
        rotate_idx = fruit_cmd.index(bakset_color)
        rotate_num = fruit_cmd[rotate_idx]
        print("前往"+str(put_basket_count+1)+"号位\n")
        arrived = False
        camera_dir = True
        if(put_basket_count == 1):
            chassis_drive_y(-0.15, car_cmd_pub, cmd_vel_pub)
            chassis_turn(-65, car_cmd_pub, cmd_vel_pub)
        elif(put_basket_count == 3):# 向后退
            camera_dir = False
            chassis_turn(150.0, car_cmd_pub, cmd_vel_pub)

        pub_wp(put_wp[put_basket_count], 
            car_cmd, at_target, car_target, exit_condition,
            car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub, camera_dir, cam_orie_pub)

        while not rospy.is_shutdown():
            if not arrived:
                rospy.sleep(1)
            else:
                rospy.sleep(1)

                print("到达"+str(put_basket_count+1)+"号位置")
                put_basket_count += 1
                break
        time.sleep(1)
        select_output_packet(rotate_num, car_cmd_pub)

    print("\n返回\n")
    arrived = False
    pub_wp(go_back_wp, 
           car_cmd, at_target, car_target, exit_condition,
           car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub, False, cam_orie_pub)
    while not rospy.is_shutdown():
        if not arrived:
            rospy.sleep(1)
        else:
            break

    
    print(f"\n停入 {park_mission}号 车位\n")
    arrived = False
    chassis_drive_y(0.38*park_mission, car_cmd_pub, cmd_vel_pub) 

    pub_wp(park_wp[park_mission-1], 
           car_cmd, at_target, car_target, exit_condition,
           car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub, True, cam_orie_pub)
    while not rospy.is_shutdown():
        if not arrived:
            time.sleep(1)
        else:
            break
      

if __name__ == "__main__":
    main()
