#!/usr/bin/env python
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

import rospy
from std_msgs.msg import Bool, String, Int16 
from sensor_msgs.msg import CameraInfo, Image, JointState
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped
from user_msgs.msg import CarCmd, ExitCondition, ObjDet, ObjDets


img_raw = None
car_cmd = CarCmd()
arrived = False
yolov5_dets = ObjDets()
left_arm_ik_servo_degree = JointState()
left_arm_get_cmd_ik = False
right_arm_ik_servo_degree = JointState()
right_arm_get_cmd_ik = False


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


def yolov5_dets_cb(msg:ObjDets):
    global yolov5_dets
    yolov5_dets = msg


def left_arm_ik_servo_degree_cb(msg:JointState):
    global left_arm_ik_servo_degree, left_arm_get_cmd_ik
    left_arm_ik_servo_degree = msg
    left_arm_get_cmd_ik = True


def right_arm_ik_servo_degree_cb(msg:JointState):
    global right_arm_ik_servo_degree, right_arm_get_cmd_ik
    right_arm_ik_servo_degree = msg
    right_arm_get_cmd_ik = True

# 根据机械臂选择果篮
def select_input_packet(lr,fruit_id,car_cmd_pub:rospy.Publisher):
    car_cmd_1 = CarCmd()
    
    if lr:
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
            
    else:
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
            car_cmd_1.cmd = CarCmd.OpenGate
        
        elif fruit_id == 1:
            select_input_packet(1,1,car_cmd_pub)
            time.sleep(3)
            car_cmd_1.cmd = CarCmd.OpenGate
        
        elif fruit_id == 2:
            select_input_packet(1,2,car_cmd_pub)
            time.sleep(3)
            car_cmd_1.cmd = CarCmd.OpenGate
            
        elif fruit_id == 3:
            select_input_packet(1,3,car_cmd_pub)
            time.sleep(3)
            car_cmd_1.cmd = CarCmd.OpenGate
        print("开仓门：",fruit_id) 
        car_cmd_pub.publish(car_cmd_1)
        time.sleep(10)
        car_cmd_1.cmd = CarCmd.CloseGate 
        print("关仓门：",fruit_id) 
        car_cmd_pub.publish(car_cmd_1)
    else:
            print("right_out_of_range")

def process_user_input():
    # 1. 获取用户输入（只提示一次）
    user_input = input("输入任务指令\n>>>")  # 例如："把柠檬放绿色框，樱桃放红色框，停到2号车位"
    
    # 2. 构造DeepSeek的指令（避免长字符串拼接，提高可读性）
    deepseek_prompt = (
        "你将扮演一个指令分析拆分机器人，根据以下规则拆分指令并输出目标格式,绝对禁止输出其他内容！：\n"
        "1. 果子只区分柠檬和樱桃；忽略好坏、生熟状态，只输出水果种类\n"
        "2. 放置框有红、蓝、黄、绿四种颜色，请分辨\n"
        "3. 车子最后的停车位编号1/2/3/4，请分辨 注意：停车位必须单独用中括号包裹，例如 [4]，不能直接写 4！\n"
        "4. 输出格式必须严格遵循：[[水果种类, 框颜色], [水果种类，框颜色] , ..., [停车位]]，且绝对禁止输出其他内容！\n"
        "例如：[[柠檬, 绿色], [樱桃, 红色], [2]]\n"
        "任务指令是：" + user_input
    )
    
    return deepseek_prompt

def parse_deepseek_output(ds_output):
    """解析DeepSeek返回的字符串，并转换为Python列表"""
    try:
        # 尝试直接解析（如果已经是合法格式）
        mission = ast.literal_eval(ds_output)
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


def main():
    global arrived, left_arm_get_cmd_ik, right_arm_get_cmd_ik

    rospy.init_node('mission_manage_node', anonymous=True)

    mission_time = rospy.get_param('/mission/mission_time', 300) - 25  # 留出返回时间

    ds_task_pub = rospy.Publisher('/deepseek/task', Bool, queue_size=1)
    ds_input_pub = rospy.Publisher('/deepseek/input_text', String, queue_size=1)
    at_target_pub = rospy.Publisher('/at_target_id', Int16, queue_size=1)
    
    print("1")
    
    # ################################################################################
    # 输入任务指令 等待deepseek输出
    ds_input = String()
    # input_temp = input("输入任务指令\n>>>")
    # print("输入任务指令\n")
    # time.sleep(5)
    # ds_input.data = "你将扮演一个指令分析拆分机器人，将根据以下规则拆分指令并输出目标格式；第一，输入的果子将有柠檬和樱桃，它们均有三种状态，分别是半熟，熟，坏，分辨柠檬或者樱桃状态；第二，放置框有四种颜色，分别是红色，蓝色，黄色，绿色，请分辨颜色；第三，停车位有四个编号，分别是1,2,3,4,请分辨停车位编号；第四，请将上述分辨的内容以格式[[柠檬，蓝色],[樱桃，红色],...,[2]]的二维矩阵形式输出，绝对禁止输出其他内容！ " \
    #                    + input_temp + "。"
    # ds_input_pub.publish(ds_input)
    # ################################################################################

    ds_input.data = process_user_input()
    ds_input_pub.publish(ds_input)
    
    t0 = time.time()
    rospy.loginfo("任务开始\n")

    # ################################################################################
    # 等待deepseek处理得到的结果
    # ds_output = String()
    # ds_output: String = rospy.wait_for_message('/deepseek/output_text', String, timeout=None)
    
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
    
    # ds_output.data = "[[柠檬, 绿色], [柠檬, 蓝色], [樱桃, 红色], [樱桃, 黄色], [2]]"
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
    
    while 1:
      print("test")
      time.sleep(10)
    

if __name__ == "__main__":
    main()
