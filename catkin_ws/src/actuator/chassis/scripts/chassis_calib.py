#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-
import serial
import time
import yaml


head = [0xa5]
tail = [0x00, 0x5a]
disarm = 0
arm = 1
vel_mode = 2

yaml_file = open("/home/orangepi/catkin_ws/src/user_params/params/chassis_config.yaml", 'r')
yaml_data = yaml.safe_load(yaml_file)
serial_port = yaml_data["chassis"]["serial_port"]
serial_baudrate = yaml_data["chassis"]["serial_baudrate"]
l_0x40 = yaml_data["chassis"]["l_0x40"]
a_0x40 = yaml_data["chassis"]["a_0x40"]


ser = serial.Serial(port=serial_port, baudrate=serial_baudrate, timeout=None)  # 打开串口


def drive_calib(mode):
    """
    速度校准 (认为线速度/角速度与电压成近似线性关系 取半电压为基准运行1s 测量实际运行情况)

    Args:
        mode:               校准模式  linear 线速度校准   angular 角速度校准
    """
    calib_data = []
    stop_data = head + [vel_mode & 0xff] + [0x00, 0x00, 0x00] + tail
    
    if mode == "linear":
        calib_data = head + [vel_mode & 0xff] + [0x40, 0x00, 0x00] + tail  # linear.x send  0x40 (64)
    elif mode == "angular":
        calib_data = head + [vel_mode & 0xff] + [0x00, 0x00, 0x40] + tail  # angular.z send 0xc0 (-64) 为方便配合手机指南针功能中的角度定义使用右转
    if ser.is_open:
        ser.write(calib_data)

        time.sleep(1)
        
        ser.write(stop_data)

        time.sleep(.1)


def chassis_turn(degree=0):
    """
    在当前位置原地旋转一定角度 方向满足右手螺旋定则

    Args:
        cmd_chassis:        底盘控制 (共享)
        degree:             旋转的角度
    """
    # 转向角度不为0
    if degree:
        print(f"开始旋转 {degree}°")
        # 向目标方向以半速开始旋转
        if degree>= 0:
            ser.write( head + [vel_mode & 0xff] + [0x00, 0x00, 0xc0] + tail)
        else:
            ser.write( head + [vel_mode & 0xff] + [0x00, 0x00, 0x40] + tail)
        # 延时特定时间
        time.sleep(abs(degree) / a_0x40)
        # 速度归零
        ser.write(head + [vel_mode & 0xff] + [0x00, 0x00, 0x00] + tail)
        # 保险起见 等待一定时间
        time.sleep(.05)
        print(f"完成旋转 {degree}°")


def chassis_drive(distance=0):
    """
    在当前位置前后运动一定距离

    Args:
        cmd_chassis:        底盘控制 (共享)
        distance:           前后运动的距离 cm
    """
    # 距离不为0
    if distance:
        print(f"开始行驶 {distance}cm")
        # 向目标方向以半速开始旋转
        if distance >= 0:
            ser.write(head + [vel_mode & 0xff] + [0x40, 0x00, 0x00] + tail)
        else:
            ser.write(head + [vel_mode & 0xff] + [0xc0, 0x00, 0x00] + tail)
        # 延时特定时间
        time.sleep(abs(distance/100) / l_0x40 + 0.06)  # + 启动耗时
        # 速度归零
        ser.write( head + [vel_mode & 0xff] + [0x00, 0x00, 0x00] + tail)
        # 保险起见 等待一定时间
        time.sleep(.05)
        print(f"完成行驶 {distance}cm")


def chassis_drive_(distance=0):
    """
    在当前位置前后运动一定距离

    Args:
        cmd_chassis:        底盘控制 (共享)
        distance:           前后运动的距离 cm
    """
    # 距离不为0
    if distance:
        print(f"开始行驶 {distance}cm")
        # 向目标方向以半速开始旋转
        if distance >= 0:
            ser.write(head + [vel_mode & 0xff] + [0x00, 0x40, 0x00] + tail)
        else:
            ser.write(head + [vel_mode & 0xff] + [0x00, 0xc0, 0x00] + tail)
        # 延时特定时间
        time.sleep(abs(distance/100) / l_0x40 + 0.06)  # + 启动耗时
        # 速度归零
        ser.write( head + [vel_mode & 0xff] + [0x00, 0x00, 0x00] + tail)
        # 保险起见 等待一定时间
        time.sleep(.05)
        print(f"完成行驶 {distance}cm")


if __name__ == "__main__":

    while 1:
        print("\n代号      指令")
        print("se        启动引擎")
        print("cl        校准线速度")
        print("ca        校准角速度")
        print("d         前后移动")
        print("cyy         前后移动")
        print("t         原地转向")
        print("ke        关闭引擎")
        print("q         退出")

        command = input("\n请输入指令代码: \n>>>")
        if command == "se":
            print("\n当前指令  启动引擎\n")
            ser.write(head + [arm & 0xff] + [0x00, 0x00, 0x00] + tail)
            time.sleep(1)
        elif command == "cl":
            print("\n当前指令  校准线速度\n")
            drive_calib('linear')
        elif command == "ca":
            print("\n当前指令  校准角速度\n")
            drive_calib('angular')
        elif command == "d":
            print("\n当前指令  前后移动\n")
            distance = input("\n请输入前进距离(cm): \n>>>")
            chassis_drive(int(distance))
        elif command == "cyy":
            print("\n当前指令  水平移动\n")
            distance = input("\n请输入水平移动距离(cm): \n>>>")
            chassis_drive_(int(distance))
        elif command == "t":
            print("\n当前指令  原地转向\n")
            degree = input("\n请输入目标角度(满足右手螺旋定则 °): \n>>>")
            chassis_turn(int(degree))
        elif command == "ke":
            print("\n当前指令  关闭引擎\n")
            ser.write(head + [disarm & 0xff] + [0x00, 0x00, 0x00] + tail)
            time.sleep(1)
        elif command == "q":
            print("\n当前指令  退出\n")
            break
        else:
            print("\n未知指令 请重新输入\n")
