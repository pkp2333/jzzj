#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import serial
import threading
import math
import os
import sys
from functools import wraps
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from stage_c_base_safety import BaseLease

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from user_msgs.msg import CarCmd


def serialized(method):
    @wraps(method)
    def call(self,*args,**kwargs):
        with self.io_lock:return method(self,*args,**kwargs)
    return call


def threshold(x, dead_zone=0, activate_zone=0, clamp=0):
    if x > dead_zone:
        return min((x-dead_zone) + activate_zone, clamp)
    elif x < -dead_zone:
        return max((x+dead_zone) - activate_zone, -clamp)
    else:
        return 0
    

class ChassisNode:
    def __init__(self) -> None:
        self.io_lock=threading.RLock()
        self.micro=BaseLease(
            enabled=rospy.get_param('~stage_c_micro/enabled',False),
            calibrated=rospy.get_param('~stage_c_micro/calibrated',False),
            watchdog_verified=rospy.get_param('~stage_c_micro/controller_watchdog_verified',False))
        self.micro_stop_written=False
        print()
        rospy.loginfo(f"底盘节点初始化中...\n")

        serial_port = rospy.get_param("chassis/serial_port", "/dev/ttyS0")
        serial_baudrate = rospy.get_param("chassis/serial_baudrate", 115200)
        l_0x40 = rospy.get_param("chassis/l_0x40", 0.5)
        a_0x40 = rospy.get_param("chassis/a_0x40", 100)
        self.xy_dead_zone = rospy.get_param("chassis/dead_zone_xy", 0.0)
        self.xy_activate_zone = rospy.get_param("chassis/activate_zone_xy", 0.0)
        self.z_dead_zone = rospy.get_param("chassis/dead_zone_z", 0.0)
        self.z_activate_zone = rospy.get_param("chassis/activate_zone_z", 0.0)
        self.max_vel_linear = rospy.get_param("chassis/max_vel_linear", 0.5)
        self.max_vel_angular = rospy.get_param("chassis/max_vel_angular", 48)
        print("self.max_vel_angular:",self.max_vel_angular)
        self.RA2DE = 180 / 3.1415926
        self.head = [0xa5]
        self.tail = [0x00, 0x5a]

        # 模式位
        self.disarm   = 0
        self.arm      = 1
        self.vel_mode = 2
        self.basket_mode = 3

        a_0x40 = a_0x40 / self.RA2DE  # ° 转 rad
        self.vel_2_0x40 = 64 / l_0x40  # 分母为 0x40/(+64)校准时 1s内行驶的距离 单位 m
        self.omega_2_0x40 = 64 / a_0x40  # 分母为 最大值0x40/(+64)校准时 1s内旋转的角度 单位 °

        self.z_dead_zone = self.z_dead_zone / self.RA2DE
        self.z_activate_zone = self.z_activate_zone / self.RA2DE
        self.max_vel_angular = self.max_vel_angular / self.RA2DE

        self.task = True
        self.delled = False
        
        try:
            self.ser = serial.Serial(port=serial_port, baudrate=serial_baudrate, timeout=None, write_timeout=.1)
        except Exception as e:
            print(e)
            exit(f"tty串口 {serial_port} @ 波特率-{serial_baudrate} 打开失败")

        if self.ser.is_open:
            pass
        else:
            exit(f"tty串口 {serial_port} @ 波特率-{serial_baudrate} 打开失败")

        self.chassis_cmd_data = self.head + [self.arm & 0xff] + [0x00, 0x00, 0x00] + self.tail
        self.ser.write(self.chassis_cmd_data)
        rospy.loginfo(f"\33[32m引擎启动\33[0m\n")

        self.stop_data = self.head + [self.vel_mode & 0xff] + [0x00, 0x00, 0x00] + self.tail
        self.ser.write(self.stop_data)
        self.chassis_cmd_data = self.stop_data
        self.last_chassis_cmd_data = self.chassis_cmd_data
        self.last_cmd_vel = [0.0, 0.0, 0.0]
        self.car_cmd = CarCmd.HOLD

        self.basket_data = self.head + [self.basket_mode & 0xff] + [0x00, 0x00, 0x00] + self.tail
        self.ser.write(self.basket_data)
        self.last_basket_test = [0,0,0]

        self.car_task_sub = rospy.Subscriber("/task", Bool, self.task_cb)
        self.car_cmd_sub = rospy.Subscriber("/car_cmd", CarCmd, self.car_cmd_cb)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb) 
        if rospy.get_param('~stage_c_micro/enabled',False):
            from user_msgs.msg import StageCBaseCommand, StageCBaseStatus
            self.micro_status_type=StageCBaseStatus
            self.micro_pub=rospy.Publisher('/stage_c/base/status',StageCBaseStatus,queue_size=1)
            self.micro_sub=rospy.Subscriber('/stage_c/base/command',StageCBaseCommand,self.micro_cb,queue_size=1)
        
        self.controller_timer = rospy.Timer(rospy.Duration(0.05), self.controller)
        
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(f"底盘控制启动成功\n")
    
    @serialized
    def shutdown(self):
        if not self.delled and self.ser and self.ser.is_open:
            cmd_data = self.head + [self.disarm & 0xff] + [0x00, 0x00, 0x00] + self.tail
            try:
                self.ser.write(cmd_data)
                rospy.loginfo("\33[31m引擎关闭\33[0m")
            except Exception as e:
                rospy.logwarn(f"引擎关闭命令发送失败: {e}")
            try:
                self.ser.close()
                rospy.loginfo("底盘控制成功结束")
            except Exception as e:
                rospy.logwarn(f"串口关闭失败: {e}")
            finally:
                self.delled = True

    @staticmethod
    def gen_body(data):
        """
        将输入的数据转为16进制的高8位低8位

        Args:
            data:               输入数据

        Returns:
            16进制数据, [高8位, 低8位, 高8位, 低8位, ...]
        """
        data_body = []
        for i in range(3):
            if data[i] > 127:
                data[i] = 127
            elif data[i] < -128:
                data[i] = -128
            data_body.append(data[i] & 0xff)  # 低8位
        return data_body
    
    @serialized
    def task_cb(self, msg:Bool):
        if not msg.data:
            with self.io_lock:
                self.micro.fault('task stop requested');self.clear_motion()
                self.ser.write(self.stop_data)
        self.task = msg.data
        if not self.task:
            rospy.logwarn("关闭底盘控制\n")
        
    @serialized
    def car_cmd_cb(self, msg:CarCmd):
        with self.io_lock:
            if msg.cmd in (CarCmd.HOLD,CarCmd.ARRIVE):
                self.micro.fault('external stop requested');self.clear_motion()
                self.ser.write(self.stop_data)
            elif self.micro.owner:
                return  # Includes basket commands: no shared-serial interference.
        self.car_cmd = msg.cmd

        if self.car_cmd == CarCmd.OpenGate:
            self.basket_control(1,5)
            print("打开仓门")
            print("cangmen_basket_data",self.basket_data)
            self.ser.write(self.basket_data)
        elif self.car_cmd == CarCmd.CloseGate:
            self.basket_control(0,5)
            print("关闭仓门")
            print("cangmen_basket_data",self.basket_data)
            self.ser.write(self.basket_data) 
        elif self.car_cmd >= 7:
            if self.car_cmd == CarCmd.Position_1:
                self.basket_control(0,0)    # 0°
                print("旋转到位置0")
            elif self.car_cmd == CarCmd.Position_2:
                self.basket_control(0,1)   # 90°
                print("旋转到位置1")
            elif self.car_cmd == CarCmd.Position_3:
                self.basket_control(0,2)  # 180°
                print("旋转到位置2")
            elif self.car_cmd == CarCmd.Position_4:
                self.basket_control(0,3)  # 
                print("旋转到位置3")
            print("basket_data",self.basket_data)
            self.ser.write(self.basket_data)

        
    @serialized
    def cmd_vel_cb(self, msg:Twist):
        with self.io_lock:
            if self.micro.owner:
                if all(v==0 for v in (msg.linear.x,msg.linear.y,msg.angular.z)):
                    self.micro.fault('external zero velocity stop');self.clear_motion()
                    self.ser.write(self.stop_data)
                return
        # cmd_vel = [msg.cmd_vel.linear.x, msg.cmd_vel.linear.y, msg.cmd_vel.angular.z * self.RA2DE]
        # if cmd_vel != self.last_cmd_vel:
        #     # print(f"\ncurrently:\tx线速 {round(cmd_vel[0], 2)}\t " \
        #     #       f"y线速 {round(cmd_vel[1], 2)}\t " \
        #     #       f"z角速 {round(cmd_vel[2], 2)} \n")
        #     self.last_cmd_vel = cmd_vel
        
        msg.linear.x = threshold(msg.linear.x, 
                                 dead_zone=self.xy_dead_zone, 
                                 activate_zone=self.xy_activate_zone, 
                                 clamp=self.max_vel_linear)
        msg.linear.y = threshold(msg.linear.y, 
                                 dead_zone=self.xy_dead_zone, 
                                 activate_zone=self.xy_activate_zone, 
                                 clamp=self.max_vel_linear)
        msg.angular.z = threshold(msg.angular.z, 
                                  dead_zone=self.z_dead_zone, 
                                  activate_zone=self.z_activate_zone, 
                                  clamp=self.max_vel_angular)
        
        vel = [-int(msg.linear.y * self.vel_2_0x40), 
               -int(msg.linear.x * self.vel_2_0x40), 
               int(msg.angular.z * self.omega_2_0x40)]
               
        body = self.gen_body(vel)
        self.chassis_cmd_data = self.head + [self.vel_mode & 0xff] + body + self.tail

    def clear_motion(self):
        self.chassis_cmd_data=list(self.stop_data)
        self.last_chassis_cmd_data=list(self.stop_data)
        self.car_cmd=CarCmd.HOLD

    def micro_cb(self, msg):
        with self.io_lock:
            try:
                age=(rospy.Time.now()-msg.header.stamp).to_sec()
                if not math.isfinite(age) or not -.05<=age<=.3:raise ValueError('stale micro command')
                stopped=self.car_cmd in (CarCmd.HOLD,CarCmd.ARRIVE)
                self.micro.accept(msg.session_id,msg.command_id,msg.operation,msg.vx,msg.vy,
                                  time.monotonic(),stopped and self.task,msg.operator_confirmed)
                self.micro_stop_written=False
                if not self.micro.owner:self.clear_motion();self.ser.write(self.stop_data)
            except ValueError as exc:
                # Malformed commands from the owner fail closed; other sessions
                # cannot take ownership or renew its watchdog.
                if msg.session_id==self.micro.owner:self.micro.fault(str(exc))
                rospy.logwarn_throttle(1.,'stage C base rejected: %s',exc)


    def basket_control(self, test: bool, position):
        if position == 0:
            basket_test = [0,0,0]
        elif position == 1:
            basket_test = [0,0,1]
        elif position == 2:
            basket_test = [0,1,0]
        elif position == 3:
            basket_test = [0,1,1]
        else :
            basket_test = self.last_basket_test

        if test == 1:
            basket_test[0] = 1
        elif test == 0:
            basket_test[0] = 0
            
        self.last_basket_test = basket_test
        body = self.gen_body(basket_test)
        self.basket_data = self.head + [self.basket_mode & 0xff] + body + self.tail


    def publish_micro_status(self):
        if hasattr(self,'micro_pub'):
            m=self.micro_status_type();m.header.stamp=rospy.Time.now();m.session_id=self.micro.owner
            m.command_id=self.micro.command_id;m.phase=self.micro.phase
            m.serial_stop_written=self.micro_stop_written;m.diagnostic=self.micro.diagnostic
            self.micro_pub.publish(m)

    @serialized
    def controller(self, *args):
        if self.delled:return
        with self.io_lock:
            if self.micro.owner:
                vx,vy=self.micro.tick(time.monotonic())
                if not self.task:self.micro.fault('task disabled');vx=vy=0.
                data=self.stop_data if vx==0 and vy==0 else self.head+[self.vel_mode]+self.gen_body(
                    [-int(vy*self.vel_2_0x40),-int(vx*self.vel_2_0x40),0])+self.tail
                try:
                    if self.ser.write(data)!=len(data):raise IOError('short chassis serial write')
                    self.micro_stop_written=data==self.stop_data
                except Exception as exc:
                    self.micro.fault('serial write failed: '+str(exc));self.micro_stop_written=False
                self.chassis_cmd_data=list(self.stop_data);self.last_chassis_cmd_data=list(self.stop_data)
                self.publish_micro_status()
                return
            self.publish_micro_status()
        if not self.task or self.delled:
            return
        
        if self.car_cmd == CarCmd.HOLD or self.car_cmd == CarCmd.ARRIVE:
            self.ser.write(self.stop_data)
            self.chassis_cmd_data=list(self.stop_data)
            self.last_chassis_cmd_data = self.stop_data
            return

        # if self.car_cmd == CarCmd.Get:
        #     basket_test = [0, 0, 0]
        #     body = self.gen_body(basket_test)
        #     self.basket_data = self.head + [0x01 ,0x00, 0x00] + body + self.tail 
        #     return

        if self.chassis_cmd_data != self.last_chassis_cmd_data:
            self.ser.write(self.chassis_cmd_data)
            self.last_chassis_cmd_data = self.chassis_cmd_data
    
    
if __name__ == '__main__':
    rospy.init_node("chassis_node", anonymous=True)
    
    chassis = ChassisNode()
    rospy.spin()
    
