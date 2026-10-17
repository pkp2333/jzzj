#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-
import select
import sys
import termios
import time
import tty

import rospy
from geometry_msgs.msg import Twist
from user_msgs.msg import CarCmd


class TeleopKey:

    tips = """
底盘控制说明
---------------------------

  q  w  e
a    s    d  f

    space
  _________

w/s : 增加/减少纵向线速度
q/e : 增加/减少自旋角速度
a/d : 增加/减少横向线速度

f: 角速度置零直线行驶

空格 : 急停

Ctrl + c 退出
"""
    err = """
          通信失败
          """

    DE2RA = 3.1415926 / 180
    
    def __init__(self):
        rospy.loginfo(f"键盘控制节点初始化中...\n")

        self.MAX_LIN_VEL =  rospy.get_param("chassis/max_vel_linear", 0.5)  # m/s
        self.MAX_ANG_VEL = rospy.get_param("chassis/max_vel_angular", 90.0)  # °/s
        self.LIN_VEL_STEP_SIZE = rospy.get_param("chassis/key_control/step_size_linear", 0.05)  # m/s
        self.ANG_VEL_STEP_SIZE = rospy.get_param("chassis/key_control/step_size_angular", 15.0)  # rad/s


        print(TeleopKey.tips)
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = ''
        self.target_x_linear_vel = 0.0
        self.target_y_linear_vel = 0.0
        self.target_z_angular_vel = 0.0
        # to smooth control
        self.control_x_linear_vel = 0.0
        self.control_y_linear_vel = 0.0
        self.control_z_angular_vel = 0.0

        self.task = True
        self.tip_cnt = 0
        self.last_key = ''
        self.get_key = False

        self.car_cmd_pub = rospy.Publisher("/car_cmd", CarCmd, queue_size=3)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)

        self.ctrl_timer = rospy.Timer(rospy.Duration(0.05), self.jikong_key_controller)

        rospy.loginfo(f"键盘控制节点启动成功\n")


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            self.key = sys.stdin.read(1)
        else:
            self.key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def vels(self):
        return f"\n当前:\tX线速度 {round(self.target_x_linear_vel, 2)}\t " \
               f"Y线速度 {round(self.target_y_linear_vel, 2)}\t " \
               f"Z角速度 {round(self.target_z_angular_vel, 2)} \n"

    @staticmethod
    def makeSimpleProfile(in_put, out_put, slop):
        if in_put > out_put:
            out_put = min(in_put, out_put + slop)
        elif in_put < out_put:
            out_put = max(in_put, out_put - slop)
        else:
            out_put = in_put
        return out_put

    def checkXLinearVel(self, x_lin_vel_in):
        if x_lin_vel_in < -self.MAX_LIN_VEL:
            self.target_x_linear_vel = -self.MAX_LIN_VEL
        elif x_lin_vel_in > self.MAX_LIN_VEL:
            self.target_x_linear_vel = self.MAX_LIN_VEL
        else:
            self.target_x_linear_vel = x_lin_vel_in

    def checkYLinearVel(self, y_lin_vel_in):
        if y_lin_vel_in < -self.MAX_LIN_VEL:
            self.target_y_linear_vel = -self.MAX_LIN_VEL
        elif y_lin_vel_in > self.MAX_LIN_VEL:
            self.target_y_linear_vel = self.MAX_LIN_VEL
        else:
            self.target_y_linear_vel = y_lin_vel_in

    def checkAngularVel(self, ang_vel_in):
        if ang_vel_in < -self.MAX_ANG_VEL:
            self.target_z_angular_vel = -self.MAX_ANG_VEL
        elif ang_vel_in > self.MAX_ANG_VEL:
            self.target_z_angular_vel = self.MAX_ANG_VEL
        else:
            self.target_z_angular_vel = ang_vel_in

    def jikong_key_controller(self, *args):
        if not self.task:
            return
        
        self.getKey()
        car_cmd = CarCmd()
        
        cmd_vel = Twist()

        if self.key in ['w', 'W']:
            if self.target_x_linear_vel < 0:
                self.target_x_linear_vel = 0
            else:
                self.checkXLinearVel(0.25)
            self.tip_cnt += 1
            self.get_key = True
            print(self.vels())
        elif self.key in ['s', 'S']:
            if self.target_x_linear_vel > 0:
                self.target_x_linear_vel = 0
            else:
                self.checkXLinearVel(-0.25)
            self.tip_cnt += 1
            self.get_key = True
            print(self.vels())

        elif self.key in ['a', 'A']:
            if self.target_y_linear_vel < 0:
                self.target_y_linear_vel = 0
            else:
                self.checkYLinearVel(0.25)
            self.tip_cnt += 1
            self.get_key = True
            print(self.vels())
        elif self.key in ['d', 'D']:
            if self.target_y_linear_vel > 0:
                self.target_y_linear_vel = 0
            else:
                self.checkYLinearVel(-0.25)
            self.tip_cnt += 1
            self.get_key = True
            print(self.vels())

        elif self.key in ['q', 'Q']:
            if self.control_x_linear_vel >= 0:
                if self.target_z_angular_vel >= 0:
                    self.checkAngularVel(60.5)
                else:
                    self.target_z_angular_vel = 0
            else:
                if self.target_z_angular_vel <= 0:
                    self.checkAngularVel(0.75)
                else:
                    self.target_z_angular_vel = 0
            self.tip_cnt += 1
            self.get_key = True
            print(self.vels())
        elif self.key in ['e', 'E']:
            if self.control_x_linear_vel >= 0:
                if self.target_z_angular_vel <= 0:
                    self.checkAngularVel(-60.5)
                else:
                    self.target_z_angular_vel = 0
            else:
                if self.target_z_angular_vel >= 0:
                    self.checkAngularVel(-0.75)
                else:
                    self.target_z_angular_vel = 0
            self.tip_cnt += 1
            self.get_key = True
            print(self.vels())

        elif self.key in ['f', 'F']:
            self.target_z_angular_vel = 0.0
            self.control_z_angular_vel = 0.0
            self.get_key = True
            print(self.vels())

        elif self.key == ' ':
            self.target_x_linear_vel = 0.0
            self.target_y_linear_vel = 0.0
            self.target_z_angular_vel = 0.0
            self.control_x_linear_vel = 0.0
            self.control_y_linear_vel = 0.0
            self.control_z_angular_vel = 0.0
            self.get_key = True
            print(self.vels())
            print("\n\033[31m急停\033[0m\n")

        elif self.key == '\x03':
            print("\n\033[31m退出\033[0m\n")
            self.target_x_linear_vel = 0.0
            self.target_y_linear_vel = 0.0
            self.target_z_angular_vel = 0.0
            self.control_x_linear_vel = 0.0
            self.control_y_linear_vel = 0.0
            self.control_z_angular_vel = 0.0
            self.get_key = True
            time.sleep(.5)
            self.task = False

        if self.get_key:
            self.last_key = self.key
            self.get_key = 0

        if self.tip_cnt == 20:
            print(TeleopKey.tips)
            self.tip_cnt = 0

        self.control_x_linear_vel = self.makeSimpleProfile(self.target_x_linear_vel, self.control_x_linear_vel, 
                                                           self.LIN_VEL_STEP_SIZE)
        cmd_vel.linear.x  = self.control_x_linear_vel

        self.control_y_linear_vel = self.makeSimpleProfile(self.target_y_linear_vel, self.control_y_linear_vel, 
                                                           self.LIN_VEL_STEP_SIZE)
        cmd_vel.linear.y = self.control_y_linear_vel

        self.control_z_angular_vel = self.makeSimpleProfile(self.target_z_angular_vel, self.control_z_angular_vel, 
                                                            self.ANG_VEL_STEP_SIZE)
        cmd_vel.angular.z = self.control_z_angular_vel * TeleopKey.DE2RA

        if cmd_vel.linear.x or cmd_vel.linear.y or cmd_vel.angular.z:
            car_cmd.cmd = CarCmd.REMOTE
        else:
            car_cmd.cmd = CarCmd.HOLD
        self.car_cmd_pub.publish(car_cmd)
        self.cmd_vel_pub.publish(cmd_vel)


def main():
    try:
        rospy.init_node('rs_key_control_node', anonymous=True)
    except rospy.ROSInterruptException:
        rospy.logwarn("键盘控制节点启动失败...")
    else:
        car_key_control_node = TeleopKey()
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            car_key_control_node.__del__()


if __name__ == "__main__":
    main()
