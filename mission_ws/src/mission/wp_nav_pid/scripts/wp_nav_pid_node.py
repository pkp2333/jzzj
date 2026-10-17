#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-
import time
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from pid import PID_incremental

import rospy
from std_msgs.msg import Bool
from user_msgs.msg import CarCmd, PID
from geometry_msgs.msg import Twist, PoseStamped


class WaypointNavigationPIDNode(object):
    def __init__(self) -> None:
        
        rospy.loginfo(f"Waypoint Navigation Initializing...\n")

        self.kp_x = rospy.get_param('wp_nav/pid_x/kp', 0)
        self.ki_x = rospy.get_param('wp_nav/pid_x/ki', 0)
        self.kd_x = rospy.get_param('wp_nav/pid_x/kd', 0)
        i_a_x = rospy.get_param('wp_nav/pid_x/i_a', 0)
        o_t_x = rospy.get_param('wp_navpid_x//o_t', 0.2)
        self.kp_y = rospy.get_param('wp_nav/pid_y/kp', 0)
        self.ki_y = rospy.get_param('wp_nav/pid_y/ki', 0)
        self.kd_y = rospy.get_param('wp_nav/pid_y/kd', 0)
        i_a_y = rospy.get_param('wp_nav/pid_y/i_a', 0)
        o_t_y = rospy.get_param('wp_nav/pid_y/o_t', 0.2)
        self.kp_yaw = rospy.get_param('wp_nav/pid_yaw/kp', 0)
        self.ki_yaw = rospy.get_param('wp_nav/pid_yaw/ki', 0)
        self.kd_yaw = rospy.get_param('wp_nav/pid_yaw/kd', 0)
        i_a_yaw = rospy.get_param('wp_nav/pid_yaw/i_a', 0)
        o_t_yaw = rospy.get_param('wp_nav/pid_yaw/o_t', 0.2)
        
        self.end_l = rospy.get_param('wp_nav/end_l', 0.1)
        self.end_a = rospy.get_param('wp_nav/end_a', 5)

        self.DE2RA = np.pi / 180

        self.task = True
        self.cam_orientation = True # 设定选用摄像头，前面为True
        self.get_car_pose_front = False # 前方摄像头丢失标志位
        self.get_car_pose_back = False  # 后方摄像头丢失标志位
        self.car_pose_front = PoseStamped()
        self.car_pose_front.pose.orientation.w = 1
        self.car_pose_back = PoseStamped()
        self.car_pose_back.pose.orientation.w = 1
        self.get_target = False
        self.now_target_id = 1
        self.last_target_id = -1
        self.car_target = PoseStamped()
        self.car_target.pose.orientation.w = 1
        self.car_target_xyz = [0, 0, 0]
        self.car_target_rpy = [0, 0, 0]
        self.last_yaw = 0
        self.last_spin_dir = 1
        # 找不到二维码停车标志位
        self.stop_flag = 0
        self.stop_flag_1 = 0

        self.wait_for_acquire = False
        self.loc_lost_time = time.time()
        self.suspend = False
        self.arrive_cnt = 0
        
        self.car_cmd = CarCmd()
        self.car_cmd.cmd = CarCmd.HOLD
        self.cmd_vel = Twist()
        self.zero_vel = Twist()

        self.car_pid_vertical = PID_incremental(p=self.kp_x, i=self.ki_x, d=self.kd_x, i_a=i_a_x, o_t=o_t_x)
        self.car_pid_horizontal = PID_incremental(p=self.kp_y, i=self.ki_y, d=self.kd_y, i_a=i_a_y, o_t=o_t_y)
        self.car_pid_yaw = PID_incremental(p=self.kp_yaw, i=self.ki_yaw, d=self.kd_yaw, i_a=i_a_yaw, o_t=o_t_yaw)

        self.car_pid_x_sub = rospy.Subscriber('wp_nav_pid/pid_x', PID, self.car_pid_x_cb)
        self.car_pid_y_sub = rospy.Subscriber('wp_nav_pid/pid_y', PID, self.car_pid_y_cb)
        self.car_pid_yaw_sub = rospy.Subscriber('wp_nav_pid/pid_yaw', PID, self.car_pid_yaw_cb)

        self.tag_det_sub_front = rospy.Subscriber('apriltag_detected', Bool, self.tag_det_cb_front)
        self.tag_det_sub_back = rospy.Subscriber('back_apriltag_detected', Bool, self.tag_det_cb_back)
        self.cam_orientation_sub = rospy.Subscriber('orientation_bool', Bool, self.cam_orie_cb)
        self.front_car_pose_sub = rospy.Subscriber('front_car_pose_orig_map', PoseStamped, self.front_car_pose_cb)
        self.back_car_pose_sub = rospy.Subscriber('back_car_pose_filt_map', PoseStamped, self.back_car_pose_cb)
        self.car_target_sub = rospy.Subscriber('car_target', PoseStamped, self.car_target_cb)
        
        self.car_cmd_pub = rospy.Publisher('car_cmd', CarCmd, queue_size=3)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=3)

        time.sleep(5)
        self.car_nav_timer = rospy.Timer(rospy.Duration(0.05), self.car_navigator)

        rospy.loginfo(f"Waypoint Navigation Successfully Initialized!\n")

    def __del__(self):
        print('\nWaypoint Navigation Terminated\n')

    def cam_orie_cb(self, msg:Bool):
        self.cam_orientation = msg.data

    def tag_det_cb_front(self, msg:Bool):
        self.get_car_pose_front = msg.data

    def tag_det_cb_back(self, msg:Bool):
        self.get_car_pose_back = msg.data

    def front_car_pose_cb(self, msg:PoseStamped):
        self.car_pose_front = msg
        self.get_car_pose_front = True

    def back_car_pose_cb(self, msg:PoseStamped):
        self.car_pose_back = msg
        self.get_car_pose_back = True

    def car_pid_x_cb(self, msg:PID):
        self.kp_x = msg.p
        self.ki_x = msg.i
        self.kd_x = msg.d
        self.car_pid_vertical.update_param(p=msg.p, i=msg.i, d=msg.d, i_a=msg.i_activate, o_t=msg.out_thresh)
    
    def car_pid_y_cb(self, msg:PID):
        self.kp_y = msg.p
        self.ki_y = msg.i
        self.kd_y = msg.d
        self.car_pid_horizontal.update_param(p=msg.p, i=msg.i, d=msg.d, i_a=msg.i_activate, o_t=msg.out_thresh)
        
    def car_pid_yaw_cb(self, msg:PID):
        self.kp_yaw = msg.p
        self.ki_yaw = msg.i
        self.kd_yaw = msg.d
        self.car_pid_yaw.update_param(p=msg.p, i=msg.i, d=msg.d, i_a=msg.i_activate, o_t=msg.out_thresh)        

    def car_target_cb(self, msg:PoseStamped):
        if msg.header.seq > self.last_target_id:
            self.now_target_id = msg.header.seq
            self.car_pid_vertical.reset()
            self.car_pid_horizontal.reset()
            self.car_pid_yaw.reset()
            self.car_target_xyz = [msg.pose.position.x, 
                                   msg.pose.position.y, 
                                   msg.pose.position.z]
            rpy =  Rotation.from_quat([msg.pose.orientation.x, 
                                       msg.pose.orientation.y, 
                                       msg.pose.orientation.z, 
                                       msg.pose.orientation.w]).as_euler('xyz', degrees=True)
            self.car_target_rpy = [rpy[0], 
                                   rpy[1], 
                                   rpy[2]]
            self.get_target = True
            rospy.loginfo(f"777777777777777777777777777777777\n")
            rospy.logwarn("航点导航获得第%d个航点 @ x:%.2f  y:%.2f  yaw:%.2f\n\n", self.now_target_id, self.car_target_xyz[0]*100, self.car_target_xyz[1]*100, self.car_target_rpy[2])
        else:
            self.get_target = False

    def car_navigator(self, *args):
        if not self.task:
            return
        
        canvas = np.zeros((400, 480, 3), dtype=np.uint8)

        # 未获取自身位姿 当两个摄像头均丢失位姿信息时停车
        if not (self.get_car_pose_front or self.get_car_pose_back):
            if not self.wait_for_acquire:
                self.loc_lost_time = time.time()
                self.wait_for_acquire = True
            if time.time() - self.loc_lost_time > 1 and self.stop_flag == 0:
                self.cmd_vel.linear.x = 0 #-0.05
                self.cmd_vel.linear.y = 0
                # self.cmd_vel.angular.z = self.last_spin_dir * 1 * self.DE2RA
                self.cmd_vel.angular.z = 0
                self.car_cmd.cmd = self.car_cmd.HOLD
                self.stop_flag = 1
                self.car_cmd_pub.publish(self.car_cmd)
                self.cmd_vel_pub.publish(self.cmd_vel)
                rospy.loginfo(f"video_lost\n")
            
            cv2.putText(canvas, "Wait for self Location",
                        (10, 30), 3, 0.8, (0, 0, 255), 2)
            
            cv2.imshow('nav_info', canvas)
            key = cv2.waitKey(1) % 0xff
            if key == 27:
                cv2.destroyAllWindows()
                self.car_cmd.cmd = self.car_cmd.HOLD
                self.car_cmd_pub.publish(self.car_cmd)
                self.task = False
            elif key == 32:
                self.car_cmd.cmd = self.car_cmd.HOLD
                self.car_cmd_pub.publish(self.car_cmd)
                self.suspend = True
            return
        
        self.stop_flag = 1
        self.wait_for_acquire = False

        # 选择采用哪个摄像头数据，当选用摄像头位姿丢失时采用另一个摄像头
        if(self.cam_orientation):

            if(self.get_car_pose_front):
                # rospy.loginfo(f"front_video_frame\n")
                quat = self.car_pose_front.pose.orientation
                rpy = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz', degrees=True)
                cur = [self.car_pose_front.pose.position.x,
                       self.car_pose_front.pose.position.y,
                       rpy[2]]
            else:
                # rospy.loginfo(f"frontlsotback_video_frame\n")
                quat = self.car_pose_back.pose.orientation
                rpy = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz', degrees=True)
                cur = [self.car_pose_back.pose.position.x,
                       self.car_pose_back.pose.position.y,
                       rpy[2]]    
        else:
            if(self.get_car_pose_back):
                rospy.loginfo(f"back_video_frame\n")
                quat = self.car_pose_back.pose.orientation
                rpy = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz', degrees=True)
                cur = [self.car_pose_back.pose.position.x,
                       self.car_pose_back.pose.position.y,
                       rpy[2]]
            else:
                rospy.loginfo(f"backlost,front_video_frame\n")
                quat = self.car_pose_front.pose.orientation
                rpy = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz', degrees=True)
                cur = [self.car_pose_front.pose.position.x,
                       self.car_pose_front.pose.position.y,
                       rpy[2]]   
                
        cur_yaw_RA = rpy[2] * self.DE2RA

        cv2.putText(canvas, "X p:%.2f i:%.2f d:%.2f" 
                    % (self.kp_x, self.ki_x, self.kd_x),
                    (10, 60), 3, 0.8, (255, 120, 0), 2)
        cv2.putText(canvas, "Y p:%.2f i:%.2f d:%.2f" 
                    % (self.kp_y, self.ki_y, self.kd_y),
                    (10, 85), 3, 0.8, (255, 120, 0), 2)
        cv2.putText(canvas, "W p:%.2f i:%.2f d:%.2f" 
                    % (self.kp_yaw, self.ki_yaw, self.kd_yaw),
                    (10, 110), 3, 0.8, (255, 120, 0), 2)
        
        cv2.putText(canvas, "Cur x:%.1f y:%.1f a:%.1f" 
                    % (cur[0] * 100, cur[1] * 100, cur[2]),
                    (10, 170), 3, 0.8, (255, 120, 0), 2)
        
        if(not self.get_target):
            if(self.stop_flag_1==0):
                self.stop_flag_1 = 1
                self.car_cmd.cmd = self.car_cmd.HOLD
                self.car_cmd_pub.publish(self.car_cmd)
                rospy.loginfo(f"111111111111111111111111111111111\n")
            cv2.putText(canvas, "Wait for Target",
                        (10, 30), 3, 0.8, (0, 0, 255), 2)
            cv2.imshow('nav_info', canvas)
            key = cv2.waitKey(1) % 0xff
            if key == 27:
                cv2.destroyAllWindows()
                self.car_cmd.cmd = self.car_cmd.HOLD
                self.car_cmd_pub.publish(self.car_cmd)
                self.task = False
            elif key == 32:
                self.car_cmd.cmd = self.car_cmd.HOLD
                self.car_cmd_pub.publish(self.car_cmd)
                self.suspend = True
            return
        else:
            self.stop_flag_1 = 0

            if self.suspend:
                self.car_cmd.cmd = self.car_cmd.HOLD
                self.car_cmd_pub.publish(self.car_cmd)
                rospy.loginfo(f"2222222222222222222222222222222\n")

                cv2.putText(canvas, "Nav Suspend",
                            (10, 30), 3, 0.8, (0, 0, 255), 2)
                
                cv2.imshow('nav_info', canvas)
                key = cv2.waitKey(1) % 0xff
                if key == 27:
                    cv2.destroyAllWindows()
                    self.car_cmd.cmd = self.car_cmd.HOLD
                    self.car_cmd_pub.publish(self.car_cmd)
                    self.task = False
                elif key == 32:
                    self.car_cmd.cmd = self.car_cmd.HOLD
                    self.car_cmd_pub.publish(self.car_cmd)
                    self.suspend = False
                return
            
            cv2.putText(canvas, "Nav On",
                        (10, 30), 3, 0.8, (0, 255, 0), 2)    
            cv2.putText(canvas, "Tar-%d x:%.1f y:%.1f a:%.1f" 
                        % (self.now_target_id,
                        self.car_target_xyz[0] * 100, 
                        self.car_target_xyz[1] * 100, 
                        self.car_target_rpy[2]),
                        (10, 140), 3, 0.8, (255, 120, 0), 2)
            
            err_yaw = self.car_target_rpy[2] - cur[2]
            err_yaw = 180 - (err_yaw + 180) % 360

            vel_x_world = self.car_pid_vertical.update(target=self.car_target_xyz[0], now=cur[0])
            vel_y_world = self.car_pid_horizontal.update(target=self.car_target_xyz[1], now=cur[1])
            vel_z_world = self.car_pid_yaw.update(target=0, now=err_yaw)

            cv2.putText(canvas, "err_x:%.1f err_y:%.1f err_a:%.1f" % (self.car_pid_vertical.err*100, self.car_pid_horizontal.err*100, self.car_pid_yaw.err),
                        (10, 260), 3, 0.8, (255, 120, 0), 2)
            
            # 退出条件
            if abs(self.car_pid_vertical.err) < self.end_l and abs(self.car_pid_horizontal.err) < self.end_l and abs(self.car_pid_yaw.err) < self.end_a:
            # if abs(self.car_pid_yaw.err) < self.end_a:
                self.arrive_cnt += 1
                rospy.logwarn(f"航点 {self.now_target_id} 到位 {self.arrive_cnt}\n")
                if self.arrive_cnt >= 4:
                    self.car_pid_vertical.reset()
                    self.car_pid_horizontal.reset()
                    self.car_pid_yaw.reset()
                    self.cmd_vel_pub.publish(self.zero_vel)
                    time.sleep(.1)
                    self.get_target = False
                    self.last_target_id = self.now_target_id
                    self.car_cmd.cmd = self.car_cmd.ARRIVE
                    self.car_cmd_pub.publish(self.car_cmd)
                    time.sleep(.1)
                    self.arrive_cnt = 0
                return
            else:
                self.arrive_cnt = 0
            
            vel_body_x = (vel_x_world * np.cos(cur_yaw_RA) + vel_y_world * np.sin(cur_yaw_RA))  # x fb
            vel_body_y = (-vel_x_world * np.sin(cur_yaw_RA) + vel_y_world * np.cos(cur_yaw_RA)) # y lr
            vel_body_z = vel_z_world                                                            # w rl
            cv2.putText(canvas, 'w  x:%.1f y:%.1f z:%.1f' % (vel_x_world*100, vel_y_world*100, vel_z_world),
                        (10, 305), 3, 0.6, (255, 120, 0), 2)
            cv2.putText(canvas, 'b  x:%.1f y:%.1f z:%.1f' % (vel_body_x*100, vel_body_y*100, vel_body_z),
                        (10, 330), 3, 0.6, (255, 120, 0), 2)
            
            self.cmd_vel.linear.x = vel_body_x
            self.cmd_vel.linear.y = vel_body_y
            self.cmd_vel.angular.z = vel_body_z * self.DE2RA
            if vel_body_z > 0:
                self.last_spin_dir = 1 
            elif vel_body_z < 0:
                self.last_spin_dir = -1
            else:
                self.last_spin_dir = 0 
            self.car_cmd.cmd = self.car_cmd.NAVIGATION
            self.car_cmd_pub.publish(self.car_cmd)
            self.cmd_vel_pub.publish(self.cmd_vel)
            
            cv2.imshow('nav_info', canvas)
            key = cv2.waitKey(1) % 0xff
            if key == 27:
                cv2.destroyAllWindows()
                self.car_cmd.cmd = self.car_cmd.HOLD
                self.car_cmd_pub.publish(self.car_cmd)
                self.task = False
            elif key == 32:
                self.car_cmd.cmd = self.car_cmd.HOLD
                self.car_cmd_pub.publish(self.car_cmd)
                self.suspend = True



if __name__ == "__main__":
    rospy.init_node('wp_nav_pid_node')

    wp_nav_pid_node = WaypointNavigationPIDNode()
    rospy.spin()
    
