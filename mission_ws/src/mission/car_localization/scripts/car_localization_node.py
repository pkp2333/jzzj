#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-
import numpy as np
from scipy.spatial.transform import Rotation
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


def eulerAngles2rotationMat(roll, pitch, yaw):
    R_x = np.array([[             1,            0,             0],
                    [             0, np.cos(roll), -np.sin(roll)],
                    [             0, np.sin(roll),  np.cos(roll)]])
    
    R_y = np.array([[ np.cos(pitch),            0, np.sin(pitch)],
                    [             0,            1,             0],
                    [-np.sin(pitch),            0, np.cos(pitch)]])
    
    R_z = np.array([[   np.cos(yaw), -np.sin(yaw),             0],
                    [   np.sin(yaw),  np.cos(yaw),             0],
                    [             0,            0,             1]])
    return R_z @ R_y @ R_x


class KalmanFilter:
    def __init__(self, dt, Q, R):
        # 状态：pos(3), vel(3), euler(3)
        # 观测：pos(3), euler(3)
        self.dt = dt
        self.dim_x = 9
        self.dim_z = 6

        F = np.eye(self.dim_x)
        for i in range(3):
            F[i, i+3] = dt 
        self.F = F

        H = np.zeros((self.dim_z, self.dim_x))
        H[0:3, 0:3] = np.eye(3)
        H[3:6, 6:9] = np.eye(3)
        self.H = H

        self.Q = Q.copy()
        self.R = R.copy()
        self.P = np.eye(self.dim_x)
        self.x = np.zeros((self.dim_x,))

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        z[3:6] = np.array([((ang + np.pi) % (2*np.pi) - np.pi) for ang in z[3:6]])

        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        y = z - (self.H @ self.x)

        y[3:6] = np.array([((ang + np.pi) % (2*np.pi) - np.pi) for ang in y[3:6]])

        self.x = self.x + K @ y
        I = np.eye(self.dim_x)
        self.P = (I - K @ self.H) @ self.P

        self.x[6:9] = np.array([((ang + np.pi) % (2*np.pi) - np.pi) for ang in self.x[6:9]])


class CarLocalization:
    def __init__(self):
        rospy.loginfo("car_localization_node 初始化...")

        dt      = rospy.get_param('car_localization/filter/dt',     0.04)
        q_pos   = rospy.get_param('car_localization/filter/q_pos',   0.1)
        q_vel   = rospy.get_param('car_localization/filter/q_vel',   1.0)
        q_att   = rospy.get_param('car_localization/filter/q_att',   0.1)
        r_pos   = rospy.get_param('car_localization/filter/r_pos',   0.5)
        r_att   = rospy.get_param('car_localization/filter/r_att',   0.5)

        self.base_r_pos = r_pos

        Q = np.zeros((9,9))
        Q[0:3,0:3] = np.eye(3)*q_pos
        Q[3:6,3:6] = np.eye(3)*q_vel
        Q[6:9,6:9] = np.eye(3)*q_att
        R = np.zeros((6,6))
        R[0:3,0:3] = np.eye(3)*r_pos
        R[3:6,3:6] = np.eye(3)*r_att

        self.kf = KalmanFilter(dt, Q, R)

        t_cam_2_car = np.array(rospy.get_param('ascamera/translation', [0, 0, 0])).reshape((3,1))
        rpy = rospy.get_param('ascamera/rotation', [-90 ,0, -90])
        R_cam_2_car = eulerAngles2rotationMat(*np.radians(rpy))
        T_cam_2_car = np.vstack([np.hstack([R_cam_2_car, t_cam_2_car]), [0, 0, 0, 1]])
        self.T_car_2_cam = np.linalg.inv(T_cam_2_car)

        self.id_list = rospy.get_param('car_localization/tag_pose/id_list', [0, 1, 2])
        self.T_tag_2_map_list = []
        for tid in self.id_list:
            tag_pose = rospy.get_param(f'car_localization/tag_pose/tag_{tid}', [0, 0, 0, 0, 0, 0])
            t_tag_2_map = np.array(tag_pose[:3]).reshape((3,1))
            R_tag_2_map = eulerAngles2rotationMat(roll=np.radians(tag_pose[3]), 
                                                  pitch=np.radians(tag_pose[4]), 
                                                  yaw=np.radians(tag_pose[5]))
            self.T_tag_2_map_list.append(np.vstack([np.hstack([R_tag_2_map, t_tag_2_map]), [0, 0, 0, 1]]))

        self.cam_pose_sub = rospy.Subscriber('/cam_pose_tag', PoseWithCovarianceStamped, self.cam_pose_cb, callback_args='back', queue_size=1)        

        self.car_pose_orig_pub = rospy.Publisher('/front_car_pose_orig_map', PoseStamped, queue_size=1)
        self.car_pose_filt_pub = rospy.Publisher('/front_car_pose_filt_map', PoseStamped, queue_size=1)

        self.kf_initialized = False

    def cam_pose_cb(self, msg: PoseWithCovarianceStamped, camera_type):
        tag_id = int(msg.header.frame_id.split("_")[1])
        if not tag_id in self.id_list:
            rospy.logwarn(f"{tag_id} not in tag lists")
            return
        
        tag_index = self.id_list.index(tag_id)

        p_cam_2_tag = msg.pose.pose.position
        q_cam_2_tag = msg.pose.pose.orientation
        R_cam_2_tag = Rotation.from_quat([q_cam_2_tag.x, q_cam_2_tag.y, q_cam_2_tag.z, q_cam_2_tag.w]).as_matrix()
        T_cam_2_tag = np.vstack([np.hstack([R_cam_2_tag, [[p_cam_2_tag.x], [p_cam_2_tag.y], [p_cam_2_tag.z]]]), [0, 0, 0, 1]])
        T_car_2_tag = T_cam_2_tag @ self.T_car_2_cam
        T_car_2_map = self.T_tag_2_map_list[tag_index] @ T_car_2_tag

        pose_car_2_map_orig = T_car_2_map[:3,3]
        pose_car_2_map_orig[2] = 0
        euler_car_2_map_orig = Rotation.from_matrix(T_car_2_map[:3,:3]).as_euler('xyz')

        pose_orig = PoseStamped()
        pose_orig.header.stamp = msg.header.stamp
        pose_orig.header.frame_id = "map"
        pose_orig.pose.position.x = pose_car_2_map_orig[0]
        pose_orig.pose.position.y = pose_car_2_map_orig[1]
        pose_orig.pose.position.z = pose_car_2_map_orig[2]
        q_orig = Rotation.from_matrix(T_car_2_map[:3, :3]).as_quat()
        pose_orig.pose.orientation.x = q_orig[0]
        pose_orig.pose.orientation.y = q_orig[1]
        pose_orig.pose.orientation.z = q_orig[2]
        pose_orig.pose.orientation.w = q_orig[3]

        self.car_pose_orig_pub.publish(pose_orig)

        z = np.hstack([pose_car_2_map_orig, euler_car_2_map_orig])

        if not self.kf_initialized:
            self.kf.x[0:3] = pose_car_2_map_orig
            self.kf.x[6:9] = euler_car_2_map_orig
            self.kf_initialized = True
        else:
            self.kf.predict()
            self.kf.update(z)
        
        pose_filt = PoseStamped()
        pose_filt.header.stamp = msg.header.stamp
        pose_filt.header.frame_id = "map"
        pose_filt.pose.position.x = self.kf.x[0]
        pose_filt.pose.position.y = self.kf.x[1]
        pose_filt.pose.position.z = self.kf.x[2]
        q_filt = Rotation.from_euler('xyz', self.kf.x[6:9]).as_quat()
        pose_filt.pose.orientation.x = q_filt[0]
        pose_filt.pose.orientation.y = q_filt[1]
        pose_filt.pose.orientation.z = q_filt[2]
        pose_filt.pose.orientation.w = q_filt[3]

        self.car_pose_filt_pub.publish(pose_filt)


if __name__ == '__main__':
    rospy.init_node('car_localization_node')
    CarLocalization()
    rospy.spin()
