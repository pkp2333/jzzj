#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-
import numpy as np
from scipy.spatial.transform import Rotation
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


def eulerAngles2rotationMat(roll, pitch, yaw):
    """将欧拉角(XYZ内旋顺序)转换为3×3旋转矩阵。
    旋转顺序为 Z-Y-X (即先绕X轴转roll，再绕Y轴转pitch，最后绕Z轴转yaw)，
    对应内旋(intrinsic) XYZ 顺序，等价于外旋(extrinsic) ZYX 顺序。
    Args:
        roll:  绕X轴旋转角度(弧度)
        pitch: 绕Y轴旋转角度(弧度)
        yaw:   绕Z轴旋转角度(弧度)
    Returns:
        3×3 numpy旋转矩阵 R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """
    # 绕X轴旋转矩阵(roll)
    R_x = np.array([[             1,            0,             0],
                    [             0, np.cos(roll), -np.sin(roll)],
                    [             0, np.sin(roll),  np.cos(roll)]])
    # 绕Y轴旋转矩阵(pitch)
    R_y = np.array([[ np.cos(pitch),            0, np.sin(pitch)],
                    [             0,            1,             0],
                    [-np.sin(pitch),            0, np.cos(pitch)]])
    # 绕Z轴旋转矩阵(yaw)
    R_z = np.array([[   np.cos(yaw), -np.sin(yaw),             0],
                    [   np.sin(yaw),  np.cos(yaw),             0],
                    [             0,            0,             1]])
    # 组合旋转：先X再Y最后Z，矩阵乘法顺序从右到左
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
        """初始化车辆定位节点。
        初始化流程:
            1. 加载卡尔曼滤波参数(Q过程噪声、R观测噪声)，构造滤波器
            2. 加载相机外参，计算 车→相机 齐次变换矩阵 T_car_2_cam
            3. 加载场地Tag位姿列表，预计算每个 Tag→地图 的齐次变换矩阵
            4. 订阅AprilTag检测位姿话题，创建原始/滤波位姿发布器
        """
        rospy.loginfo("car_localization_node 初始化...")

        # ---- 卡尔曼滤波参数 ----
        dt      = rospy.get_param('car_localization/filter/dt',     0.04)   # 滤波周期(秒)
        q_pos   = rospy.get_param('car_localization/filter/q_pos',   0.1)   # 位置过程噪声
        q_vel   = rospy.get_param('car_localization/filter/q_vel',   1.0)   # 速度过程噪声
        q_att   = rospy.get_param('car_localization/filter/q_att',   0.1)   # 姿态过程噪声
        r_pos   = rospy.get_param('car_localization/filter/r_pos',   0.5)   # 位置观测噪声
        r_att   = rospy.get_param('car_localization/filter/r_att',   0.5)   # 姿态观测噪声

        self.base_r_pos = r_pos  # 保存基础位置观测噪声，用于动态调整

        # 构造9×9过程噪声协方差Q: 对角分块[位置3×3, 速度3×3, 姿态3×3]
        Q = np.zeros((9,9))
        Q[0:3,0:3] = np.eye(3)*q_pos
        Q[3:6,3:6] = np.eye(3)*q_vel
        Q[6:9,6:9] = np.eye(3)*q_att
        # 构造6×6观测噪声协方差R: 对角分块[位置3×3, 姿态3×3]
        R = np.zeros((6,6))
        R[0:3,0:3] = np.eye(3)*r_pos
        R[3:6,3:6] = np.eye(3)*r_att

        # 创建卡尔曼滤波器实例
        self.kf = KalmanFilter(dt, Q, R)

        # ---- 相机外参: 计算车体→相机变换矩阵 ----
        t_cam_2_car = np.array(rospy.get_param('ascamera/translation', [0, 0, 0])).reshape((3,1))  # 相机在车体系下的平移
        rpy = rospy.get_param('ascamera/rotation', [-90 ,0, -90])                                  # 相机在车体系下的欧拉角(度)
        R_cam_2_car = eulerAngles2rotationMat(*np.radians(rpy))                                     # 欧拉角→旋转矩阵
        T_cam_2_car = np.vstack([np.hstack([R_cam_2_car, t_cam_2_car]), [0, 0, 0, 1]])             # 构造4×4齐次变换矩阵
        self.T_car_2_cam = np.linalg.inv(T_cam_2_car)                                               # 求逆得到 车→相机 变换

        # ---- 场地Tag位姿: 预计算每个Tag→地图的变换矩阵 ----
        self.id_list = rospy.get_param('car_localization/tag_pose/id_list', [0, 1, 2])  # 有效的Tag ID列表
        self.T_tag_2_map_list = []
        for tid in self.id_list:
            # 每个Tag位姿: [x, y, z, roll, pitch, yaw]，位置单位m，角度单位度
            tag_pose = rospy.get_param(f'car_localization/tag_pose/tag_{tid}', [0, 0, 0, 0, 0, 0])
            t_tag_2_map = np.array(tag_pose[:3]).reshape((3,1))                      # Tag在地图中的平移
            R_tag_2_map = eulerAngles2rotationMat(roll=np.radians(tag_pose[3]), 
                                                  pitch=np.radians(tag_pose[4]), 
                                                  yaw=np.radians(tag_pose[5]))     # Tag在地图中的旋转
            # 构造4×4齐次变换矩阵并保存
            self.T_tag_2_map_list.append(np.vstack([np.hstack([R_tag_2_map, t_tag_2_map]), [0, 0, 0, 1]]))

        # ---- 话题订阅与发布 ----
        # 订阅AprilTag检测节点输出的相机位姿(tag_id在frame_id中)
        self.cam_pose_sub = rospy.Subscriber('/cam_pose_tag', PoseWithCovarianceStamped, self.cam_pose_cb, callback_args='back', queue_size=1)        

        # 发布原始位姿(仅坐标变换，未滤波)
        self.car_pose_orig_pub = rospy.Publisher('/front_car_pose_orig_map', PoseStamped, queue_size=1)
        # 发布滤波后位姿(卡尔曼滤波平滑输出)
        self.car_pose_filt_pub = rospy.Publisher('/front_car_pose_filt_map', PoseStamped, queue_size=1)

        # 卡尔曼滤波是否已用首次观测初始化
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