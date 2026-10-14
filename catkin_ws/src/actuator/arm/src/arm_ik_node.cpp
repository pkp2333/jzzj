#include <iostream>

#include "stdio.h"
#include "stdlib.h"
#include <unistd.h>
#include "sys/types.h"
#include <sys/stat.h>
#include "string.h"
#include <arpa/inet.h>
#include <sys/un.h>
#include "arm/arm_kinematics.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


using namespace KDL;


// 弧度转角度
const float RA2DE = 180.0f / M_PI;
// 角度转弧度
const float DE2RA = M_PI / 180.0f;

geometry_msgs::PoseStamped arm_target_pose;
geometry_msgs::PoseStamped last_target_pose;

/*
 * 这是求机械臂反解的代码,不涉及夹爪.
 * 注意:反解求出的关节角度只是数据,故可能会出现越界的情况.
 */


float clip(float num, float lower, float upper, float offset_tolerance=30) {
    if (num < lower-offset_tolerance) {
        num += 360*int(abs(num)/360+1);
    }
    else if (num > upper+offset_tolerance) {
        num -= 360*int(abs(num)/360+1);
    }

    if (num < lower-offset_tolerance) {
        num = lower;
    }
    else if (num > upper+offset_tolerance) {
        num = upper;
    }
    return num;
}


void target_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    arm_target_pose = *msg;
}


int main(int argc, char **argv) {
    setlocale(LC_ALL, "en_US.UTF-8");

    ros::init(argc, argv, "arm_ik_server");
    ros::NodeHandle nh;

    ROS_INFO("机械臂运动学反解节点初始化中...\n");
    // 创建机械臂实例
    MyArm arm = MyArm();
    std::string urdf_file = "/home/orangepi/catkin_ws/src/actuator/arm/urdf/arm.urdf";

    ros::Subscriber arm_target_sub = nh.subscribe<geometry_msgs::PoseStamped>("arm_target", 3, target_pose_cb);
    ros::Publisher arm_servo_degree_pub = nh.advertise<sensor_msgs::JointState>("arm_ik_servo_degree", 1);

    ROS_INFO("机械臂运动学反解节点成功初始化\n");

    ros::Rate rate(10.0);

    sensor_msgs::JointState arm_degree;
    arm_degree.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    arm_degree.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    while (ros::ok()) {
        ros::spinOnce();

        /*
        * 这是求机械臂反解的代码,不涉及夹爪.
        * 注意:反解求出的关节角度只是数据,故可能会出现越界的情况.
        */ 
        if (arm_target_pose.header.seq > last_target_pose.header.seq) {
            double x = arm_target_pose.pose.position.x;
            double y = arm_target_pose.pose.position.y;
            double z = arm_target_pose.pose.position.z;
            double roll, pitch, yaw;
            tf2::Quaternion quat(
                arm_target_pose.pose.orientation.x,
                arm_target_pose.pose.orientation.y,
                arm_target_pose.pose.orientation.z,
                arm_target_pose.pose.orientation.w
            );
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            // 末端位置(单位: m)
            double xyz[]{x, y, z};
            // 末端姿态(单位: rad)
            double rpy[]{roll, pitch, yaw};

            ROS_INFO("第 %d 个末端期望位姿 @ x:%0.1f, y:%0.1f, z:%0.1f, R:%.1f, P:%.1f, Y:%.1f", 
                     arm_target_pose.header.seq,
                     xyz[0]*100.0, xyz[1]*100.0, xyz[2]*100.0,
                     rpy[0]*RA2DE, rpy[1]*RA2DE, rpy[2]*RA2DE);

            std::vector<double> outjoints;
            std::vector<double> targetXYZ;
            std::vector<double> targetRPY;
            for (int i = 0; i < 3; i++) 
                targetXYZ.push_back(xyz[i]);
            for (int i = 0; i < 3; i++) 
                targetRPY.push_back(rpy[i]);

            // 反解求到达目标点的各关节角度
            arm.arm_getIK(urdf_file, targetXYZ, targetRPY, outjoints);
            
            std::cout << "\033[1m\33[32m====================机械臂运动学反解结果:====================\33[0m\n";
            std::cout << std::fixed << std::setprecision(1);
            // double data_f_original[] = {0};
            for (int i = 0; i < 5; i++) {
                arm_degree.position[i] = outjoints.at(i) * RA2DE;
                std::cout << "\033[1m\33[34m机械臂 " << i+1 << " 号舵机\33[0m " << outjoints.at(i) * RA2DE << " °\n";
            };
            std::cout << "\n\n\n\n";

            arm_degree.position[5] = 50;
            arm_servo_degree_pub.publish(arm_degree);
            last_target_pose = arm_target_pose;
        }
    }
    ROS_INFO("Arm Server Successfully Terminated");
    return 0;
}
