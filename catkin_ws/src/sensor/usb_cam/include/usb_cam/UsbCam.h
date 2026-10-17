#pragma once


// modified from https://github.com/LarryDong/usb_cam_utils

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

#include <thread>
#include <mutex>


class UsbCam {

public:
    UsbCam(ros::NodeHandle &nh);
    ~UsbCam();
    
    ros::Subscriber task_sub;

    std::unique_ptr<camera_info_manager::CameraInfoManager> cam_info_manager;
    image_transport::Publisher camera_image_pub;
    ros::Publisher camera_info_pub;

    bool task = true;
    
    void init_node();

    void run();
    
private:
    std::mutex g_mutex;
    cv::Mat g_newest_frame;
    std::thread retrieve_thread;

    cv::VideoCapture cap;
    cv::Point img_center;
    int actual_width, actual_height;
    sensor_msgs::CameraInfo cam_info;
    
    std::string video_port;
    std::string pixel_format;
    int fps;
    int exposure;
    int width, height;
    int pub_rate;
    std::string camera_name;
    std::string camera_info_path;
    std::string kill_switch_topic;
    bool visualize;

    void task_cb(const std_msgs::Bool::ConstPtr& msg);

    void read_video_buffer();

};
