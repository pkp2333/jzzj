#include "usb_cam/UsbCam.h"


UsbCam::UsbCam(ros::NodeHandle &nh) {
    nh.param<std::string>("/usb_cam/video_port", video_port, "0");           // get videoX, X is defined in roslaunch file.
    nh.param<std::string>("/usb_cam/camera_name", camera_name, "usb_cam");   // set pub ns & frame_id 
    nh.param<std::string>("/usb_cam/pixel_format", pixel_format, "yuyv");    // set pixel_format (yuyv / mjpeg).
    nh.param<int>("/usb_cam/fps", fps, 30);                                  // set FPS.
    nh.param<int>("/usb_cam/exposure", exposure, 0);                         // set exposure time. unit: ms.
    nh.param<int>("/usb_cam/width", width, 640);                             // set frame width. unit: pix.
    nh.param<int>("/usb_cam/height", height, 480);                           // set frame height. unit: pix.
    nh.param<int>("/usb_cam/pub_rate", pub_rate, 30);                        // set pub rate. unit: Hz.
    nh.param<std::string>("/usb_cam/camera_info_path", camera_info_path, "file:///home/jetson/.ros/camera_info/");
    nh.param<std::string>("/usb_cam/kill_switch_topic", kill_switch_topic, "/task");
    nh.param<bool>("/usb_cam/visualize", visualize, false);                  // show image or not.

    cam_info_manager = std::make_unique<camera_info_manager::CameraInfoManager>(nh, camera_name, camera_info_path + camera_name + "_calib.yaml");
    cam_info = cam_info_manager->getCameraInfo();

    task_sub = nh.subscribe(kill_switch_topic, 1, &UsbCam::task_cb, this);

    image_transport::ImageTransport it(nh);
    camera_image_pub = it.advertise("/"+camera_name+"/image_raw", 1);
    camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/"+camera_name+"/camera_info", 3);
}


UsbCam::~UsbCam()  {
    task = false;
    cv::destroyAllWindows();
    if (retrieve_thread.joinable()) {
        retrieve_thread.join();
    }
    ROS_INFO("\33[32mThread Joined\33[0m");
    cap.release();
    ROS_INFO("\33[32mCAP Released\33[0m");
    ROS_WARN("\33[32mUSB Cam Node Terminated\33[0m");
}



void UsbCam::init_node() {
    if (video_port.size() == 1 && 48 <= video_port[0] <= 57) {
        ROS_INFO_STREAM("\33[32mOpening camera from: \33[37m/dev/video" << video_port << "\33[0m\n");
        cap.open(video_port[0]-48, cv::CAP_V4L2);
    }
    else {
        ROS_INFO_STREAM("\33[32mOpening camera from: \33[37m" << video_port << "\33[0m\n");
        cap.open(video_port, cv::CAP_V4L2);
    }
    
    
    while (!cap.isOpened()) {
        ROS_WARN("Cannot open camera...");
        sleep(1);
    }
    
    ROS_INFO("\33[32mCamera opened. Setting camera properties...\33[0m\n");
    
    if (pixel_format == "mjpeg" or pixel_format == "MJPEG") {
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        ROS_INFO_STREAM("Set pixel_format: \033[32m" << pixel_format << "\033[0m\n");
    }
    else if (pixel_format == "yuyv" or pixel_format == "YUYV") {
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
        ROS_INFO_STREAM("Set pixel_format: \033[32m" << pixel_format << "\033[0m\n");
    }
    else {
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    }
    
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, fps);

    if (exposure == 0) {
        ROS_INFO("\033[33mUsing auto-exposure\033[0m\n");
        cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);  // 0-2.6 manual exposure  2.6-4 auto exposure
    }
    else {
        ROS_INFO_STREAM("\033[33mSet exposure: " << exposure << " | 10000\033[0m");
        cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
        cap.set(cv::CAP_PROP_EXPOSURE, exposure);
    }
    int actual_fps = cap.get(cv::CAP_PROP_FPS);
    if (actual_fps != fps) {
        ROS_WARN_STREAM("FPS not setted correctly. Set: " << fps << ", actual: " << actual_fps);
    }
    if (actual_fps < pub_rate) {
        ROS_WARN_STREAM("image publish rate (" << pub_rate << ") is larger than actual fps (" << actual_fps << "), which may contain duplicated frames");
    }
    
    actual_width = int(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    actual_height = int(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    img_center.x = int(actual_width/2);
    img_center.y = int(actual_height/2);

    std::cout << std::endl;
    ROS_INFO_STREAM("\n\33[34m#################################\033[1m\33[35m Settings \033[0m\33[34m#################################\n"
                    "\n\33[36m Image size: \t\33[37m" << actual_width << ", " << actual_height << "\n"
                    "\n\33[36m Exposure time: \33[37m" << cap.get(cv::CAP_PROP_EXPOSURE) << ". (Setting: " << (exposure ? "Manual "+std::to_string(exposure)+" | 10000" : "Auto") << ")." << "\n"
                    "\n\33[36m FPS: \t\t\33[37m" << cap.get(cv::CAP_PROP_FPS) << ". (Setting: " << fps << "). " << "\n"
                    "\n\33[36m Publish rate: \t\33[37m" << pub_rate << "\n"
                    "\n\33[34m#################################\033[1m\33[35m Settings \033[0m\33[34m#################################\n");
}


void UsbCam::task_cb(const std_msgs::Bool::ConstPtr& msg) {
    task = msg->data;
}


void UsbCam::read_video_buffer() {
    if (!cap.isOpened()) {
        ROS_ERROR("No cap found");
        return ;
    }
    
    ROS_INFO("\33[32m--> USB Cam Started\33[0m");
    
    while (task) {                   
        // always retriving image using "grab" in a new thread.
        bool res = cap.grab();
        g_mutex.lock();
        if (res) {
            cap.retrieve(g_newest_frame);
        }
        g_mutex.unlock();
    }
}

void UsbCam::run() {
    std::string win_name = camera_name;  // "video "+std::to_string(video_port);

    retrieve_thread = std::thread(&UsbCam::read_video_buffer, this);   // start a new treading for reading.
    
    ros::Rate r(pub_rate);
    
    cv::Mat frame;

    std_msgs::Header hd;
    hd.frame_id = camera_name;

    while (ros::ok() and task) {
        g_mutex.lock();
        frame = g_newest_frame.clone();
        g_mutex.unlock();
        
        if (!frame.empty()) {
            hd.stamp = ros::Time::now();

            // cv::flip(src, src, 1);
            // cv::flip(src, src, 0);

            // cv::line(src, cv::Point(img_center.x, 0), cv::Point(img_center.x, actual_height),  cv::Scalar(0, 0, 255), 1);
            // cv::line(src, cv::Point(0, img_center.y), cv::Point(actual_width, img_center.),  cv::Scalar(0, 0, 255), 1);

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(hd, "bgr8", frame).toImageMsg();
            camera_image_pub.publish(msg);
            
            cam_info.header = hd;
            camera_info_pub.publish(cam_info);
        
            if (visualize) {
                cv::imshow(win_name, frame);
                int key = cv::waitKey(1) & 0xff;
                if (key == 27){
                    std::cout << std::endl;
                    ROS_WARN_STREAM("Stop viewing image, But still running, You can shut it down using \33[32mrostopic pub " << kill_switch_topic << " std_msgs/Bool \"data: false\"\33[0m\n");
                    visualize = false;
                    cv::destroyAllWindows();
                }
            }
        }
        
        ros::spinOnce();
        r.sleep();
    }

}
