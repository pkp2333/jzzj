#include <iostream>
#include <mutex>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


std::mutex rgb_frame_mutex;
cv::Mat rgb_frame_sub;
std_msgs::Header hd;

std_msgs::Int16 at_target_id;


void at_target_id_cb(const std_msgs::Int16::ConstPtr& msg) {
    at_target_id = *msg;
}


void image_raw_cb(const sensor_msgs::Image::ConstPtr& msg) {
    
    cv::Mat msg2frame = cv_bridge::toCvShare(msg, msg->encoding)->image;
    hd = msg->header;
    rgb_frame_mutex.lock();
    rgb_frame_sub = msg2frame.clone();
    rgb_frame_mutex.unlock();

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "apriltag_localization");
	ros::NodeHandle nh;

    std::string image_topic_name, detected_topic_name, pose_topic_name;
    float tag_size;
    float tag_size_half;
    std::vector<int> default_id_list = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::vector<int> id_list;
    int resolve_method;
    std::string resolve_method_[6] = {"SOLVEPNP_ITERATIVE", "SOLVEPNP_EPNP", "SOLVEPNP_P3P", "SOLVEPNP_DLS", "SOLVEPNP_UPNP", "SOLVEPNP_AP3P"};
    bool visualize, pub_tf;

    nh.param<std::string>("apriltag_localization/image_topic_name", image_topic_name, "/usb_cam/image_raw");
    nh.param<std::string>("apriltag_localization/detected_topic_name", detected_topic_name, "/apriltag_detected");
    nh.param<std::string>("apriltag_localization/pose_topic_name", pose_topic_name, "/cam_pose_tag");
    nh.param<float>("apriltag_localization/tag_size", tag_size, 0.1);
    nh.param<std::vector<int>>("apriltag_localization/id_list", id_list, default_id_list);
    nh.param<int>("apriltag_localization/resolve_method", resolve_method, 0);
    nh.param<bool>("apriltag_localization/visualize", visualize, false);
    nh.param<bool>("apriltag_localization/pub_tf", pub_tf, false);

    size_t last_slash_pos = image_topic_name.find_last_of('/');
    std::string camera_name = image_topic_name.substr(0, last_slash_pos);

    int cameraWidth, cameraHeight;
    cv::Mat cameraMatrix, distCoeffs;
    boost::shared_ptr<sensor_msgs::CameraInfo const> camera_info_msg;
    camera_info_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_name+"/camera_info", ros::Duration(30));
    if (!camera_info_msg) {
        ROS_ERROR("Failed to receive camera info.");
        return -1;
    }
    cameraWidth = camera_info_msg->width;
    cameraHeight = camera_info_msg->height;
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        cameraMatrix.at<double>(i/3, i%3) = camera_info_msg->K[i];
    }
    distCoeffs = cv::Mat::zeros(camera_info_msg->D.size(), 1, CV_64F);
    for (size_t i = 0; i < camera_info_msg->D.size(); ++i) {
        distCoeffs.at<double>(i) = camera_info_msg->D[i];
    }
    int halfWidth = (int)cameraWidth/2;
    int halfHeight = (int)cameraHeight/2;

    tag_size_half = tag_size / 2;
    std::vector<cv::Point3f> objectPoints;
    objectPoints.push_back(cv::Point3f(0.0,  tag_size_half, -tag_size_half));  // bl
    objectPoints.push_back(cv::Point3f(0.0, -tag_size_half, -tag_size_half));  // br
    objectPoints.push_back(cv::Point3f(0.0, -tag_size_half,  tag_size_half));  // tr
    objectPoints.push_back(cv::Point3f(0.0,  tag_size_half,  tag_size_half));  // tl

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_raw_sub = it.subscribe(image_topic_name, 1, image_raw_cb);
    ros::Subscriber target_id_sub = nh.subscribe("at_target_id", 1, at_target_id_cb);

    ros::Publisher tag_det_pub = nh.advertise<std_msgs::Bool>(detected_topic_name, 1);
    ros::Publisher tag_img_pub = nh.advertise<sensor_msgs::Image>(camera_name+"/apriltag", 1);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic_name, 5);

    apriltag_family_t *tag_family = tag36h11_create();
    apriltag_detector_t *tag_detector = apriltag_detector_create();
    apriltag_detector_add_family(tag_detector, tag_family);

    std::string id_list_out;
    for(int i = 0; i < id_list.size(); ++i) {
        id_list_out = id_list_out + std::to_string(id_list[i]) + " / ";
    }
    ROS_INFO_STREAM("\33[32mApriltag Localization Node Successfully Initialized!\n"
                    "\n\33[34m#################################\033[1m\33[35m Settings \033[0m\33[34m#################################\n"
                    "\n\33[36m image_topic_name: \t\33[37m" << image_topic_name << "\n"
                    "\n\33[36m pose_topic_name: \t\33[37m" << pose_topic_name << "\n"
                    "\n\33[36m camera w/h: \t\t\33[37m" << cameraWidth  << " / " << cameraHeight << " pix\n"
                    "\n\33[36m tag_size: \t\t\33[37m" << tag_size << " m\n"
                    "\n\33[36m tag_id_list: \t\t\33[37m" << id_list_out << "\n"
                    "\n\33[36m resolve_method: \t\33[37m" << resolve_method_[resolve_method] << "\n"
                    "\n\33[34m#################################\033[1m\33[35m Settings \033[0m\33[34m#################################\n");

    int dets_size = 0;
    std_msgs::Bool at_detected;
    at_detected.data = false;
    at_target_id.data = -1;

    ros::Rate r(30);
    
    while (ros::ok()) {
        at_detected.data = false;
        
        if (!rgb_frame_sub.empty()) {
            
            cv::Mat frame, frame_gray;

            rgb_frame_mutex.lock();
            frame = rgb_frame_sub.clone();
            rgb_frame_mutex.unlock();

            auto start = std::chrono::system_clock::now();

            cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

            image_u8_t im = {.width = frame_gray.cols, .height = frame_gray.rows, .stride = frame_gray.cols, .buf = frame_gray.data};
            zarray_t *detections = apriltag_detector_detect(tag_detector, &im);

            cv::line(frame, cv::Point(halfWidth-15, halfHeight), cv::Point(halfWidth+15, halfHeight), cv::Scalar(255, 150, 0), 2);
            cv::line(frame, cv::Point(halfWidth, halfHeight-15), cv::Point(halfWidth, halfHeight+15), cv::Scalar(255, 150, 0), 2);

            dets_size = zarray_size(detections);
            
            std::vector<int> ids;
            std::vector<float> dist2s;
            std::vector<apriltag_detection_t*> dets;
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;

                zarray_get(detections, i, &det);

                int id_now = det->id;
                if (std::find(id_list.begin(), id_list.end(), id_now) != id_list.end()) {
                    ids.push_back(id_now);
                    float dist2 = std::pow(det->c[0]-halfWidth, 2) + std::pow(det->c[1]-halfHeight, 2);
                    dist2s.push_back(dist2);
                    dets.push_back(det);
                }
                putText(frame, std::to_string(det->id), cv::Point(det->c[0],det->c[1]), 3, 0.8, cv::Scalar(255, 0, 255), 1);
                
                apriltag_detection_destroy(det);
            }

            if (ids.size()) {
                int close_index;

                if (at_target_id.data < 0) {
                    auto close_it = std::min_element(dist2s.begin(), dist2s.end());
                    close_index = std::distance(dist2s.begin(), close_it);
                }
                else {
                    auto appointed_it = std::find(ids.begin(), ids.end(), at_target_id.data);
                    if (appointed_it != ids.end()) {
                        close_index = std::distance(ids.begin(), appointed_it);
                    }
                    else {
                        auto close_it = std::min_element(dist2s.begin(), dist2s.end());
                        close_index = std::distance(dist2s.begin(), close_it);
                    }
                }

                std::vector<cv::Point2f> imagePoints;

                cv::circle(frame, cv::Point(dets[close_index]->p[0][0], dets[close_index]->p[0][1]), 4, cv::Scalar(255, 0, 0), 2);  // bl
                cv::circle(frame, cv::Point(dets[close_index]->p[1][0], dets[close_index]->p[1][1]), 4, cv::Scalar(0, 255, 0), 2);  // br
                cv::circle(frame, cv::Point(dets[close_index]->p[2][0], dets[close_index]->p[2][1]), 4, cv::Scalar(0, 0, 255), 2);  // tr
                cv::circle(frame, cv::Point(dets[close_index]->p[3][0], dets[close_index]->p[3][1]), 4, cv::Scalar(255, 0, 255), 2);  // tl

                imagePoints.push_back(cv::Point2f(dets[close_index]->p[0][0], dets[close_index]->p[0][1]));
                imagePoints.push_back(cv::Point2f(dets[close_index]->p[1][0], dets[close_index]->p[1][1]));
                imagePoints.push_back(cv::Point2f(dets[close_index]->p[2][0], dets[close_index]->p[2][1]));
                imagePoints.push_back(cv::Point2f(dets[close_index]->p[3][0], dets[close_index]->p[3][1]));
                
                cv::Mat r_vec, t_vec;
                bool success = cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, r_vec, t_vec, false, resolve_method);

                if (success) {
                
                    at_detected.data = true;
                    
                    if (visualize) {
                        cv::putText(frame, "solvePnP Success", 
                                    cv::Point(halfWidth, cameraHeight-10), 
                                    3, 1, cv::Scalar(0, 255, 0), 2);
                    }
                    
                    cv::Mat R;
                    cv::Rodrigues(r_vec, R);

                    tf2::Matrix3x3 tf_R(
                        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
                    );
                    tf2::Vector3 tf_t(t_vec.at<double>(0), t_vec.at<double>(1), t_vec.at<double>(2));
                    tf2::Transform transform_cam_tag(tf_R, tf_t);
                    tf2::Transform transform_tag_cam = transform_cam_tag.inverse();
                    tf2::Vector3 translation = transform_tag_cam.getOrigin();
                    tf2::Quaternion quaternion;
                    transform_tag_cam.getBasis().getRotation(quaternion);

                    tf2::Matrix3x3 rotation_matrix(quaternion);
                    double roll, pitch, yaw;
                    rotation_matrix.getRPY(roll, pitch, yaw);
                    roll = roll * 180.0 / M_PI;
                    pitch = pitch * 180.0 / M_PI;
                    yaw = yaw * 180.0 / M_PI;

                    geometry_msgs::PoseWithCovarianceStamped cam_pose;
                    cam_pose.header.stamp = ros::Time::now();
                    cam_pose.header.frame_id = "tag_" + std::to_string(ids[close_index]);
                    cam_pose.pose.pose.position.x = translation.x();
                    cam_pose.pose.pose.position.y = translation.y();
                    cam_pose.pose.pose.position.z = translation.z();
                    cam_pose.pose.pose.orientation.x = quaternion.x();
                    cam_pose.pose.pose.orientation.y = quaternion.y();
                    cam_pose.pose.pose.orientation.z = quaternion.z();
                    cam_pose.pose.pose.orientation.w = quaternion.w();

                    pose_pub.publish(cam_pose);

                    if (pub_tf) {
                        geometry_msgs::TransformStamped transformStamped;
                        transformStamped.header.stamp = ros::Time::now();
                        transformStamped.header.frame_id = "tag_" + std::to_string(ids[close_index]);
                        transformStamped.child_frame_id = hd.frame_id;
                        transformStamped.transform.translation.x = translation.x();
                        transformStamped.transform.translation.y = translation.y();
                        transformStamped.transform.translation.z = translation.z();
                        transformStamped.transform.rotation.x = quaternion.x();
                        transformStamped.transform.rotation.y = quaternion.y();
                        transformStamped.transform.rotation.z = quaternion.z();
                        transformStamped.transform.rotation.w = quaternion.w();

                        static tf2_ros::TransformBroadcaster br;
                        br.sendTransform(transformStamped);
                    }
                    
                    if (visualize) {
                        std::ostringstream ss;
                        ss << "T: x:" << std::fixed << std::setprecision(1) << translation.x()*100.0
                        << " y:" << translation.y()*100.0
                        << " z:" << translation.z()*100.0;
                        cv::putText(frame, ss.str(), cv::Point(20, cameraHeight-80), 3, 1, cv::Scalar(0, 0, 255), 2);
                        ss.str("");

                        ss << "T: r:" << std::fixed << std::setprecision(1) << roll
                        << " p:" << pitch
                        << " y:" << yaw;
                        cv::putText(frame, ss.str(), cv::Point(20, cameraHeight-40), 3, 1, cv::Scalar(0, 0, 255), 2);
                        ss.str("");
                    }

                } else {
                    if (visualize) {
                        cv::putText(frame, "solvePnP Failure", 
                                    cv::Point(halfWidth, cameraHeight-10), 
                                    3, 1, cv::Scalar(0, 0, 255), 2);
                    }
                }
            }
            else {
                if (visualize) {
                    cv::putText(frame, "No Tag", 
                                cv::Point(halfWidth, cameraHeight-10), 
                                3, 1, cv::Scalar(0, 0, 255), 2);
                }
            }

            if (visualize) {
                auto end = std::chrono::system_clock::now();
                int fps = 1000000 / std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                cv::putText(frame, "FPS: " + std::to_string(fps), 
                            cv::Point(20, cameraHeight-10), 
                            3, 1, cv::Scalar(255, 0, 255), 2);
                cv::imshow("Apriltag", frame);
                zarray_destroy(detections);
                int key = 0;
                key = cv::waitKey(1) & 0xff;
                if (key == 27 || key == 'q' || key == 'Q') break;
            }

            tag_det_pub.publish(at_detected);
            sensor_msgs::ImagePtr at_img = cv_bridge::CvImage(hd, "bgr8", frame).toImageMsg();
            tag_img_pub.publish(at_img);
        
        }
        
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

