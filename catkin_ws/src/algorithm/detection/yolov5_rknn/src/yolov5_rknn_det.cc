// Copyright (c) 2021 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*-------------------------------------------
                Includes
-------------------------------------------*/
#include <iostream>
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "user_msgs/ObjDet.h"
#include "user_msgs/ObjDets.h"

#define _BASETSD_H

#include "RgaUtils.h"
#include "im2d.h"
#include "rga.h"
#include "rknn_api.h"
#include "yolov5_rknn_func.hpp"
#include "BYTETracker.h"

#define PERF_WITH_POST 1

bool task = true;
std::mutex frame_mutex;
cv::Mat frame_sub;
std_msgs::Header hd;


void task_cb(const std_msgs::Bool::ConstPtr& msg) {

    task = msg->data;

}


void image_raw_cb(const sensor_msgs::Image::ConstPtr& msg) {
    
    cv::Mat msg2frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    hd = msg->header;
    frame_mutex.lock();
    frame_sub = msg2frame.clone();
    frame_mutex.unlock();

}


void pub_dets(const ros::Publisher& publisher, 
              detect_result_group_t& dets, 
              int& fps,
              std::unordered_map<int, std::string>& labels_map) {

    user_msgs::ObjDets Dets;
    
    Dets.header = hd;
    Dets.fps = fps;
    Dets.num = dets.count;
    
    for (size_t i = 0; i < Dets.num; i++) {
        user_msgs::ObjDet Det;
        Det.index = 0;
        Det.class_id = (int)dets.results[i].class_id;
        Det.label = labels_map[Det.class_id];
        Det.score = dets.results[i].conf;
        Det.bbox.x1 = dets.results[i].bbox.left;
        Det.bbox.y1 = dets.results[i].bbox.top;
        Det.bbox.x2 = dets.results[i].bbox.right;
        Det.bbox.y2 = dets.results[i].bbox.bottom;
        Det.bbox.x = (Det.bbox.x1 + Det.bbox.x2) / 2.0;
        Det.bbox.y = (Det.bbox.y1 + Det.bbox.y2) / 2.0;
        Det.bbox.w = Det.bbox.x2 - Det.bbox.x1;
        Det.bbox.h = Det.bbox.y2 - Det.bbox.y1;
        Dets.dets.push_back(Det);
    }

    publisher.publish(Dets);

}


void pub_sorted_dets(const ros::Publisher& publisher,
                     std::vector<STrack>& output_stracks,
                     int& fps,
                     std::unordered_map<int, std::string>& labels_map) {

    user_msgs::ObjDets Dets;
    
    Dets.header = hd;
    Dets.fps = fps;
    Dets.num = output_stracks.size();

    for (size_t i = 0; i < Dets.num; i++) {
        user_msgs::ObjDet Det;
        Det.index = output_stracks[i].track_id;
        Det.class_id = output_stracks[i].class_id;
        Det.label = labels_map[Det.class_id];
        Det.score = output_stracks[i].score;
        std::vector<float> tlwh = output_stracks[i].tlwh;
        int x1 = tlwh[0];
        int y1 = tlwh[1];
        int x2 = tlwh[0] + tlwh[2];
        int y2 = tlwh[1] + tlwh[3];
        Det.bbox.x1 = tlwh[0];
        Det.bbox.y1 = tlwh[1];
        Det.bbox.x2 = tlwh[0] + tlwh[2];
        Det.bbox.y2 = tlwh[1] + tlwh[3];
        Det.bbox.x = tlwh[0] + tlwh[2] / 2.0;
        Det.bbox.y = tlwh[1] + tlwh[3] / 2.0;
        Det.bbox.w = tlwh[2];
        Det.bbox.h = tlwh[3];
        Dets.dets.push_back(Det);
    }

    publisher.publish(Dets);

}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "Yolov5_rknn_det");
	ros::NodeHandle nh;

    int camera_pub_rate;
    std::string image_topic_name;
    std::string model_url;
    std::string labels_url;
    int class_num;
    std::vector<float> default_anchors = {10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326};
    std::vector<float> anchors;
    float ConfThresh, NmsThresh;
    int track_buffer;  // target_class_id,
    float track_thresh, high_thresh, match_thresh;
    std::string kill_switch_topic;
    bool visualize;

    nh.param<int>("/usb_cam/pub_rate", camera_pub_rate, 30);
    nh.param<std::string>("image_topic_name", image_topic_name, "/usb_cam/image_raw");
    nh.param<std::string>("model_url", model_url, "/home/orangepi/Documents/yolov5_weights/coco_s_rk3588s.rknn");
    nh.param<std::string>("labels_url", labels_url, "/home/orangepi/Documents/yolov5_weights/labels_coco.txt");
    nh.param<int>("class_num", class_num, 80);
    nh.param<std::vector<float>>("anchors", anchors, default_anchors);
    // ROS_INFO("Got anchors param: %d", anchors.size());
    // for (float num : anchors) {
    //     ROS_INFO("%f", num);
    // }
    nh.param<float>("conf_thresh", ConfThresh, 0.45f);
    nh.param<float>("iou_thresh", NmsThresh, 0.45f);

    nh.param<float>("byte_track/track_thresh", track_thresh, 0.5f);
    nh.param<float>("byte_track/high_thresh", high_thresh, 0.6f);
    nh.param<float>("byte_track/match_thresh", match_thresh, 0.8f);
    nh.param<int>("byte_track/track_buffer", track_buffer, 30);

    nh.param<std::string>("kill_switch_topic", kill_switch_topic, "/task");
    nh.param<bool>("visualize", visualize, false);

    // Get camera name
    size_t last_slash_pos = image_topic_name.find_last_of('/');
    std::string camera_name = image_topic_name.substr(0, last_slash_pos);

    // Get engine name
    last_slash_pos = model_url.find_last_of('/');
    std::string model_file_name = model_url.substr(last_slash_pos, model_url.size());
    size_t first_dot_pos = model_file_name.find_first_of('.');
    std::string model_name = model_file_name.substr(1, first_dot_pos - 1);

    // Read the txt file for classnames
    std::ifstream labels_file(labels_url, std::ios::binary);
    if (!labels_file.good()) {
        ROS_ERROR_STREAM("Read " << labels_url << " error!\n");
        return -1;
    }
    std::unordered_map<int, std::string> labels_map;
    read_labels(labels_url, labels_map, class_num);

    assert(anchors.size() == 18);
    const float anchor0[6] = {anchors[0], anchors[1], anchors[2], anchors[3], anchors[4], anchors[5]};
    const float anchor1[6] = {anchors[6], anchors[7], anchors[8], anchors[9], anchors[10], anchors[11]};
    const float anchor2[6] = {anchors[12], anchors[13], anchors[14], anchors[15], anchors[16], anchors[17]};

    int            status             = 0;
    rknn_context   ctx;
    size_t         actual_size        = 0;
    int            img_width          = 0;
    int            img_height         = 0;
    int            img_channel        = 0;
    int            ret;

    // init rga context
    rga_buffer_t src;
    rga_buffer_t dst;
    im_rect      src_rect;
    im_rect      dst_rect;
    memset(&src_rect, 0, sizeof(src_rect));
    memset(&dst_rect, 0, sizeof(dst_rect));
    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));

    /* Create the neural network */
    printf("Loading mode...\n");

    int            model_data_size = 0;
    unsigned char* model_data      = load_model(model_url, &model_data_size);
    ret                            = rknn_init(&ctx, model_data, model_data_size, 0, NULL);
    
    if (ret < 0) {
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }

    rknn_sdk_version version;
    ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(rknn_sdk_version));
    if (ret < 0) {
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }
    printf("sdk version: %s driver version: %s\n", version.api_version, version.drv_version);

    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret < 0) {
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }
    printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for (int i = 0; i < io_num.n_input; i++) {
        input_attrs[i].index = i;
        ret                  = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret < 0) {
            printf("rknn_init error ret=%d\n", ret);
            return -1;
        }
        // dump_tensor_attr(&(input_attrs[i]));
    }

    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (int i = 0; i < io_num.n_output; i++) {
        output_attrs[i].index = i;
        ret                   = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        // dump_tensor_attr(&(output_attrs[i]));
    }

    int channel = 3;
    int width   = 0;
    int height  = 0;
    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
        printf("model is NCHW input fmt\n");
        channel = input_attrs[0].dims[1];
        height  = input_attrs[0].dims[2];
        width   = input_attrs[0].dims[3];
    } else {
        printf("model is NHWC input fmt\n");
        height  = input_attrs[0].dims[1];
        width   = input_attrs[0].dims[2];
        channel = input_attrs[0].dims[3];
    }

    printf("model input height=%d, width=%d, channel=%d\n", height, width, channel);

    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index        = 0;
    inputs[0].type         = RKNN_TENSOR_UINT8;
    inputs[0].size         = width * height * channel;
    inputs[0].fmt          = RKNN_TENSOR_NHWC;
    inputs[0].pass_through = 0;

    BYTETracker bytetracker(camera_pub_rate, track_thresh, high_thresh, match_thresh, track_buffer);

    // subscriber
    ros::Subscriber task_sub = nh.subscribe(kill_switch_topic, 1, task_cb);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_raw_sub = it.subscribe(image_topic_name, 1, image_raw_cb);

    // publisher
    ros::Publisher detection_pub = nh.advertise<user_msgs::ObjDets>(camera_name+"/yolov5_detections", 1);

    std::string win_name = "Yolov5_rknn_det - " + model_name + " - " + image_topic_name;

    cv::Size target_size(width, height);
    // void* resize_buf = nullptr;

    std::cout << std::endl;
    ROS_INFO_STREAM("\33[32mYolov5_rknn_det Node Successfully Initialized!\n"
                    "\n\33[34m#################################\033[1m\33[35m Settings \033[0m\33[34m#################################\n"
                    "\n\33[36m image_topic_name: \t\33[37m" << image_topic_name << "\n"
                    "\n\33[36m Model: \t\33[37m" << model_url  <<
                    "\n\33[36m Labels: \t\33[37m" << labels_url << "\n"
                    "\n\33[36m Conf Thresh: \t\33[37m" << ConfThresh <<
                    "\n\33[36m IOU Thresh: \t\33[37m" << NmsThresh << "\n"
                    "\n\33[36m Track Thresh: \t\33[37m" << track_thresh <<
                    "\n\33[36m High Thresh: \t\33[37m" << high_thresh <<
                    "\n\33[36m Match Thresh: \t\33[37m" << match_thresh <<
                    "\n\33[36m Track Buffer: \t\33[37m" << track_buffer <<  "\n"
                    "\n\33[36m Visualize: \t\33[37m" << visualize <<  "\n"
                    "\n\33[34m#################################\033[1m\33[35m Settings \033[0m\33[34m#################################\n");


    ros::Rate r(30);
    
    while (ros::ok() and task) {
        if (!frame_sub.empty()) {
            
            cv::Mat frame;

            frame_mutex.lock();
            frame = frame_sub.clone();
            frame_mutex.unlock();

            auto start = std::chrono::system_clock::now();
    
            img_width = frame.cols;
            img_height = frame.rows;

            float scale_w = (float)width / img_width;
            float scale_h = (float)height / img_height;

            
            cv::Mat resized_img(target_size.height, target_size.width, CV_8UC3);
        
            cv::Mat img;
            cv::cvtColor(frame, img, cv::COLOR_BGR2RGB);
            
            // You may not need resize when src resulotion equals to dst resulotion

            if (img_width != width || img_height != height) {
                rga_buffer_t src;
                rga_buffer_t dst;
                memset(&src, 0, sizeof(src));
                memset(&dst, 0, sizeof(dst));
                ret = resize_rga(src, dst, img, resized_img, target_size);
                if (ret != 0) {
                    fprintf(stderr, "resize with rga error\n");
                }
                inputs[0].buf = resized_img.data;

                // // printf("resize with RGA!\n");
                // resize_buf = malloc(height * width * channel);
                // memset(resize_buf, 0x00, height * width * channel);

                // src = wrapbuffer_virtualaddr((void*)img.data, img_width, img_height, RK_FORMAT_RGB_888);
                // dst = wrapbuffer_virtualaddr((void*)resize_buf, width, height, RK_FORMAT_RGB_888);
                // ret = imcheck(src, dst, src_rect, dst_rect);
                // if (IM_STATUS_NOERROR != ret) {
                //     printf("%d, check error! %s", __LINE__, imStrError((IM_STATUS)ret));
                //     return -1;
                // }
                // IM_STATUS STATUS = imresize(src, dst);

                // inputs[0].buf = resize_buf;
            } else {
                inputs[0].buf = (void*)img.data;
            }

            rknn_inputs_set(ctx, io_num.n_input, inputs);

            rknn_output outputs[io_num.n_output];
            memset(outputs, 0, sizeof(outputs));
            for (int i = 0; i < io_num.n_output; i++) {
                outputs[i].want_float = 0;
            }

            ret = rknn_run(ctx, NULL);
            ret = rknn_outputs_get(ctx, io_num.n_output, outputs, NULL);
            // gettimeofday(&stop_time, NULL);
            // printf("once run use %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);

            // post process
            detect_result_group_t detect_result_group;
            std::vector<float>    out_scales;
            std::vector<int32_t>  out_zps;
            for (int i = 0; i < io_num.n_output; ++i) {
                out_scales.push_back(output_attrs[i].scale);
                out_zps.push_back(output_attrs[i].zp);
            }
            post_process(class_num,
                         (int8_t*)outputs[0].buf, (int8_t*)outputs[1].buf, (int8_t*)outputs[2].buf, 
                         height, width,
                         (float*)anchor0, (float*)anchor1, (float*)anchor2,
                         ConfThresh, NmsThresh, 
                         scale_w, scale_h, out_zps, out_scales, &detect_result_group);

            std::vector<STrack> output_stracks = bytetracker.update(detect_result_group, 0);

            auto end = std::chrono::system_clock::now();
            int fps = 1000000 / std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

            // pub_dets(detection_pub, detect_result_group, fps, labels_map);
            pub_sorted_dets(detection_pub, output_stracks, fps, labels_map);

            if (visualize) {
                // draw_bbox(frame, detect_result_group, labels_map);

                draw_sorted_bbox(frame, output_stracks, labels_map);

                cv::putText(frame, "FPS: " + std::to_string(fps), 
                            cv::Point(30, 30), 
                            fontFace, fontScale, cv::Scalar(255, 0, 255), thickness);
                            
                cv::imshow(win_name, frame);
            }
            
            int key = 0;
            key = cv::waitKey(1) & 0xff;
            if (key == 27 || key == 'q' || key == 'Q') break;
            
            ret = rknn_outputs_release(ctx, io_num.n_output, outputs);

            // if (resize_buf) {
            //     free(resize_buf);
            // }
        }
        else {
            // std::cout << "No Frame Received\n";
        }
        
        ros::spinOnce();
        r.sleep();

    }

    // release
    ret = rknn_destroy(ctx);

    if (model_data) {
        free(model_data);
    }

    ROS_WARN("\33[32mYolov5_rknn_det Node Node Terminated\33[0m");

    return 0;
}
