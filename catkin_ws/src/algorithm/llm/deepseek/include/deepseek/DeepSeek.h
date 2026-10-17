#pragma once

#include <iostream>
#include <cstring>
#include <string>
#include <regex>
#include "librkllm_api/rkllm.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#define PROMPT_TEXT_PREFIX  "<｜begin▁of▁sentence｜><｜User｜>"
#define PROMPT_TEXT_POSTFIX "<｜Assistant｜>"

class DeepSeek
{
public:
    DeepSeek(ros::NodeHandle& nh);
    ~DeepSeek();

    bool ds_task = true;

private:
    ros::Subscriber ds_task_sub;
    ros::Subscriber input_text_sub;
    ros::Publisher output_text_pub;

    LLMHandle llmHandle;
    RKLLMInferParam rkllm_infer_params;

    ros::Time start_time;
    std::string output_buffer;

    std::string model_url, kill_switch_topic;
    int max_new_tokens;
    int max_context_len;
    float temperature;
    float top_p;
    int top_k;
    float repeat_penalty;

    static const std::regex think_re;

    void destroy_ds();

    void ds_task_cb(const std_msgs::Bool::ConstPtr& msg);
    void input_text_cb(const std_msgs::String::ConstPtr& msg);
    static void inference_cb(RKLLMResult* result, void* userdata, LLMCallState state);
};
