#include "deepseek/DeepSeek.h"


const std::regex DeepSeek::think_re("<think>[\\s\\S]*?</think>");

DeepSeek::DeepSeek(ros::NodeHandle& nh) : llmHandle(nullptr) {
    nh.param<std::string>("deepseek/model_url", model_url, "/home/orangepi/Documents/deepseek_weights/DeepSeek-R1-Distill-Qwen-1.5B_W8A8_RK3588.rkllm");
    nh.param<int>("deepseek/max_new_tokens", max_new_tokens, 2048);
    nh.param<int>("deepseek/max_context_len", max_context_len, 4096);
    nh.param<float>("deepseek/temperature", temperature, 0.8);
    nh.param<float>("deepseek/top_p", top_p, 0.95);
    nh.param<int>("deepseek/top_k", top_k, 1);
    nh.param<float>("deepseek/repeat_penalty", repeat_penalty, 1.1);
    nh.param<std::string>("/deepseek/kill_switch_topic", kill_switch_topic, "/deepseek/task");

    ds_task_sub = nh.subscribe(kill_switch_topic, 1, &DeepSeek::ds_task_cb, this);
    input_text_sub = nh.subscribe("/deepseek/input_text", 10, &DeepSeek::input_text_cb, this);

    output_text_pub = nh.advertise<std_msgs::String>("/deepseek/output_text", 10);

    RKLLMParam param = rkllm_createDefaultParam();
    param.model_path = model_url.c_str();
    param.max_new_tokens = max_new_tokens;
    param.max_context_len = max_context_len;
    param.temperature = temperature;
    param.top_p = top_p;
    param.top_k = top_k;
    param.repeat_penalty = repeat_penalty;
    param.frequency_penalty = 0.0;
    param.presence_penalty = 0.0;
    param.skip_special_token = true;
    param.extend_param.base_domain_id = 0;
    
    int ret = rkllm_init(&llmHandle, &param, &DeepSeek::inference_cb);
    if (ret != 0 || llmHandle == nullptr) {
        ROS_ERROR("Failed to initialize RKLLM (ret=%d)", ret);
        ros::shutdown();
        return;
    }
    
    // 初始化 infer 参数结构体
    memset(&rkllm_infer_params, 0, sizeof(RKLLMInferParam));  // 将所有内容初始化为 0

    // 1. 初始化并设置 LoRA 参数（如果需要使用 LoRA）
    // RKLLMLoraAdapter lora_adapter;
    // memset(&lora_adapter, 0, sizeof(RKLLMLoraAdapter));
    // lora_adapter.lora_adapter_path = "qwen0.5b_fp16_lora.rkllm";
    // lora_adapter.lora_adapter_name = "test";
    // lora_adapter.scale = 1.0;
    // ret = rkllm_load_lora(llmHandle, &lora_adapter);
    // if (ret != 0) {
    //     printf("\nload lora failed\n");
    // }

    // 加载第二个lora
    // lora_adapter.lora_adapter_path = "Qwen2-0.5B-Instruct-all-rank8-F16-LoRA.gguf";
    // lora_adapter.lora_adapter_name = "knowledge_old";
    // lora_adapter.scale = 1.0;
    // ret = rkllm_load_lora(llmHandle, &lora_adapter);
    // if (ret != 0) {
    //     printf("\nload lora failed\n");
    // }

    // RKLLMLoraParam lora_params;
    // lora_params.lora_adapter_name = "test";  // 指定用于推理的 lora 名称
    // rkllm_infer_params.lora_params = &lora_params;

    // 2. 初始化并设置 Prompt Cache 参数（如果需要使用 prompt cache）
    // RKLLMPromptCacheParam prompt_cache_params;
    // prompt_cache_params.save_prompt_cache = true;                  // 是否保存 prompt cache
    // prompt_cache_params.prompt_cache_path = "./prompt_cache.bin";  // 若需要保存prompt cache, 指定 cache 文件路径
    // rkllm_infer_params.prompt_cache_params = &prompt_cache_params;
    
    // rkllm_load_prompt_cache(llmHandle, "./prompt_cache.bin"); // 加载缓存的cache

    rkllm_infer_params.mode = RKLLM_INFER_GENERATE;
    
    std::cout << std::endl;
    ROS_INFO_STREAM("\33[32mDeepSeek RKLLM Node Successfully Initialized!\n"
                    "\n\33[34m#################################\033[1m\33[35m Settings \033[0m\33[34m#################################\n"
                    "\n\33[36m Model: \t\t\33[37m" << model_url  <<
                    "\n\33[36m max_new_tokens: \t\33[37m" << max_new_tokens << "\n"
                    "\n\33[36m max_context_len: \t\33[37m" << max_context_len <<
                    "\n\33[34m#################################\033[1m\33[35m Settings \033[0m\33[34m#################################\n");

}


DeepSeek::~DeepSeek() {
    destroy_ds();
}


void DeepSeek::destroy_ds() {
    if (llmHandle) {
        ROS_WARN("RKLLM Release\n");
        rkllm_destroy(llmHandle);
        llmHandle = nullptr;
    }
}


void DeepSeek::ds_task_cb(const std_msgs::Bool::ConstPtr& msg) {
    if (!msg->data) {
        ds_task = false;
        destroy_ds();
    }
}


void DeepSeek::input_text_cb(const std_msgs::String::ConstPtr& msg) {
    std::string prompt = PROMPT_TEXT_PREFIX + msg->data + PROMPT_TEXT_POSTFIX;
    RKLLMInput rkllm_input;
    rkllm_input.input_type   = RKLLM_INPUT_PROMPT;
    rkllm_input.prompt_input = (char*)prompt.c_str();

    output_buffer.clear();

    ROS_INFO("\33[32mStarting inference for input\33[0m");
    std::cout << "\33[34m##################################################################\33[0m\n" << msg->data << "\n=================================================================\n";
    start_time = ros::Time::now();
    rkllm_run(llmHandle, &rkllm_input, &rkllm_infer_params, this);
}


void DeepSeek::inference_cb(RKLLMResult* result, void* userdata, LLMCallState state) {
    DeepSeek* self = static_cast<DeepSeek*>(userdata);
    if (!self) return;

    if (state == RKLLM_RUN_NORMAL) {
        std::string chunk = result->text ? result->text : "";
        self->output_buffer += chunk;
        std::cout << chunk;

    } else if (state == RKLLM_RUN_FINISH) {
        std::string clean = std::regex_replace(self->output_buffer,
                                               think_re, "");
        std_msgs::String out_msg;
        out_msg.data = clean;
        self->output_text_pub.publish(out_msg);
        double time_cost = std::round((ros::Time::now() - self->start_time).toSec() * 10) / 10.0;
        std::cout << "\n=================================================================\n";
        ROS_INFO("Inference completed, Cost \33[32m%.1f s\33[0m, Output Published\n", time_cost);
        std::cout << "\33[34m##################################################################\33[0m\n\n\n\n";

    } else if (state == RKLLM_RUN_ERROR) {
        ROS_ERROR("RKLLM inference error");

    } else {
        // ROS_ERROR("Unknown error");
    }
}



