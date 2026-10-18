#include "motor_encoder.h"

// 每个编码器独立的中断处理函数
static void encoder_isr(void *args, rt_bool_t is_pin_a)
{
	motor_encoder_obj_t *enc = (motor_encoder_obj_t *)args;
	rt_base_t current_pin = is_pin_a ? enc->pin_a : enc->pin_b;
	rt_base_t other_pin = is_pin_a ? enc->pin_b : enc->pin_a;

	rt_enter_critical();

	// 获取两个引脚的状态
	rt_uint8_t current_state = rt_pin_read(current_pin);
	rt_uint8_t other_state = rt_pin_read(other_pin);

	// 根据两个引脚的状态确定旋转方向
	if (is_pin_a)
	{
		// A相变化时
		if (current_state == other_state)
		{
			enc->pulse_count--; // 反转
		}
		else
		{
			enc->pulse_count++; // 正转
		}
	}
	else
	{
		// B相变化时
		if (current_state == other_state)
		{
			enc->pulse_count++; // 正转
		}
		else
		{
			enc->pulse_count--; // 反转
		}
	}

	rt_exit_critical();
}

static void encoder_isr1(void *args)
{
	encoder_isr(args, RT_TRUE);
}

static void encoder_isr2(void *args)
{
	encoder_isr(args, RT_FALSE);
}

/* 速度和里程计算函数 */
static void calculate_speed_and_distance(void *args)
{
	motor_encoder_obj_t *encoder = (motor_encoder_obj_t *)args;
	float new_speed;
	float wheel_circumference = encoder->params.wheel_diameter * 3.14159f;

	rt_enter_critical();
	rt_int32_t current_count = encoder->pulse_count;
	encoder->pulse_count = 0;
	rt_exit_critical();

	// 计算原始速度（RPM）
	new_speed = (current_count / (float)encoder->params.pulses_per_rev) / encoder->params.sample_time * 60.0f;

	// 应用低通滤波
	encoder->speed_rpm = lpf_update(&encoder->speed_filter, new_speed);

	// 计算线速度（m/s）并滤波
	new_speed = (encoder->speed_rpm / 60.0f) * wheel_circumference;
	encoder->speed_mps = lpf_update(&encoder->speed_mps_filter, new_speed);

	// 计算里程
	float revolutions = current_count / (float)encoder->params.pulses_per_rev;
	encoder->distance_m += revolutions * wheel_circumference;
}

motor_encoder_obj_t *motor_encoder_init(rt_base_t pin_a,
										rt_base_t pin_b,
										char *timer_name,
										encoder_params_t *params)
{
	motor_encoder_obj_t *obj = (motor_encoder_obj_t *)rt_malloc(sizeof(motor_encoder_obj_t));
	RT_ASSERT(obj != RT_NULL);

	rt_memset(obj, 0, sizeof(motor_encoder_obj_t));

	// 保存引脚配置
	obj->pin_a = pin_a;
	obj->pin_b = pin_b;

	// 保存编码器参数
	obj->params = *params;

	// 初始化滤波器
	lpf_init(&obj->speed_filter, params->filter_alpha);
	lpf_init(&obj->speed_mps_filter, params->filter_alpha);

	// 配置引脚和中断
	rt_pin_mode(pin_a, PIN_MODE_INPUT_PULLUP);
	rt_pin_mode(pin_b, PIN_MODE_INPUT_PULLUP);
	rt_pin_attach_irq(pin_a, PIN_IRQ_MODE_RISING_FALLING, encoder_isr1, obj);
	rt_pin_attach_irq(pin_b, PIN_IRQ_MODE_RISING_FALLING, encoder_isr2, obj);
	rt_pin_irq_enable(pin_a, PIN_IRQ_ENABLE);
	rt_pin_irq_enable(pin_b, PIN_IRQ_ENABLE);

	// 创建定时器
	uint32_t timer_period = (uint32_t)(params->sample_time * 1000); // 转换为ms
	obj->speed_timer = rt_timer_create(timer_name,
									   calculate_speed_and_distance,
									   obj,
									   timer_period,
									   RT_TIMER_FLAG_PERIODIC);
	RT_ASSERT(obj->speed_timer != RT_NULL);
	rt_timer_start(obj->speed_timer);

	return obj;
}

// #define TEST_ENCODER

#ifdef TEST_ENCODER
motor_encoder_obj_t *encoder1;

int test_motor_encoder_init(void)
{
	encoder1 = motor_encoder_init(GET_PIN(A, 15), GET_PIN(B, 3), "tim_enc1", NULL);

	return RT_EOK;
}

INIT_APP_EXPORT(test_motor_encoder_init);
#endif

// 添加函数实现
void lpf_init(lpf_t *filter, float alpha)
{
	filter->alpha = alpha;
	filter->filtered_value = 0.0f;
}

float lpf_update(lpf_t *filter, float new_value)
{
	filter->filtered_value = filter->alpha * new_value + (1.0f - filter->alpha) * filter->filtered_value;
	return filter->filtered_value;
}
