/*
 * @Author: wzw
 * @Date: 2025-02-18 13:51:38
 * @LastEditors: wzw
 * @Description: Do not edit
 */
#ifndef __MOTOR_ENCODER_H
#define __MOTOR_ENCODER_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

/******根据使用的电机型号，确定这里使用的分辨率的宏定义，只能有一个定义*********/

#define PULSES_PER_REVOLUTION 2464 // 178rpm JGB37-520

// #define PULSES_PER_REVOLUTION 1540    // 170rpm JGB25-370

// #define PULSES_PER_REVOLUTION 440 // 620rpm JGB25-370

/*******************************************************************************/

typedef struct
{
	float alpha;		  // 滤波系数
	float filtered_value; // 滤波后的值
} lpf_t;

/* 编码器参数结构体 */
typedef struct
{
	uint16_t pulses_per_rev; // 编码器每圈脉冲数
	float wheel_diameter;	 // 轮子直径(m)
	float sample_time;		 // 采样时间(s)
	float filter_alpha;		 // 滤波系数
} encoder_params_t;

typedef struct
{
	rt_base_t pin_a;
	rt_base_t pin_b;

	rt_int32_t pulse_count; /* absolute pulse number in total */

	float speed_rpm;  // 速度，单位 RPM
	float distance_m; // 里程，单位米
	float speed_mps;

	lpf_t speed_filter;		// RPM低通滤波器
	lpf_t speed_mps_filter; // m/s低通滤波器

	rt_timer_t speed_timer; // 速度计算定时器
	char *timer_name;		// 定时器别名

	// 添加参数成员
	encoder_params_t params; // 编码器参数

	void (*cal_spd_and_dis)(void *args);
	void (*encoder_isr1)(void *args);
	void (*encoder_isr2)(void *args);
} motor_encoder_obj_t;

// 修改初始化函数声明
motor_encoder_obj_t *motor_encoder_init(rt_base_t pin_a,
										rt_base_t pin_b,
										char *timer_name,
										encoder_params_t *params);

// 添加函数声明
void lpf_init(lpf_t *filter, float alpha);
float lpf_update(lpf_t *filter, float new_value);

#endif
