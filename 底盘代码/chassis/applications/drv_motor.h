/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 文件名称: drv_motor.h
 * 文件说明: 通用电机驱动接口头文件 (基于DRV8870实现)
 * 作    者: Your Name
 * 日    期: 2023-xx-xx
 */

#ifndef __DRV_MOTOR_H__
#define __DRV_MOTOR_H__

#include <rtthread.h>
#include <rtdevice.h>

/* 电机运行模式枚举 */
typedef enum {
    MOTOR_MODE_STOP = 0,   /* 停止模式 */
    MOTOR_MODE_FORWARD,    /* 正转模式 */
    MOTOR_MODE_REVERSE,    /* 反转模式 */
    MOTOR_MODE_BRAKE       /* 刹车模式 */
} motor_mode_t;

/* 电机驱动结构体 */
struct motor_device
{
    struct rt_device_pwm *pwm_dev1;/* PWM设备1 */
    struct rt_device_pwm *pwm_dev2;/* PWM设备2 */
    rt_uint32_t pwm_channel1;      /* PWM通道1 */
    rt_uint32_t pwm_channel2;      /* PWM通道2 */
    rt_uint32_t pwm_period;        /* PWM周期(单位:纳秒) */
    rt_int16_t  speed;             /* 当前速度值:-100~100 */
    motor_mode_t mode;             /* 当前运行模式 */
    char pwm_dev1_name[RT_NAME_MAX];/* PWM设备1名称 */
    char pwm_dev2_name[RT_NAME_MAX];/* PWM设备2名称 */
    
    /* 私有实现数据 (DRV8870驱动特殊配置) */
    rt_uint8_t reserved[4];        /* 预留字节用于扩展 */
};
typedef struct motor_device *motor_device_t;

/* 函数声明 */
rt_err_t motor_init(struct motor_device *motor, const char *pwm_dev1_name, rt_uint32_t channel1,
                   const char *pwm_dev2_name, rt_uint32_t channel2, rt_uint32_t pwm_freq_hz);
rt_err_t motor_set_speed(struct motor_device *motor, rt_int16_t speed);
rt_int16_t motor_get_speed(struct motor_device *motor);
rt_err_t motor_stop(struct motor_device *motor);
rt_err_t motor_brake(struct motor_device *motor);
motor_mode_t motor_get_mode(struct motor_device *motor);

#endif /* __DRV_MOTOR_H__ */
