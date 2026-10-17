/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 文件名称: drv_motor.c
 * 文件说明: 通用电机驱动接口源文件 (基于DRV8870实现)
 * 作    者: Your Name
 * 日    期: 2023-xx-xx
 */

#include "drv_motor.h"

#define DBG_TAG             "motor"
#define DBG_LVL             DBG_LOG//DBG_INFO
#include <rtdbg.h>

/**
 * 初始化电机设备
 * 
 * @param motor 电机设备结构体指针
 * @param pwm_dev1_name PWM设备1名称
 * @param channel1 PWM设备1通道
 * @param pwm_dev2_name PWM设备2名称
 * @param channel2 PWM设备2通道
 * @param pwm_freq_hz PWM频率(单位:Hz)
 * 
 * @return 0成功，否则失败
 */
rt_err_t motor_init(struct motor_device *motor, const char *pwm_dev1_name, rt_uint32_t channel1,
                   const char *pwm_dev2_name, rt_uint32_t channel2, rt_uint32_t pwm_freq_hz)
{
    RT_ASSERT(motor != RT_NULL);
    RT_ASSERT(pwm_dev1_name != RT_NULL);
    RT_ASSERT(pwm_dev2_name != RT_NULL);
    
    rt_uint32_t pwm_period;
    
    /* 验证PWM频率 */
    if (pwm_freq_hz == 0 || pwm_freq_hz > 100000)
    {
        LOG_E("Invalid PWM frequency: %d Hz (should be between 1 and 100000 Hz)", pwm_freq_hz);
        return -RT_EINVAL;
    }
    
    /* 计算PWM周期(ns) */
    pwm_period = 1000000000 / pwm_freq_hz;
    
    /* 设置PWM设备名称和通道 */
    rt_strncpy(motor->pwm_dev1_name, pwm_dev1_name, RT_NAME_MAX);
    rt_strncpy(motor->pwm_dev2_name, pwm_dev2_name, RT_NAME_MAX);
    motor->pwm_channel1 = channel1;
    motor->pwm_channel2 = channel2;
    motor->pwm_period = pwm_period;
    motor->speed = 0;
    motor->mode = MOTOR_MODE_STOP;
    
    /* 查找PWM设备 */
    motor->pwm_dev1 = (struct rt_device_pwm *)rt_device_find(motor->pwm_dev1_name);
    if (motor->pwm_dev1 == RT_NULL)
    {
        LOG_E("PWM device %s not found!", motor->pwm_dev1_name);
        return -RT_ERROR;
    }
    
    motor->pwm_dev2 = (struct rt_device_pwm *)rt_device_find(motor->pwm_dev2_name);
    if (motor->pwm_dev2 == RT_NULL)
    {
        LOG_E("PWM device %s not found!", motor->pwm_dev2_name);
        return -RT_ERROR;
    }
    
    /* 设置PWM输出为0，停止电机 */
    rt_pwm_set(motor->pwm_dev1, motor->pwm_channel1, motor->pwm_period, 0);
    rt_pwm_set(motor->pwm_dev2, motor->pwm_channel2, motor->pwm_period, 0);
    
    /* 使能PWM输出 */
    rt_pwm_enable(motor->pwm_dev1, motor->pwm_channel1);
    rt_pwm_enable(motor->pwm_dev2, motor->pwm_channel2);
    
    LOG_I("Motor driver initialized with PWM freq: %d Hz (period: %d ns)", 
          pwm_freq_hz, pwm_period);
    
    return RT_EOK;
}

/*
 * 设置电机速度
 * 
 * @param motor 电机设备结构体指针
 * @param speed 电机速度(-100~100)，正值正转，负值反转，0停止
 * 
 * @return 0成功，否则失败
 */
rt_err_t motor_set_speed(struct motor_device *motor, rt_int16_t speed)
{
    RT_ASSERT(motor != RT_NULL);
    
    /* 限制速度范围在-100~100之间 */
    if (speed > 100)
        speed = 100;
    else if (speed < -100)
        speed = -100;
    
    /* 记录当前速度 */
    motor->speed = speed;
    
    /* 计算PWM占空比 */
    rt_uint32_t pulse;
    
    if (speed > 0)
    {
        /* 正转: IN1=PWM, IN2=0 */
        pulse = (rt_uint32_t)((motor->pwm_period * speed) / 100);
        
        /* 确保先关闭反向PWM再启用正向PWM */
//        rt_pwm_set(motor->pwm_dev2, motor->pwm_channel2, motor->pwm_period, motor->pwm_period);
//        rt_pwm_set(motor->pwm_dev1, motor->pwm_channel1, motor->pwm_period, motor->pwm_period - pulse);
		
		rt_pwm_set(motor->pwm_dev1, motor->pwm_channel1, motor->pwm_period, motor->pwm_period);
        rt_pwm_set(motor->pwm_dev2, motor->pwm_channel2, motor->pwm_period, motor->pwm_period - pulse);
        
        /* 确保两个通道的PWM都已使能 */
        rt_pwm_enable(motor->pwm_dev2, motor->pwm_channel2);
        rt_pwm_enable(motor->pwm_dev1, motor->pwm_channel1);
        
        motor->mode = MOTOR_MODE_FORWARD;
        LOG_D("Motor forward, speed: %d, pulse: %d", speed, pulse);
    }
    else if (speed < 0)
    {
        /* 反转: IN1=0, IN2=PWM */
        pulse = (rt_uint32_t)((motor->pwm_period * (-speed)) / 100);
        
        /* 确保先关闭正向PWM再启用反向PWM */
//        rt_pwm_set(motor->pwm_dev1, motor->pwm_channel1, motor->pwm_period, motor->pwm_period);
//        rt_pwm_set(motor->pwm_dev2, motor->pwm_channel2, motor->pwm_period, motor->pwm_period - pulse);
		
		rt_pwm_set(motor->pwm_dev2, motor->pwm_channel2, motor->pwm_period, motor->pwm_period);
		rt_pwm_set(motor->pwm_dev1, motor->pwm_channel1, motor->pwm_period, motor->pwm_period - pulse);
        
        /* 确保两个通道的PWM都已使能 */
        rt_pwm_enable(motor->pwm_dev1, motor->pwm_channel1);
        rt_pwm_enable(motor->pwm_dev2, motor->pwm_channel2);
        
        motor->mode = MOTOR_MODE_REVERSE;
        LOG_D("Motor reverse, speed: %d, pulse: %d", speed, pulse);
    }
    else
    {

        /* 停止: 两个PWM都输出0 */
        rt_pwm_set(motor->pwm_dev1, motor->pwm_channel1, motor->pwm_period,  motor->pwm_period);
        rt_pwm_set(motor->pwm_dev2, motor->pwm_channel2, motor->pwm_period,  motor->pwm_period);
		
		 /* 确保两个通道的PWM都已使能 */
        rt_pwm_enable(motor->pwm_dev1, motor->pwm_channel1);
        rt_pwm_enable(motor->pwm_dev2, motor->pwm_channel2);
        
        motor->mode = MOTOR_MODE_STOP;
        LOG_D("Motor stop");
    }
    
    return RT_EOK;
}


/**
 * 获取当前电机速度
 * 
 * @param motor 电机设备结构体指针
 * 
 * @return 当前电机速度(-100~100)
 */
rt_int16_t motor_get_speed(struct motor_device *motor)
{
    RT_ASSERT(motor != RT_NULL);
    
    return motor->speed;
}

/**
 * 停止电机（让电机自由滑行）
 * 
 * @param motor 电机设备结构体指针
 * 
 * @return 0成功，否则失败
 */
rt_err_t motor_stop(struct motor_device *motor)
{
    RT_ASSERT(motor != RT_NULL);
    
    return motor_set_speed(motor, 0);
}

/**
 * 电机制动（快速刹车）
 * 此模式下对电机施加反电动势，快速停止电机
 * DRV8870通过IN1和IN2都设为高电平实现刹车
 * 
 * @param motor 电机设备结构体指针
 * 
 * @return 0成功，否则失败
 */
rt_err_t motor_brake(struct motor_device *motor)
{
    RT_ASSERT(motor != RT_NULL);
    
    /* 刹车模式：两个PWM都输出100% */
    rt_pwm_set(motor->pwm_dev1, motor->pwm_channel1, motor->pwm_period, motor->pwm_period);
    rt_pwm_set(motor->pwm_dev2, motor->pwm_channel2, motor->pwm_period, motor->pwm_period);
    
    /* 更新状态 */
    motor->speed = 0;
    motor->mode = MOTOR_MODE_BRAKE;
    
    LOG_D("Motor brake activated");
    
    return RT_EOK;
}

/**
 * 获取当前运行模式
 * 
 * @param motor 电机设备结构体指针
 * 
 * @return 当前运行模式
 */
motor_mode_t motor_get_mode(struct motor_device *motor)
{
    RT_ASSERT(motor != RT_NULL);
    
    return motor->mode;
}

/* 导出为MSH命令的测试函数 */
#ifdef RT_USING_FINSH
#include <finsh.h>

/* 为演示定义的全局电机对象 */
static struct motor_device test_motor;
static rt_bool_t motor_initialized = RT_FALSE;

/* 电机初始化命令 */
static int motor_init_cmd(int argc, char **argv)
{
    if (argc != 6)
    {
        rt_kprintf("Usage: motor_init pwm1_dev chan1 pwm2_dev chan2 freq_hz\n");
        rt_kprintf("Example: motor_init pwm1 1 pwm1 2 20000\n");
        return -1;
    }

    if (motor_initialized)
    {
        rt_kprintf("Motor already initialized\n");
        return 0;
    }

    int chan1 = atoi(argv[2]);
    int chan2 = atoi(argv[4]);
    int freq = atoi(argv[5]);

    rt_err_t ret = motor_init(&test_motor, argv[1], chan1, argv[3], chan2, freq);
    if (ret == RT_EOK)
    {
        motor_initialized = RT_TRUE;
        rt_kprintf("Motor initialized successfully\n");
        return 0;
    }
    else
    {
        rt_kprintf("Motor initialization failed: %d\n", ret);
        return -1;
    }
}

/* 电机运行命令 */
static int motor_run_cmd(int argc, char **argv)
{
    if (!motor_initialized)
    {
        rt_kprintf("Motor not initialized, please run motor_init first\n");
        return -1;
    }

    if (argc != 2)
    {
        rt_kprintf("Usage: motor_run speed\n");
        rt_kprintf("Speed range: -100 to 100, 0 to stop\n");
        rt_kprintf("Example: motor_run 50\n");
        return -1;
    }

    int speed = atoi(argv[1]);
    motor_set_speed(&test_motor, speed);
    rt_kprintf("Motor set to speed %d\n", speed);
    return 0;
}

/* 电机刹车命令 */
static int motor_brake_cmd(void)
{
    if (!motor_initialized)
    {
        rt_kprintf("Motor not initialized, please run motor_init first\n");
        return -1;
    }

    motor_brake(&test_motor);
    rt_kprintf("Motor brake activated\n");
    return 0;
}

/* 电机状态查询命令 */
static int motor_status_cmd(void)
{
    if (!motor_initialized)
    {
        rt_kprintf("Motor not initialized, please run motor_init first\n");
        return -1;
    }

    rt_kprintf("Motor status:\n");
    rt_kprintf("  Speed: %d\n", motor_get_speed(&test_motor));
    
    switch (motor_get_mode(&test_motor))
    {
        case MOTOR_MODE_STOP:
            rt_kprintf("  Mode: STOP\n");
            break;
        case MOTOR_MODE_FORWARD:
            rt_kprintf("  Mode: FORWARD\n");
            break;
        case MOTOR_MODE_REVERSE:
            rt_kprintf("  Mode: REVERSE\n");
            break;
        case MOTOR_MODE_BRAKE:
            rt_kprintf("  Mode: BRAKE\n");
            break;
        default:
            rt_kprintf("  Mode: UNKNOWN\n");
            break;
    }
    
    return 0;
}

/**
 * 电机驱动完整测试函数
 * 测试包括:
 * 1. 初始化电机
 * 2. 正转加速和减速
 * 3. 紧急刹车功能
 * 4. 反转加速和减速
 * 5. 自由停止功能
 */
void motor_test(int argc, char *argv[])
{
    struct motor_device test_motor;
    const char *pwm1_name = "pwm1";
    const char *pwm2_name = "pwm1";
    rt_uint32_t pwm1_channel = 3;
    rt_uint32_t pwm2_channel = 4;
    rt_uint32_t pwm_freq = 50000;  /* 50kHz */
    rt_int16_t speed;
    rt_err_t ret;
    
    /* 解析命令行参数(如果提供) */
    if (argc >= 3) {
        pwm1_name = argv[1];
        pwm1_channel = atoi(argv[2]);
    }
    
    if (argc >= 5) {
        pwm2_name = argv[3];
        pwm2_channel = atoi(argv[4]);
    }
    
    if (argc >= 6) {
        pwm_freq = atoi(argv[5]);
    }
    
    rt_kprintf("Motor Test Program\n");
    rt_kprintf("------------------\n");
    rt_kprintf("PWM1: %s channel %d\n", pwm1_name, pwm1_channel);
    rt_kprintf("PWM2: %s channel %d\n", pwm2_name, pwm2_channel);
    rt_kprintf("Frequency: %d Hz\n", pwm_freq);
    rt_kprintf("------------------\n\n");
    
    /* 1. 初始化电机 */
    rt_kprintf("[Test] Initializing motor...\n");
    ret = motor_init(&test_motor, pwm1_name, pwm1_channel, pwm2_name, pwm2_channel, pwm_freq);
    if (ret != RT_EOK) {
        rt_kprintf("Motor initialization failed! Error: %d\n", ret);
        return;
    }
    rt_kprintf("Motor initialized successfully.\n\n");
    
    /* 2. 测试正转加速和减速 */
    rt_kprintf("[Test] Forward acceleration and deceleration test\n");
    
    /* 正转加速 */
    rt_kprintf("Accelerating forward...\n");
    for (speed = 0; speed <= 100; speed += 10) {
        motor_set_speed(&test_motor, speed);
        rt_kprintf("Speed: %d%%\n", speed);
        rt_thread_mdelay(300);
    }
    
    /* 维持最大速度一段时间 */
    rt_thread_mdelay(1000);
    
    /* 正转减速 */
    rt_kprintf("Decelerating...\n");
    for (speed = 100; speed >= 0; speed -= 10) {
        motor_set_speed(&test_motor, speed);
        rt_kprintf("Speed: %d%%\n", speed);
        rt_thread_mdelay(200);
    }
    
    rt_thread_mdelay(500);
    
    /* 3. 测试紧急刹车功能 */
    rt_kprintf("\n[Test] Emergency brake test\n");
    rt_kprintf("Accelerating to 70%%...\n");
    for (speed = 0; speed <= 70; speed += 10) {
        motor_set_speed(&test_motor, speed);
        rt_thread_mdelay(100);
    }
    rt_kprintf("Speed: 70%%\n");
    rt_thread_mdelay(1000);
    
    rt_kprintf("Applying emergency brake...\n");
    motor_brake(&test_motor);
    rt_thread_mdelay(2000);
    
    /* 4. 测试反转加速和减速 */
    rt_kprintf("\n[Test] Reverse acceleration and deceleration test\n");
    
    /* 反转加速 */
    rt_kprintf("Accelerating reverse...\n");
    for (speed = 0; speed >= -100; speed -= 10) {
        motor_set_speed(&test_motor, speed);
        rt_kprintf("Speed: %d%%\n", speed);
        rt_thread_mdelay(300);
    }
    
    /* 维持最大速度一段时间 */
    rt_thread_mdelay(1000);
    
    /* 反转减速 */
    rt_kprintf("Decelerating...\n");
    for (speed = -100; speed <= 0; speed += 10) {
        motor_set_speed(&test_motor, speed);
        rt_kprintf("Speed: %d%%\n", speed);
        rt_thread_mdelay(200);
    }
    
    rt_thread_mdelay(500);
    
    /* 5. 测试正反快速切换和刹车 */
    rt_kprintf("\n[Test] Direction change and brake response test\n");
    
    /* 正转 */
    rt_kprintf("Forward 50%%...\n");
    motor_set_speed(&test_motor, 50);
    rt_thread_mdelay(1500);
    
    /* 快速反转 */
    rt_kprintf("Quick reverse to -50%%...\n");
    motor_set_speed(&test_motor, -50);
    rt_thread_mdelay(1500);
    
    /* 刹车 */
    rt_kprintf("Brake!\n");
    motor_brake(&test_motor);
    rt_thread_mdelay(1000);
    
    /* 6. 结束测试，停止电机 */
    rt_kprintf("\n[Test] Test completed, stopping motor\n");
    motor_stop(&test_motor);
    
    rt_kprintf("\nMotor Test Summary:\n");
    rt_kprintf("------------------\n");
    rt_kprintf("Final motor state: %s\n", 
               motor_get_mode(&test_motor) == MOTOR_MODE_STOP ? "STOPPED" : 
               (motor_get_mode(&test_motor) == MOTOR_MODE_BRAKE ? "BRAKED" : "UNKNOWN"));
    rt_kprintf("All tests completed!\n");
}
MSH_CMD_EXPORT_ALIAS(motor_test, motor_test, Motor driver comprehensive test [pwm1 ch1 pwm2 ch2 freq]);


/* 注册MSH命令 */
MSH_CMD_EXPORT_ALIAS(motor_init_cmd, motor_init, Initialize motor driver);
MSH_CMD_EXPORT_ALIAS(motor_run_cmd, motor_run, Set motor speed (-100 to 100));
MSH_CMD_EXPORT_ALIAS(motor_brake_cmd, motor_brake, Activate motor brake);
MSH_CMD_EXPORT_ALIAS(motor_status_cmd, motor_status, Get motor status);


#endif /* RT_USING_FINSH */

