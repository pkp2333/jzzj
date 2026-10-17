/* chassis.c */
#include <rtthread.h>
#include <math.h>
#include "chassis.h"
#include "stdlib.h"

#define DBG_TAG             "chsassis"
#define DBG_LVL             DBG_LOG
#include <rtdbg.h>

#define M_PI 3.1415926
#ifndef RT_CLAMP
#define RT_CLAMP(v, min, max)   ((v) < (min) ? (min) : ((v) > (max) ? (max) : (v)))
#endif


#define PWM_FREQ         50000    // 50kHz PWM频率
#define CONTROL_INTERVAL 20       // 20ms控制周期
#define MAX_SPEED        100      // 最大速度百分比

/* 四轮电机结构体 */
static struct motor_device lf_motor, rf_motor, lb_motor, rb_motor;

/* 带限幅保护的速度设置 */
static void motor_set_clamped(struct motor_device* motor, int speed)
{
    speed = RT_CLAMP(speed, -MAX_SPEED, MAX_SPEED);
    motor_set_speed(motor, speed);
}

/* 麦克纳姆轮运动学解算 */
static void mecanum_transform(int vx, int vy, int wz, int* speeds)
{
    // 运动学模型系数（根据实际车体尺寸调整）
    const float L = 0.15f;   // 车体中心到轮子的纵向距离（米）
    const float W = 0.15f;  // 车体中心到轮子的横向距离（米）
    const float R = 0.04f; // 轮子半径（米）
	
	// 核心公式（X型布局）
    speeds[0] =  (int)(vx - vy + (L + W)*wz); // LF
    speeds[1] =  (int)(vx + vy - (L + W)*wz); // RF
    speeds[2] =  (int)(vx + vy + (L + W)*wz); // LB
    speeds[3] =  (int)(vx - vy - (L + W)*wz); // RB

    /* 归一化处理 */
    int max_val = 0;
    for (int i = 0; i < 4; i++) {
        int abs_speed = abs(speeds[i]);
        max_val = abs_speed > max_val ? abs_speed : max_val;
    }

    if (max_val > MAX_SPEED) {
        float ratio = (float)MAX_SPEED / max_val;
        for (int i = 0; i < 4; i++) {
            speeds[i] = (int)(speeds[i] * ratio);
        }
    }
}

/* 底盘初始化 */
static rt_err_t chassis_hw_init(void)
{
    rt_err_t ret;

	/* 初始化右后轮（PWM1通道3&4） */
    if ((ret = motor_init(&rb_motor, "pwm1", 3, "pwm1", 4, PWM_FREQ)) != RT_EOK) {
        LOG_E("[CHASSIS] RB motor init failed");
        return ret;
    }
	
	/* 初始化右前轮（PWM1通道1&2） */
    if ((ret = motor_init(&rf_motor, "pwm1", 1, "pwm1", 2, PWM_FREQ)) != RT_EOK) {
        LOG_E("[CHASSIS] RF motor init failed");
        return ret;
    }
	
	/* 初始化左后轮（PWM9通道1&2） */
    if ((ret = motor_init(&lb_motor, "pwm9", 1, "pwm9", 2, PWM_FREQ)) != RT_EOK) {
        LOG_E("[CHASSIS] LB motor init failed");
        return ret;
    }
	
    /* 初始化左前轮（PWM10通道1& PWM11通道1） */
    if ((ret = motor_init(&lf_motor, "pwm10", 1, "pwm11", 1, PWM_FREQ)) != RT_EOK) {
        LOG_E("[CHASSIS] LF motor init failed");
        return ret;
    }

    LOG_I("[CHASSIS] All motors ready");
    return RT_EOK;
}


hostTypeDef cmd = {0};
rt_bool_t lock = 1;
	rt_bool_t vel_control_flag = 0;
/* 底盘控制线程 */
static void chassis_thread_entry(void *param)
{
	
    rt_thread_mdelay(1200);  // 等待系统稳定
    if (chassis_hw_init() != RT_EOK) return;
	
    int wheel_speeds[4] = {0};
    
    while (1) {
        rt_mutex_take(&host_mutex, RT_WAITING_FOREVER);
		cmd = host;// 全局控制指令
		rt_mutex_release(&host_mutex);

		if (cmd.mode == 0) lock = 1;  // 上锁状态
		else if (cmd.mode != 0) lock = 0;// 解锁
		
		if (cmd.mode == 2) vel_control_flag = 1;
		else vel_control_flag = 0;
		
		if (!lock)
		{
			if (vel_control_flag == 1)
			{
				/* 运动学解算 */
				mecanum_transform(cmd.x, cmd.y, cmd.Raw, wheel_speeds);

				/* 驱动电机 */
				motor_set_clamped(&lf_motor, -wheel_speeds[0]);
				motor_set_clamped(&rf_motor, wheel_speeds[1]);
				motor_set_clamped(&lb_motor, -wheel_speeds[2]);
				motor_set_clamped(&rb_motor, wheel_speeds[3]);
				
				/* 状态反馈 */
				LOG_D("[CHASSIS] LF:%4d RF:%4d LB:%4d RB:%4d\n",
						 wheel_speeds[0], wheel_speeds[1],
						 wheel_speeds[2], wheel_speeds[3]);
			}
			else 
			{
				motor_brake(&lf_motor);
				motor_brake(&rf_motor);
				motor_brake(&lb_motor);
				motor_brake(&rb_motor);
				LOG_D("急停\r\n");
			}
			
		}
		else
		{
			/* 驱动电机 */
			wheel_speeds[0] =  0; // LF
			wheel_speeds[1] =  0; // RF
			wheel_speeds[2] =  0; // LB
			wheel_speeds[3] =  0; // RB
			motor_set_clamped(&lf_motor, -wheel_speeds[0]);
			motor_set_clamped(&rf_motor, wheel_speeds[1]);
			motor_set_clamped(&lb_motor, -wheel_speeds[2]);
			motor_set_clamped(&rb_motor, wheel_speeds[3]);
			LOG_D("上位机未解锁底盘控制\r\n");
		}
        rt_thread_mdelay(CONTROL_INTERVAL);
    }
}

/* 底盘模块初始化 */
int chassis_init(void)
{
    rt_thread_t thread = rt_thread_create("chassis_ctrl",
                                        chassis_thread_entry,
                                        RT_NULL,
                                        4096,
                                        8,  // 较高优先级
                                        10);
    if (thread) {
        rt_thread_startup(thread);
        return RT_EOK;
    }
    return RT_ERROR;
}
INIT_APP_EXPORT(chassis_init);
