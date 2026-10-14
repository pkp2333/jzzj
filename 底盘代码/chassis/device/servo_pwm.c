#include "servo_pwm.h"

// 舵机控制函数
rt_err_t servo_control(void *args, rt_int16_t val)
{
    rt_err_t ret;

    servo_pwm_obj_t *servo = (servo_pwm_obj_t *)args;

    switch (servo->type)
    {
        case ANGLE_180_CONTROL:
        {
            rt_pwm_set(servo->pwm_dev, servo->channel, 20000000, (uint32_t)(val * 11110 + 500000));
            break;
        }
        case ANGLE_270_CONTROL:
        {
            rt_pwm_set(servo->pwm_dev, servo->channel, 20000000, (uint32_t)(val * 7410 + 500000));
            break;
        }
        case SPEED_CONTROL:
        {
            rt_pwm_set(servo->pwm_dev, servo->channel, 20000000, (uint32_t)(val * 11110 + 1500000));
            break;
        }
        default:
        {
            break;
        }
    }

    servo->current_angle = val; // 更新当前角度
    return ret;
}

// 新增：指定时间内运动到目标角度的函数
rt_err_t servo_control_with_time(servo_pwm_obj_t *servo, rt_int16_t target_val, rt_uint32_t time_ms)
{
    if (servo == RT_NULL || servo->pwm_dev == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_int16_t current_val = servo->current_angle; // 使用存储的当前角度
    rt_int16_t steps = time_ms / 20;              // 每 20ms 更新一次
    rt_int16_t step_val = 2*(target_val - current_val) / steps; // 每步的角度增量

    for (rt_uint32_t i = 0; i < steps; i++)
    {
        current_val += step_val;
        servo->control(servo, current_val); // 调用原始控制函数
        rt_thread_mdelay(20); // 每次更新后延时 20ms
    }

    // 确保最后到达目标值
    servo->control(servo, target_val);

    return RT_EOK;
}

// 初始化舵机
servo_pwm_obj_t *servo_pwm_init(servo_e type, char *pwm_name, rt_uint8_t channel)
{
    servo_pwm_obj_t *obj = (servo_pwm_obj_t *)rt_malloc(sizeof(servo_pwm_obj_t));
    rt_memset(obj, 0, sizeof(servo_pwm_obj_t));

    obj->pwm_dev = (struct rt_device_pwm *)rt_device_find(pwm_name);
    obj->type = type;
    obj->channel = channel;
	if (obj->type == ANGLE_180_CONTROL)
	{
		obj->current_angle = 90; // 初始化为中位角度
	}
	else if (obj->type == ANGLE_270_CONTROL)
	{
		obj->current_angle = 135; // 初始化为中位角度
	}
    
    obj->control = servo_control;

    rt_pwm_set(obj->pwm_dev, obj->channel, 20000000, 1500000); // 周期20ms 初始状态设置脉宽1.5ms 中位
    rt_pwm_enable(obj->pwm_dev, obj->channel);
    return obj;
}

//#define TEST_SERVO

#ifdef TEST_SERVO

servo_pwm_obj_t *servo1;

// 测试线程：调用带时间的控制函数
static void test_servo_pwm_thread_entry(void *parameter)
{
    while (1)
    {
        // 从当前位置移动到 180 度，耗时 2 秒
        servo_control_with_time(servo1, 180, 2000);
        rt_thread_mdelay(500);

        // 从 180 度移动到 0 度，耗时 3 秒
        servo_control_with_time(servo1, 0, 2000);
        rt_thread_mdelay(500);

        // 从 0 度移动到中位（90度），耗时 1 秒
        servo_control_with_time(servo1, 90, 2000);
        rt_thread_mdelay(500);
    }
}

// 初始化测试线程
int test_servo_pwm_thread_init(void)
{
    servo1 = servo_pwm_init(ANGLE_180_CONTROL, "pwm8", 1);

    rt_thread_t t = rt_thread_create("s_pwm",
                                     test_servo_pwm_thread_entry,
                                     RT_NULL,
                                     1024,
                                     25,
                                     5);

    if (t != RT_NULL)
    {
        rt_thread_startup(t);
    }
    else
    {
        rt_kprintf("create s_pwm thread failed\r\n");
    }

    return RT_EOK;
}

INIT_APP_EXPORT(test_servo_pwm_thread_init);

#endif
