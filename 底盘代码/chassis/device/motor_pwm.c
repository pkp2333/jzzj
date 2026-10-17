#include "motor_pwm.h"

#include <stdlib.h>

//#define TEST_MOTOR 

rt_err_t motor_speed_set(motor_pwm_obj_t *motor, rt_int16_t speed)    // speed : 0-99
{
	if (speed > 0)
	{
		rt_pwm_set(motor->pwm_dev_1, motor->channel_1, 10000000, (100-abs(speed)) * 100000);
		rt_pwm_set(motor->pwm_dev_2, motor->channel_2, 10000000, 100 * 100000);
	}
	else if (speed < 0)
	{
		rt_pwm_set(motor->pwm_dev_2, motor->channel_2, 10000000, (100-abs(speed)) * 100000);
		rt_pwm_set(motor->pwm_dev_1, motor->channel_1, 10000000, 100 * 100000);
	}
	else
	{
		rt_pwm_set(motor->pwm_dev_2, motor->channel_2, 10000000, 100 * 100000);
		rt_pwm_set(motor->pwm_dev_1, motor->channel_1, 10000000, 100 * 100000);
	}
	return RT_EOK;
}


motor_pwm_obj_t *motor_pwm_init(char *pwm_name_1, rt_uint8_t channel_1, 
								char *pwm_name_2, rt_uint8_t channel_2)
{
	motor_pwm_obj_t *obj = (motor_pwm_obj_t *)rt_malloc(sizeof(motor_pwm_obj_t));
    rt_memset(obj, 0, sizeof(motor_pwm_obj_t));
	
	if (pwm_name_1 != RT_NULL)
	{
		obj->pwm_dev_1 = (struct rt_device_pwm*)rt_device_find(pwm_name_1);
	}
	
	if (pwm_name_2 != RT_NULL)
	{
		obj->pwm_dev_2 = (struct rt_device_pwm*)rt_device_find(pwm_name_2);
	}
	else 
	{
		obj->pwm_dev_2 = obj->pwm_dev_1;
	}
	obj->channel_1 = channel_1;
	obj->channel_2 = channel_2;
	
	rt_pwm_enable(obj->pwm_dev_1, obj->channel_1);
	rt_pwm_enable(obj->pwm_dev_2, obj->channel_2);	
	
	rt_pwm_set(obj->pwm_dev_1, obj->channel_1, 10000000, 0);
	rt_pwm_set(obj->pwm_dev_2, obj->channel_2, 10000000, 0);
	return obj;
}


#ifdef TEST_MOTOR
motor_pwm_obj_t *motor1;

rt_int16_t speed = 0;

static void test_motor_pwm_thread_entry(void *parameter)
{
	while (1)
	{
		motor_speed_set(motor1, speed);
		rt_thread_mdelay(100);
	}
}
	


int test_motor_pwm_thread_init(void)
{
	motor1 = motor_pwm_init("pwm1", 3, 
							"pwm1", 4);
	
	rt_thread_t t = rt_thread_create("m_pwm",
                             test_motor_pwm_thread_entry,
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
		rt_kprintf("create m_pwm thread failed\r\n");
	}
	return RT_EOK;
}


INIT_APP_EXPORT(test_motor_pwm_thread_init);
#endif 
