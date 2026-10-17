#ifndef __MOTOR_PWM_H
#define __MOTOR_PWM_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>


typedef struct 
{
	struct rt_device_pwm *pwm_dev_1; /* PWM设 备 句 柄 */
	rt_uint8_t channel_1;
	struct rt_device_pwm *pwm_dev_2; /* PWM设 备 句 柄 */
	rt_uint8_t channel_2;
}motor_pwm_obj_t;

motor_pwm_obj_t *motor_pwm_init(char *pwm_name_1, rt_uint8_t channel_1, 
								char *pwm_name_2, rt_uint8_t channel_2);

rt_err_t motor_speed_set(motor_pwm_obj_t *motor, rt_int16_t speed);    // speed : 0-100

#endif
