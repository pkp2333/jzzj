#ifndef __SERVO_PWM_H
#define __SERVO_PWM_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

typedef enum
{
    ANGLE_180_CONTROL,
    ANGLE_270_CONTROL,
    SPEED_CONTROL,
} servo_e;

typedef struct
{
    struct rt_device_pwm *pwm_dev; /* PWM设备句柄 */
    rt_uint8_t channel;

    servo_e type;

    rt_int16_t current_angle; /* 当前角度位置 */

    rt_err_t (*control)(void *self, rt_int16_t val);
} servo_pwm_obj_t;

servo_pwm_obj_t *servo_pwm_init(servo_e type, char *pwm_name, rt_uint8_t channel);
rt_err_t servo_control_with_time(servo_pwm_obj_t *servo, rt_int16_t target_val, rt_uint32_t time_ms); // 新增函数声明

#endif
