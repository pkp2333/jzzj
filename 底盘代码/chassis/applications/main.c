/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2020-07-13     Dozingfiretruck   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define LED_PIN_G    GET_PIN(F, 6)
#define LED_PIN_R	 GET_PIN(F, 7)
#define LED_PIN_B    GET_PIN(F, 8)


// 如果使用fal+easyflash 需要包含这两个头文件
#include "fal.h"
#include "easyflash.h"

INIT_APP_EXPORT(fal_init);
INIT_APP_EXPORT(easyflash_init);
// 如果使用 fal + easyflash 需要包含这两个头文件



int main(void)
{
	
    /* set RGB pin mode to output */
    rt_pin_mode(LED_PIN_G, PIN_MODE_OUTPUT);
	rt_pin_mode(LED_PIN_R, PIN_MODE_OUTPUT);
	rt_pin_mode(LED_PIN_B, PIN_MODE_OUTPUT);
	
    while (1)
    {

        rt_pin_write(LED_PIN_G, PIN_LOW);
		rt_pin_write(LED_PIN_R, PIN_HIGH);
		rt_pin_write(LED_PIN_B, PIN_HIGH);
        rt_thread_mdelay(100);
		rt_pin_write(LED_PIN_R, PIN_LOW);
        rt_pin_write(LED_PIN_B, PIN_HIGH);
		rt_pin_write(LED_PIN_G, PIN_HIGH);
        rt_thread_mdelay(100);
		rt_pin_write(LED_PIN_B, PIN_LOW);
		rt_pin_write(LED_PIN_G, PIN_HIGH);
		rt_pin_write(LED_PIN_R, PIN_HIGH);
		rt_thread_mdelay(100);
    }
}

