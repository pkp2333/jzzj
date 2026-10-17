//
// Created by 26570 on 25-3-28.
//

#include "battery_info.h"
#include "drv_seg.h"

#define DBG_TAG "battery_info"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define ADC_DEV_NAME        "adc1"
#define ADC_DEV_CHANNEL     3
#define REFER_VOLTAGE       345//330
#define CONVERT_BITS        4096
#define DIVISION_FACTOR     5

/*线程定义*/
rt_thread_t battary_adc_tid = RT_NULL;

static void battery_voltage_display_thread(void *parameter)
{

    rt_adc_device_t barrtery_adc_dev;
    rt_uint32_t value, vol;
    rt_err_t ret = RT_EOK;

    //查找设备
    barrtery_adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (barrtery_adc_dev == RT_NULL)
    {
        rt_kprintf("No Device");
    }

    ret = rt_adc_enable(barrtery_adc_dev, ADC_DEV_CHANNEL);

    if(ret == RT_EOK){
        rt_kprintf("Success,Device name is:%s", ADC_DEV_NAME);
    }

    while(1){
        //读取采样值
        value=rt_adc_read(barrtery_adc_dev, ADC_DEV_CHANNEL);

        //转换
        vol=value*REFER_VOLTAGE * DIVISION_FACTOR /CONVERT_BITS;
        seg_set_display_data(vol, 2);

        rt_kprintf("the voltage is :%d.%02d \n", vol / 100, vol % 100);
        rt_thread_delay(500);
    }
}




int battery_disp_thread_init(void)
{

    battary_adc_tid = rt_thread_create("battery", battery_voltage_display_thread, RT_NULL, 1024, 5, 10);

    if(battary_adc_tid !=RT_NULL ){
        rt_thread_startup(battary_adc_tid);
    }

    return RT_EOK;
}

INIT_APP_EXPORT(battery_disp_thread_init);


