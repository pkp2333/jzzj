#ifndef __GRAY_SENSOR_H
#define __GRAY_SENSOR_H

#include <rtthread.h>
#include <stdint.h>
#include <rtdevice.h>
#include <board.h>


#define PCF8591_ADDR_1       0x48    // PCF8591 I2C 7 位地址
#define PCF8591_ADDR_2       0x49    // PCF8591 I2C 7 位地址
#define I2C_BUS_NAME       "i2c2"  // I2C 设备名称
#define ANALOG_CHANNEL_0   0x40    // 控制字节：选择 AIN0（单端输入）

typedef struct 
{
	rt_uint8_t val[7];
}gray_sensor_t;


#endif
