//
// Created by 26570 on 25-3-28.
//

#ifndef DRV_SEG_H
#define DRV_SEG_H

#include<rtthread.h>
#include "drv_gpio.h"

// 设置显示的数字和小数点位置
void seg_set_display_data(int data, uint8_t decimal_pos);

#endif //DRV_SEG_H
