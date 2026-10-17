#ifndef __BEEP_H
#define __BEEP_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define BEEP_QUEUE_SIZE 8   // 消息队列大小

// 定义事件类型
typedef enum
{
    BEEP_EVENT_STARTUP,   // 启动提示
    BEEP_EVENT_WARNING,   // 警告提示
    BEEP_EVENT_ERROR,     // 错误提示
} beep_event_t;

typedef struct
{
    int beep_count;       // 响声次数
    int beep_on_duration; // 每次响声持续时间（高低电平切换次数）
    int beep_off_duration;// 每次静音持续时间（高低电平切换次数）
} beep_mode_t;

// 声音模式配置
static beep_mode_t beep_modes[] = {
    [BEEP_EVENT_STARTUP] = {3, 20, 50},   // 启动：1次，响声持续20次切换，无静音
    [BEEP_EVENT_WARNING] = {3, 20, 30}, // 警告：3次，响声持续20次切换，静音30次
    [BEEP_EVENT_ERROR]   = {5, 30, 40}, // 错误：5次，响声持续30次切换，静音40次
};

typedef struct
{
	rt_base_t beep_pin;
}beep_obj_t;
	
void beep_trigger_event(beep_event_t event);	

#endif
