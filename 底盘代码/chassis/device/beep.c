#include "beep.h"

static rt_mq_t beep_mq       = RT_NULL;     // 消息队列句柄
static rt_timer_t beep_timer;
static volatile beep_event_t current_event; // 当前处理的事件


// 消息触发函数
void beep_trigger_event(beep_event_t event)
{
    if (beep_mq != RT_NULL) {
        rt_mq_send(beep_mq, &event, sizeof(beep_event_t)); // 向队列发送事件
    }
}

static void timer_cb(void *args)
{
	
	static int toggle_count = 0; // 当前高低电平切换次数
    static int state        = 0; // 状态机：0=发声，1=静音
    static int beep_cycle   = 0; // 当前滴声次数计数
	
	beep_obj_t *beep = (beep_obj_t *)args;

    // 获取当前事件的声音模式
    beep_mode_t *mode = &beep_modes[current_event];

    if (state == 0) // 发声阶段
    {
        rt_pin_write(beep->beep_pin, toggle_count % 2 == 0 ? PIN_HIGH : PIN_LOW);
        toggle_count++;

        if (toggle_count >= mode->beep_on_duration) // 发声持续时间结束
        {
            toggle_count = 0;
            state        = 1;                // 切换到静音阶段
            rt_pin_write(beep->beep_pin, PIN_LOW); // 确保静音
        }
    } else if (state == 1) // 静音阶段
    {
        toggle_count++;

        if (toggle_count >= mode->beep_off_duration) // 静音持续时间结束
        {
            toggle_count = 0;
            state        = 0; // 切换到发声阶段
            beep_cycle++;

            if (beep_cycle >= mode->beep_count) // 完成指定次数的滴声
            {
                toggle_count = 0;
                state        = 0;
                beep_cycle   = 0;
                rt_timer_stop(beep_timer); // 停止定时器
            }
        }
    }
}


// 消息处理函数
static void beep_thread_entry(void *parameter)
{
    beep_event_t event;
    while (1) {
        if (rt_mq_recv(beep_mq, &event, sizeof(beep_event_t), RT_WAITING_FOREVER) == RT_EOK) {
            current_event = event;
            rt_timer_start(beep_timer); // 启动定时器
        }
    }
}

beep_obj_t *beep_init(rt_base_t beep_pin)
{
	beep_obj_t *obj = (beep_obj_t *)rt_malloc(sizeof(beep_obj_t));
	rt_memset(obj, 0, sizeof(beep_obj_t));
	
	obj->beep_pin = beep_pin;
	
	beep_timer = rt_timer_create("beep_timer", timer_cb, obj, 5, RT_TIMER_FLAG_PERIODIC);
	
	rt_pin_mode(obj->beep_pin, PIN_MODE_OUTPUT);
    rt_pin_write(obj->beep_pin, PIN_LOW);
	
	// 创建消息队列
    beep_mq = rt_mq_create("beep_mq", sizeof(beep_event_t), BEEP_QUEUE_SIZE, RT_IPC_FLAG_FIFO);
    if (beep_mq == RT_NULL) {
        rt_kprintf("Failed to create beep message queue!\r\n");
    }
	
	    // 创建蜂鸣器线程
    rt_thread_t beep_thread = rt_thread_create("beep", beep_thread_entry, obj, 256, 10, 10);
    if (beep_thread != RT_NULL) {
        rt_thread_startup(beep_thread);
    } else {
        rt_kprintf("Failed to create beep thread!\r\n");
    }
	return obj;
}


//#define TEST_BEEP

#ifdef TEST_BEEP

beep_obj_t* beep;

int test_beep(void)
{
	beep = beep_init(GET_PIN(G, 14));
	
	beep_trigger_event(BEEP_EVENT_ERROR);
	return RT_EOK;
}

INIT_APP_EXPORT(test_beep);

#endif



