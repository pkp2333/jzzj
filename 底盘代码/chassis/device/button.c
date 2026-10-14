#include "button.h"

flex_button_t * user_button_init(rt_base_t pin, void *read, flex_button_response_callback *callback)
{
    flex_button_t *obj = (flex_button_t *)rt_malloc(sizeof(flex_button_t));
    rt_memset(obj, 0, sizeof(flex_button_t));

	obj->usr_button_read = read;
    obj->cb = (flex_button_response_callback)callback;


    rt_pin_mode(pin, PIN_MODE_INPUT); /* set KEY pin mode to input */

	obj->pressed_logic_level = 0;
	obj->click_start_tick = 20;
	obj->short_press_start_tick = 100;
	obj->long_press_start_tick = 200;
	obj->long_hold_start_tick = 300;


	flex_button_register(obj);
	return obj;
}

#define TEST_BUTTON

#ifdef TEST_BUTTON
/*使用方法：需要创建一个线程，处理所有按键扫描*/
// s1：宏定义按键引脚
// s2：定义 flex_button_t 结构体变量
// s3：定义回调函数（在回调函数中根据按键状态处理）
// s4: 封装button_read函数
// s5：初始化按键


// s1：宏定义按键引脚
#define PIN_KEY0	GET_PIN(A, 4)
#define PIN_KEY1	GET_PIN(A, 5)
#define PIN_KEY2	GET_PIN(C, 4) 

// s2：定义 flex_button_t 结构体变量
static flex_button_t *user_button_1;
static flex_button_t *user_button_2;
static flex_button_t *user_button_3;

// s3：定义回调函数
static void btn_0_cb(flex_button_t *btn)
{
    rt_kprintf("btn_0_cb\n");
    switch (btn->event)
    {
        case FLEX_BTN_PRESS_DOWN:
            rt_kprintf("btn_0_cb [FLEX_BTN_PRESS_DOWN]\n");
            break;
        case FLEX_BTN_PRESS_CLICK:
            rt_kprintf("btn_0_cb [FLEX_BTN_PRESS_CLICK]\n");
            break;
        case FLEX_BTN_PRESS_DOUBLE_CLICK:
            rt_kprintf("btn_0_cb [FLEX_BTN_PRESS_DOUBLE_CLICK]\n");
            break;
        case FLEX_BTN_PRESS_SHORT_START:
            rt_kprintf("btn_0_cb [FLEX_BTN_PRESS_SHORT_START]\n");
            break;
        case FLEX_BTN_PRESS_SHORT_UP:
            rt_kprintf("btn_0_cb [FLEX_BTN_PRESS_SHORT_UP]\n");
            break;
        case FLEX_BTN_PRESS_LONG_START:
            rt_kprintf("btn_0_cb [FLEX_BTN_PRESS_LONG_START]\n");
            break;
        case FLEX_BTN_PRESS_LONG_UP:
            rt_kprintf("btn_0_cb [FLEX_BTN_PRESS_LONG_UP]\n");
            break;
        case FLEX_BTN_PRESS_LONG_HOLD:
            rt_kprintf("btn_0_cb [FLEX_BTN_PRESS_LONG_HOLD]\n");
            break;
        case FLEX_BTN_PRESS_LONG_HOLD_UP:
            rt_kprintf("btn_0_cb [FLEX_BTN_PRESS_LONG_HOLD_UP]\n");
            break;
    }
}

static void btn_1_cb(flex_button_t *btn)
{
    rt_kprintf("btn_1_cb\n");
    switch (btn->event)
    {
        case FLEX_BTN_PRESS_DOWN:
            rt_kprintf("btn_1_cb [FLEX_BTN_PRESS_DOWN]\n");
            break;
        case FLEX_BTN_PRESS_CLICK:
            rt_kprintf("btn_1_cb [FLEX_BTN_PRESS_CLICK]\n");
            break;
        case FLEX_BTN_PRESS_DOUBLE_CLICK:
            rt_kprintf("btn_1_cb [FLEX_BTN_PRESS_DOUBLE_CLICK]\n");
            break;
        case FLEX_BTN_PRESS_SHORT_START:
            rt_kprintf("btn_1_cb [FLEX_BTN_PRESS_SHORT_START]\n");
            break;
        case FLEX_BTN_PRESS_SHORT_UP:
            rt_kprintf("btn_1_cb [FLEX_BTN_PRESS_SHORT_UP]\n");
            break;
        case FLEX_BTN_PRESS_LONG_START:
            rt_kprintf("btn_1_cb [FLEX_BTN_PRESS_LONG_START]\n");
            break;
        case FLEX_BTN_PRESS_LONG_UP:
            rt_kprintf("btn_1_cb [FLEX_BTN_PRESS_LONG_UP]\n");
            break;
        case FLEX_BTN_PRESS_LONG_HOLD:
            rt_kprintf("btn_1_cb [FLEX_BTN_PRESS_LONG_HOLD]\n");
            break;
        case FLEX_BTN_PRESS_LONG_HOLD_UP:
            rt_kprintf("btn_1_cb [FLEX_BTN_PRESS_LONG_HOLD_UP]\n");
            break;
    }
}

static void btn_2_cb(flex_button_t *btn)
{
    rt_kprintf("btn_2_cb\n");
    switch (btn->event)
    {
        case FLEX_BTN_PRESS_DOWN:
            rt_kprintf("btn_2_cb [FLEX_BTN_PRESS_DOWN]\n");
            break;
        case FLEX_BTN_PRESS_CLICK:
            rt_kprintf("btn_2_cb [FLEX_BTN_PRESS_CLICK]\n");
            break;
        case FLEX_BTN_PRESS_DOUBLE_CLICK:
            rt_kprintf("btn_2_cb [FLEX_BTN_PRESS_DOUBLE_CLICK]\n");
            break;
        case FLEX_BTN_PRESS_SHORT_START:
            rt_kprintf("btn_2_cb [FLEX_BTN_PRESS_SHORT_START]\n");
            break;
        case FLEX_BTN_PRESS_SHORT_UP:
            rt_kprintf("btn_2_cb [FLEX_BTN_PRESS_SHORT_UP]\n");
            break;
        case FLEX_BTN_PRESS_LONG_START:
            rt_kprintf("btn_2_cb [FLEX_BTN_PRESS_LONG_START]\n");
            break;
        case FLEX_BTN_PRESS_LONG_UP:
            rt_kprintf("btn_2_cb [FLEX_BTN_PRESS_LONG_UP]\n");
            break;
        case FLEX_BTN_PRESS_LONG_HOLD:
            rt_kprintf("btn_2_cb [FLEX_BTN_PRESS_LONG_HOLD]\n");
            break;
        case FLEX_BTN_PRESS_LONG_HOLD_UP:
            rt_kprintf("btn_2_cb [FLEX_BTN_PRESS_LONG_HOLD_UP]\n");
            break;
    }
}

// s4: 封装button_read函数
static uint8_t button_key0_read(void)
{
    return rt_pin_read(PIN_KEY0);
}

static uint8_t button_key1_read(void)
{
    return rt_pin_read(PIN_KEY1);
}

static uint8_t button_key2_read(void)
{
    return rt_pin_read(PIN_KEY2);
}

static void button_scan(void *arg)
{
    while(1)
    {
        flex_button_scan();
        rt_thread_mdelay(20);
    }
}

int flex_button_main(void)
{
    rt_thread_t tid = RT_NULL;

	// s5：初始化按键
    user_button_1 = user_button_init(PIN_KEY0, button_key0_read, (flex_button_response_callback*)btn_0_cb);
	user_button_2 = user_button_init(PIN_KEY1, button_key1_read, (flex_button_response_callback*)btn_1_cb);
	user_button_3 = user_button_init(PIN_KEY2, button_key2_read, (flex_button_response_callback*)btn_2_cb);

    /* Create background ticks thread */
    tid = rt_thread_create("flex_btn", button_scan, RT_NULL, 1024, 10, 10);
    if(tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }

    return 0;
}
#ifdef FINSH_USING_MSH
INIT_APP_EXPORT(flex_button_main);
#endif

#endif

