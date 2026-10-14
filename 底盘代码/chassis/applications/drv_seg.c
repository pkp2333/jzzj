#include "drv_seg.h"
#include <rtthread.h>
#include "stm32f4xx_hal.h"

// 段码表，最后一位是带小数点的段码
unsigned char LED_0F[] = 
{// 0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F     -     .(单独)
    0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90,0x88,0x83,0xC6,0xA1,0x86,0x8E,0xBF,0x7F,
    // 0.    1.    2.    3.    4.    5.    6.    7.    8.    9.    (带小数点的数字)
    0x40,0x79,0x24,0x30,0x19,0x12,0x02,0x78,0x00,0x10
};

// 独立定义引脚和端口
#define SCLK_PIN GPIO_PIN_1
#define SCLK_PORT GPIOB
#define RCLK_PIN GPIO_PIN_0
#define RCLK_PORT GPIOB
#define DIO_PIN  GPIO_PIN_5
#define DIO_PORT GPIOC

// 独立的引脚设置宏定义
#define SET_PIN(pin, port, state) HAL_GPIO_WritePin(port, pin, (state)? GPIO_PIN_SET : GPIO_PIN_RESET)
#define SCLK(state) SET_PIN(SCLK_PIN, SCLK_PORT, state)
#define RCLK(state) SET_PIN(RCLK_PIN, RCLK_PORT, state)
#define DIO(state) SET_PIN(DIO_PIN, DIO_PORT, state)

void digital_init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    // 使能各个引脚所在端口的时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

    // 初始化 SCLK 引脚
    GPIO_Initure.Pin = SCLK_PIN;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;  // 推挽输出
    GPIO_Initure.Pull = GPIO_PULLUP;          // 上拉
    GPIO_Initure.Speed = GPIO_SPEED_FAST;     // 快速
    HAL_GPIO_Init(SCLK_PORT, &GPIO_Initure);

    // 初始化 RCLK 引脚
    GPIO_Initure.Pin = RCLK_PIN;
    HAL_GPIO_Init(RCLK_PORT, &GPIO_Initure);

    // 初始化 DIO 引脚
    GPIO_Initure.Pin = DIO_PIN;
    HAL_GPIO_Init(DIO_PORT, &GPIO_Initure);
}    

void LED_OUT(uint8_t X)
{
    uint8_t i;
    for(i=8;i>=1;i--)
    {
        if (X&0x80) DIO(1); else DIO(0);
        X<<=1;
        SCLK(0);
        SCLK(1);
    }
}

void LED_display_distance(int data, uint8_t decimal_pos)    
{
    uint8_t digits[4]; // 存储4位数字
    uint8_t i;

    // 分离各位数字
    for(i = 0; i < 4; i++)
    {
        digits[i] = data % 10;
        data /= 10;
    }

    // 标记是否为首位 0 且非小数点前一位
    int skip_leading_zero = (digits[3] == 0) && (decimal_pos != 3);

    // 显示所有数字
    for(i = 3; i < 4; i--)
    {
        if (i == 3 && skip_leading_zero)
            continue;

        uint8_t seg_data;

        // 判断是否需要显示小数点
        if(i == decimal_pos)
        {
            // 使用带小数点的段码(索引18 - 27)
            seg_data = LED_0F[18 + digits[i]];
        }
        else
        {
            // 使用普通段码
            seg_data = LED_0F[digits[i]];
        }

        LED_OUT(seg_data);       // 发送段码
        LED_OUT(0x01 << i);     // 发送位码

        RCLK(0);  // 锁存数据
        RCLK(1);

        rt_thread_mdelay(2);    // 短暂延时
    }
}

// 全局变量
int seg_display_data = 1234;    // 要显示的数字
uint8_t decimal_position = 2;   // 小数点位置(0 - 3)，1表示在第2位后面

// 软件定时器回调函数
static void seg_refresh_timer_callback(void* parameter)
{
    static uint8_t current_digit = 0;
    uint8_t digits[4];
    uint8_t i;
    int temp_data = seg_display_data;

    // 分离各位数字
    for(i = 0; i < 4; i++)
    {
        digits[i] = temp_data % 10;
        temp_data /= 10;
    }

    // 标记是否为首位 0 且非小数点前一位
    int skip_leading_zero = (digits[3] == 0) && (decimal_position != 3);

    if (current_digit == 3 && skip_leading_zero)
    {
        current_digit++;
        if (current_digit >= 4)
            current_digit = 0;
        return;
    }

    // 显示当前位
    uint8_t seg_data;

    // 判断是否需要显示小数点
    if((3 - current_digit) == decimal_position)
    {
        seg_data = LED_0F[18 + digits[3 - current_digit]]; // 带小数点的数字
    }
    else
    {
        seg_data = LED_0F[digits[3 - current_digit]];
    }

    LED_OUT(seg_data);                   // 发送段码
    LED_OUT(0x01 << (3 - current_digit));      // 发送位码
    RCLK(0);                            // 锁存数据
    RCLK(1);

    // 更新到下一位
    current_digit++;
    if(current_digit >= 4)
    {
        current_digit = 0;
    }
}

// 初始化数码管驱动
int seg_driver_init(void)
{
    digital_init();

    // 创建软件定时器
    rt_timer_t seg_refresh_timer = rt_timer_create("seg_refresh", 
                                                  seg_refresh_timer_callback, 
                                                  RT_NULL, 
                                                  2,  // 2ms刷新间隔
                                                  RT_TIMER_FLAG_PERIODIC);
    if (seg_refresh_timer != RT_NULL)
    {
        rt_timer_start(seg_refresh_timer);
    }
    else
    {
        rt_kprintf("Failed to create seg refresh timer!\n");
        return -RT_ERROR;
    }
    return RT_EOK;
}
INIT_APP_EXPORT(seg_driver_init);

// 设置显示的数字和小数点位置
void seg_set_display_data(int data, uint8_t decimal_pos)
{
    if(data > 9999) data = 9999;  // 限制最大4位数
    if(decimal_pos > 3) decimal_pos = 1; // 默认小数点在第2位后面

    seg_display_data = data;
    decimal_position = decimal_pos;
}

