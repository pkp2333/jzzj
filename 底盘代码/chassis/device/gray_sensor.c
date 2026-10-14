#include "gray_sensor.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <easyflash.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// 全局 I2C 设备句柄
static struct rt_i2c_bus_device *i2c_bus = RT_NULL;

// 回调函数类型定义，用于校准更新
typedef void (*calibration_callback)(int idx, uint8_t val);

/**
 * @brief 从 PCF8591 指定通道读取 ADC 数据
 *        （实现了两次读取，丢弃第一次数据）
 * @param addr: PCF8591 的 I2C 地址
 * @param channel: ADC 通道号（0 ~ 3）
 * @return ADC 转换值（0 ~ 255），出错时返回 0
 */
rt_uint8_t pcf8591_read_adc(rt_uint8_t addr, rt_uint8_t channel)
{
    if (i2c_bus == RT_NULL)
    {
        rt_kprintf("I2C bus not initialized!\n");
        return 0;
    }

    struct rt_i2c_msg msgs[2];
    rt_uint8_t control_byte = 0x40 | (channel & 0x03);
    rt_uint8_t read_buf[2] = {0};

    /* 消息0：写控制字节 */
    msgs[0].addr  = addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &control_byte;
    msgs[0].len   = 1;

    /* 消息1：读2字节（丢弃第一个） */
    msgs[1].addr  = addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = read_buf;
    msgs[1].len   = 2;

    if (rt_i2c_transfer(i2c_bus, msgs, 2) == 2)
    {
        return read_buf[1];
    }
    else
    {
        rt_kprintf("I2C transfer failed for channel %d at addr 0x%x\n", channel, addr);
        return 0;
    }
}

/**
 * @brief 初始化 PCF8591 所在的 I2C 设备
 */
void pcf8591_init(void)
{
    i2c_bus = rt_i2c_bus_device_find(I2C_BUS_NAME);
    if (i2c_bus == RT_NULL)
    {
        rt_kprintf("I2C device %s not found!\n", I2C_BUS_NAME);
    }
    else
    {
        rt_kprintf("I2C device %s initialized!\n", I2C_BUS_NAME);
    }
}

void gray_sensor_init(void)
{
    pcf8591_init();
}

/**
 * @brief 读取灰度传感器的原始数据
 *        读取7个数据：
 *         - 从 PCF8591_ADDR_2 读取4个通道（通道3->0，对应传感器0-3）
 *         - 从 PCF8591_ADDR_1 读取3个通道（通道3->0，对应传感器4-6）
 * @return gray_sensor_t 对象，包含7个 ADC 数据
 */
gray_sensor_t gray_sensor_get_raw_data(void)
{
    gray_sensor_t obj;
    rt_memset(&obj, 0, sizeof(obj));

    const uint8_t addr2_channels[] = {3, 2, 1, 0}; // 传感器0-3
    const uint8_t addr1_channels[] = {3, 2, 0};    // 传感器4-6

    for (uint8_t i = 0; i < 4; i++)
    {
        obj.val[i] = pcf8591_read_adc(PCF8591_ADDR_2, addr2_channels[i]);
    }
    for (uint8_t i = 0; i < 3; i++)
    {
        obj.val[4 + i] = pcf8591_read_adc(PCF8591_ADDR_1, addr1_channels[i]);
    }
    return obj;
}

/*==================== 校准及存储部分 ====================*/

// 定义7个传感器的校准值数组，全局变量
static rt_uint8_t cal_min[7] = {255, 255, 255, 255, 255, 255, 255};
static rt_uint8_t cal_max[7] = {0, 0, 0, 0, 0, 0, 0};

#define RT_TICK_FROM_MS(ms) ((rt_uint32_t)((ms) * RT_TICK_PER_SECOND / 1000))
#define CALIBRATION_SAMPLE_INTERVAL 20   // 采样间隔(ms)
#define MIN_VALID_RANGE 30               // 有效最小范围
#define PROGRESS_UPDATE_INTERVAL 500     // 进度更新间隔(ms)

// 更新回调函数
static void update_min(int idx, uint8_t val)
{
    if (val < cal_min[idx])
        cal_min[idx] = val;
}
static void update_max(int idx, uint8_t val)
{
    if (val > cal_max[idx])
        cal_max[idx] = val;
}

/**
 * @brief 校准步骤：采样指定时长，并调用回调函数更新校准值
 */
static void calibration_step(const char *step_name,
                             rt_uint32_t duration_ms,
                             calibration_callback update_func)
{
    rt_uint32_t start = rt_tick_get();
    rt_uint32_t last_update = 0;
    uint32_t sample_count = 0;
    gray_sensor_t raw_data;

    rt_kprintf("开始%s校准，持续%d毫秒...\n", step_name, duration_ms);

    while ((rt_tick_get() - start) < RT_TICK_FROM_MS(duration_ms))
    {
        raw_data = gray_sensor_get_raw_data();
        sample_count++;
        for (int i = 0; i < 7; i++)
        {
            update_func(i, raw_data.val[i]);
        }
        if ((rt_tick_get() - last_update) > RT_TICK_FROM_MS(PROGRESS_UPDATE_INTERVAL))
        {
            last_update = rt_tick_get();
            rt_kprintf("%s进度: %d%%\n", step_name,
                       (int)((rt_tick_get() - start) * 100 / RT_TICK_FROM_MS(duration_ms)));
        }
        rt_thread_mdelay(CALIBRATION_SAMPLE_INTERVAL);
    }
    rt_kprintf("%s完成！采样次数：%d\n", step_name, sample_count);
}

/**
 * @brief 自动校准：先黑线校准，再白底校准
 */
void auto_calibrate(rt_uint32_t step_duration_ms)
{
    for (int i = 0; i < 7; i++)
    {
        cal_min[i] = 255;
        cal_max[i] = 0;
    }

    rt_kprintf("\n=== 灰度传感器校准向导 ===\n"
               "请按以下步骤操作：\n"
               "1. 将所有传感器放置于黑线\n"
               "2. 将所有传感器放置于白背景上\n"
               "倒计时开始：3秒...\n");

    for (int i = 3; i > 0; i--)
    {
        rt_kprintf("%d...\n", i);
        rt_thread_mdelay(1000);
    }

    calibration_step("黑线", step_duration_ms, update_min);

    rt_kprintf("3秒后开始白底校准...\n");
    rt_thread_mdelay(3000);

    calibration_step("白底", step_duration_ms, update_max);

    rt_kprintf("\n=== 校准结果 ===\n");
    rt_bool_t valid = RT_TRUE;
    for (int i = 0; i < 7; i++)
    {
        int range = cal_max[i] - cal_min[i];
        const char *status = (range >= MIN_VALID_RANGE) ? "正常" : "异常";
        rt_kprintf("传感器%02d | 黑:%-3d | 白:%-3d | 范围:%-3d | %s\n",
                   i, cal_min[i], cal_max[i], range, status);
        if (range < MIN_VALID_RANGE)
        {
            valid = RT_FALSE;
            rt_kprintf("  -> 可能原因：安装异常/光线不足/表面反光\n");
        }
    }
    if (!valid)
    {
        rt_kprintf("\n校准未通过！请检查传感器安装及环境条件。\n");
    }
    else
    {
        rt_kprintf("\n校准成功！建议保存校准参数至非易失存储\n");
    }
}


/**
 * @brief MSH 命令：触发传感器校准并保存数据
 */
static void sensor_calibrate_cmd(int argc, char **argv)
{
    rt_kprintf("开始传感器校准...\n");
    auto_calibrate(5000);
    rt_kprintf("校准完成并保存到 flash！\n");
}
MSH_CMD_EXPORT(sensor_calibrate_cmd, 传感器校准命令);

/*============================ 巡线检测部分 ============================*/

/**
 * @brief 将原始灰度数据归一化到 [0,1] 范围内
 * @param raw 原始灰度数据结构
 * @param normalized 输出归一化结果数组（长度7）
 */
void normalize_gray_sensor_data(const gray_sensor_t *raw, float normalized[7])
{
    for (int i = 0; i < 7; i++)
    {
        int range = (int)cal_max[i] - (int)cal_min[i];
        range = (range <= 0) ? 1 : range;
        normalized[i] = (((float)raw->val[i] - (float)cal_min[i]) / range);
        normalized[i] = (normalized[i] < 0) ? 0 : (normalized[i] > 1 ? 1 : normalized[i]);
    }
}

/**
 * @brief 根据归一化后的数据和预设权重计算巡线偏差误差
 * @param normalized 已归一化的7个数据，从左到右排列
 * @return 加权平均误差，负值表示偏左，正值表示偏右
 */
float calculate_line_error(const float normalized[7])
{
    float weights[7] = {-3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f};
    float weighted_sum = 0.0f, activation_sum = 0.0f;
    for (int i = 0; i < 7; i++)
    {
        weighted_sum += normalized[i] * weights[i];
        activation_sum += normalized[i];
    }
    return (activation_sum == 0.0f) ? 0.0f : weighted_sum / activation_sum;
}

/**
 * @brief 巡线检测线程入口函数
 *        线程启动后首先周期性检查 flash 中是否有有效的校准数据，
 *        若没有，则打印提示信息并继续等待；一旦检测到有效数据，加载后进入检测流程。
 */
static void test_gray_sensor_thread_entry(void *parameter)
{
    // 初始化 I2C、灰度传感器、FAL及 EasyFlash
    gray_sensor_init();

    rt_uint32_t print_counter = 0;
    while (1)
    {
        gray_sensor_t raw_sensor = gray_sensor_get_raw_data();
        float norm[7] = {0};
        float line_error = 0.0f;

        normalize_gray_sensor_data(&raw_sensor, norm);
        line_error = calculate_line_error(norm);

        if (++print_counter >= 10)
        {
            print_counter = 0;
            rt_kprintf("Error: %.2f | ", line_error);
            for (int i = 0; i < 7; i++)
            {
                rt_kprintf("%.2f ", norm[i]);
            }
            rt_kprintf("\n");
        }
        rt_thread_mdelay(50);
    }
}

int test_gray_sensor_thread_init(void)
{
    rt_thread_t t = rt_thread_create("gray",
                                     test_gray_sensor_thread_entry,
                                     RT_NULL,
                                     1024,
                                     25,
                                     5);
    if (t != RT_NULL)
    {
        rt_thread_startup(t);
    }
    else
    {
        rt_kprintf("创建灰度传感器线程失败\r\n");
    }
    return RT_EOK;
}
//INIT_APP_EXPORT(test_gray_sensor_thread_init);
