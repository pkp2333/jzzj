#include "hostpc.h"

hostTypeDef host = {0, 0, 0, 0};

// 定义互斥锁
struct rt_mutex host_mutex;

static rt_device_t serial;

struct rx_msg {
    rt_device_t dev;
    rt_size_t size;
};

static struct rt_semaphore rx_sem;
static struct rt_messagequeue rx_mq;

// 协议宏定义
#define FRAME_HEADER   0xA5
#define FRAME_END      0x5A
#define FRAME_DATA_LEN 4

static uint8_t rx_buffer[RT_SERIAL_RB_BUFSZ + 1];

static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg = {dev, size};
    return rt_mq_send(&rx_mq, &msg, sizeof(msg));
}

static void hostpc_thread_entry(void *parameter)
{
    struct rx_msg msg;
    uint8_t data_cnt = 0;
    uint8_t *data_ptr = rx_buffer;

    while (1) {
        rt_mq_recv(&rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
        
        uint32_t recv_size = rt_device_read(msg.dev, 0, 
                                          &rx_buffer[data_cnt], 
                                          msg.size);
        data_cnt += recv_size;

        // 协议解析核心逻辑
        if (data_ptr[0] == FRAME_HEADER && data_cnt >= 7) {
            if (data_ptr[6] == FRAME_END) {
                rt_mutex_take(&host_mutex, RT_WAITING_FOREVER);
				rt_memcpy(&host, &data_ptr[1], FRAME_DATA_LEN);
				rt_mutex_release(&host_mutex);
//				 // 打印解析结果
                rt_kprintf("[HOST] x:%d | y:%d | z:%d | mode:%d\r\n",
                          host.x,
                          host.y,
                          host.Raw,
                          host.mode);
                data_cnt = 0;
            } else {
                data_cnt = 0;
            }
        }
    }
}

int hostpc_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    
    if ((serial = rt_device_find(HOSTPC_UART)) == RT_NULL) {
        rt_kprintf("UART device not found\n");
        return RT_ERROR;
    }
	
	rt_mutex_init(&host_mutex, "host", RT_IPC_FLAG_FIFO);
	
    rt_mq_init(&rx_mq, "rx_mq", 
              rt_malloc(sizeof(struct rx_msg)*8),
              sizeof(struct rx_msg), 
              sizeof(struct rx_msg)*8, 
              RT_IPC_FLAG_FIFO);

    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
    rt_device_open(serial, RT_DEVICE_FLAG_DMA_RX);
    rt_device_set_rx_indicate(serial, uart_input);

    rt_thread_t thread = rt_thread_create("HOSTPC",
                                        hostpc_thread_entry,
                                        RT_NULL,
                                        1024,
                                        20,
                                        10);
    if (thread) rt_thread_startup(thread);
    
    return RT_EOK;
}
INIT_APP_EXPORT(hostpc_init);
