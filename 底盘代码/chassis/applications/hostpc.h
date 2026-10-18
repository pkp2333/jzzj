#ifndef __HOSTPC_H__
#define __HOSTPC_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define HOSTPC_UART              "uart4"

#define HOSTPC_THREAD_PRIORITY   25
#define HOSTPC_THREAD_STACK_SIZE 512
#define HOSTPC_THREAD_TIMESLICE  10

/* ----------------------- Data Struct ------------------------------------- */

typedef struct
{
    int8_t mode;
    int8_t x;
    int8_t y;
    int8_t Raw;
} hostTypeDef;

extern hostTypeDef host;

// 添加互斥锁声明
extern struct rt_mutex host_mutex;

// 新增安全访问接口
static inline void hostpc_get_data(hostTypeDef *dest) {
    rt_mutex_take(&host_mutex, RT_WAITING_FOREVER);
    *dest = host;
    rt_mutex_release(&host_mutex);
}

#endif
