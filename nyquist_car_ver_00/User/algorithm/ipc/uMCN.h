/**
 * @file uMCN.h
 * @brief uMCN (micro Message Communication Network) - Lightweight publish-subscribe IPC
 * @date 2023-08-28
 * @copyright Copyright 2020-2021 The Firmament Authors
 */

#ifndef UMCN_H__
#define UMCN_H__

/* 1. 首先包含标准类型定义，解决 uint32_t 未定义问题 */
#include <stdbool.h>
#include <stdint.h>

/* 2. 包含 FreeRTOS 核心头文件，解决 pdTRUE/pdFALSE 和调度器函数问题 */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 3. 修复 CMSIS-RTOS V2 中不存在 osThreadSuspendAll 的问题 */
#define MCN_MALLOC(size)            pvPortMalloc(size)
#define MCN_FREE(ptr)               vPortFree(ptr)
#define MCN_ENTER_CRITICAL          vTaskSuspendAll()
#define MCN_EXIT_CRITICAL           xTaskResumeAll()
#define MCN_EVENT_HANDLE            osSemaphoreId
#define MCN_SEND_EVENT(event)       osSemaphoreRelease(event)
#define MCN_WAIT_EVENT(event, time) osSemaphoreWait(event, time)
#define MCN_ASSERT(EX)              configASSERT(EX)

#define MCN_MAX_LINK_NUM        30
#define MCN_FREQ_EST_WINDOW_LEN 5

typedef enum {
    FMT_EOK = 0,
    FMT_ERROR = 1,
    FMT_ETIMEOUT = 2,
    FMT_EFULL = 3,
    FMT_EEMPTY = 4,
    FMT_ENOMEM = 5,
    FMT_ENOSYS = 6,
    FMT_EBUSY = 7,
    FMT_EIO = 8,
    FMT_EINTR = 9,
    FMT_EINVAL = 10,
    FMT_ENOTHANDLE = 11,
} fmt_err_t;

typedef struct mcn_node McnNode;
typedef struct mcn_node* McnNode_t;
struct mcn_node {
    volatile uint8_t renewal;
    MCN_EVENT_HANDLE event;
    void (*pub_cb)(void* parameter);
    McnNode_t next;
};

typedef struct mcn_hub McnHub;
typedef struct mcn_hub* McnHub_t;
struct mcn_hub {
    const char* obj_name;
    const uint32_t obj_size;
    void* pdata;
    McnNode_t link_head;
    McnNode_t link_tail;
    uint32_t link_num;
    uint8_t published;
    uint8_t suspend;
    int (*echo)(void* parameter);
    float freq;
    uint16_t freq_est_window[MCN_FREQ_EST_WINDOW_LEN];
    uint16_t window_index;
};

typedef struct mcn_list McnList;
typedef struct mcn_list* McnList_t;
struct mcn_list {
    McnHub_t hub;
    McnList_t next;
};

/******************* Helper Macro *******************/
#define MCN_HUB(_name) (&__mcn_##_name)
#define MCN_DECLARE(_name) extern McnHub __mcn_##_name
#define MCN_DEFINE(_name, _size) \
    McnHub __mcn_##_name = {     \
        .obj_name = #_name,      \
        .obj_size = _size,       \
        .pdata = NULL,           \
        .link_head = NULL,       \
        .link_tail = NULL,       \
        .link_num = 0,           \
        .published = 0,          \
        .suspend = 0,            \
        .freq = 0.0f             \
    }

/******************* API *******************/
fmt_err_t mcn_init(void);
fmt_err_t mcn_advertise(McnHub_t hub, int (*echo)(void* parameter));
McnNode_t mcn_subscribe(McnHub_t hub, MCN_EVENT_HANDLE event, void (*pub_cb)(void* parameter));
fmt_err_t mcn_unsubscribe(McnHub_t hub, McnNode_t node);
fmt_err_t mcn_publish(McnHub_t hub, const void* data);
bool mcn_poll(McnNode_t node_t);
bool mcn_wait(McnNode_t node_t, int32_t timeout);
fmt_err_t mcn_copy(McnHub_t hub, McnNode_t node_t, void* buffer);
fmt_err_t mcn_copy_from_hub(McnHub_t hub, void* buffer);
void mcn_suspend(McnHub_t hub);
void mcn_resume(McnHub_t hub);
McnList_t mcn_get_list(void);
McnHub_t mcn_iterate(McnList_t* ite);
void mcn_node_clear(McnNode_t node_t);

#ifdef __cplusplus
}
#endif

#endif