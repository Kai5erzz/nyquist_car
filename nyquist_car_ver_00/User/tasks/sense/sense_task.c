/**
 * @file    sense_task.c
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-05-03
 * @brief   传感器任务 (预留)
 */

#include "sense_task.h"
#include "cmsis_os.h"
#include "rm_task.h"

osThreadId_t senseTaskHandle;

__attribute__((noreturn))
void sense_task_entry(void *argument)
{
    for (;;) {
        /* 预留: 传感器相关逻辑 */
        osDelay(SENSE_TASK_PERIOD);
    }
}

void sense_task_init(void)
{
    const osThreadAttr_t sense_task_attributes = {
        .name = "sense_task",
        .stack_size = SENSE_TASK_STACK_SIZE,
        .priority = (osPriority_t) SENSE_TASK_PRIORITY,
    };

    senseTaskHandle = osThreadNew(sense_task_entry, NULL, &sense_task_attributes);
}
