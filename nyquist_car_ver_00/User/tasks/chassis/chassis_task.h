/**
 * @file    chassis_task.h
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-05-03
 * @brief   底盘控制任务 (巡线 + 角度环)
 */

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "robot.h"

/**
 * @brief  底盘任务初始化 (创建线程)
 */
void chassis_task_init(void);

/**
 * @brief  底盘任务入口 (FreeRTOS线程函数)
 */
void chassis_task_entry(void *argument);

#endif /* CHASSIS_TASK_H */
