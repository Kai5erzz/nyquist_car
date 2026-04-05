/**
 * @file sense_task.h
 * @brief 传感器感知任务
 * @date 2026-04-04
 */

#ifndef SENSE_TASK_H
#define SENSE_TASK_H

#include "robot.h"

/**
 * @brief 传感器任务初始化
 */
void sense_task_init(void);

/**
 * @brief 传感器任务主体
 */
void sense_task_run(void);

#endif // SENSE_TASK_H
