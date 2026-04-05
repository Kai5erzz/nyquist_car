/**
 * @file ins_task.h
 * @brief IMU/INS惯性导航任务
 * @date 2026-04-04
 */

#ifndef INS_TASK_H
#define INS_TASK_H

#include "robot.h"

/**
 * @brief INS任务初始化
 */
void ins_task_init(void);

/**
 * @brief INS任务主体
 */
void ins_task_entry(void *argument);

#endif // INS_TASK_H
