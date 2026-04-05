/**
 * @file chassis_task.h
 * @brief 底盘控制任务
 * @date 2026-04-04
 */

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "robot.h"

/**
 * @brief 底盘控制任务初始化
 */
void chassis_task_init(void);

/**
 * @brief 底盘控制任务主体
 */
void chassis_control_task(void);

#endif // CHASSIS_TASK_H
