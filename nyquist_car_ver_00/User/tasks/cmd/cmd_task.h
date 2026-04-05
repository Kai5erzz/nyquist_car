/**
 * @file cmd_task.h
 * @brief 命令/遥控器任务
 * @date 2026-04-04
 */

#ifndef CMD_TASK_H
#define CMD_TASK_H

#include "robot.h"

/**
 * @brief CMD任务初始化
 */
void cmd_task_init(void);

/**
 * @brief CMD控制任务主体
 */
void cmd_control_task(void);

#endif // CMD_TASK_H
