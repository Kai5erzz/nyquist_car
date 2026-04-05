/**
 * @file motor_task.h
 * @brief 电机控制任务
 * @date 2026-04-04
 */

#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "robot.h"
#include "gpio.h"
/**
 * @brief 电机任务初始化
 */
void motor_task_init(void);

/**
 * @brief 电机控制任务主体
 */
void motor_control_task(void);

#endif // MOTOR_TASK_H
