/**
 * @file rm_task.h
 * @brief FreeRTOS task management
 * @date 2026-04-04
 */

#ifndef RM_TASK_H
#define RM_TASK_H

#include "cmsis_os.h"

/* ==================== Task Priorities ==================== */
#define INS_TASK_PRIORITY    osPriorityNormal
#define MOTOR_TASK_PRIORITY  osPriorityNormal
#define CHASSIS_TASK_PRIORITY osPriorityNormal
#define CMD_TASK_PRIORITY    osPriorityHigh
#define SENSE_TASK_PRIORITY  osPriorityNormal

/* ==================== Task Stack Sizes ==================== */
#define INS_TASK_STACK_SIZE    1024
#define MOTOR_TASK_STACK_SIZE  2048
#define CHASSIS_TASK_STACK_SIZE 2048
#define CMD_TASK_STACK_SIZE     1024
#define SENSE_TASK_STACK_SIZE  1024

/* ==================== Task Periods (ms) ==================== */
#define INS_TASK_PERIOD       1    // 1ms = 1000Hz
#define MOTOR_TASK_PERIOD     1    // 1ms = 1000Hz
#define CHASSIS_TASK_PERIOD   1    // 1ms = 1000Hz
#define CMD_TASK_PERIOD       10   // 10ms = 100Hz
#define SENSE_TASK_PERIOD     10   // 10ms = 100Hz

/* ==================== OS Task Init ==================== */
/**
 * @brief 初始化所有RTOS任务
 */
void OS_task_init(void);

/* ==================== Task Entry Declarations ==================== */
void ins_task_entry(void *argument);
void motor_task_entry(void *argument);
void chassis_task_entry(void *argument);
void cmd_task_entry(void *argument);
void sense_task_entry(void *argument);

/* ==================== Task Init Declarations ==================== */
void ins_task_init(void);
void motor_task_init(void);
void chassis_task_init(void);
void cmd_task_init(void);
void sense_task_init(void);

#endif // RM_TASK_H
