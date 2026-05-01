/**
 * @file    sense_task.h
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-05-01
 * @brief   传感器调试任务
 *
 * 功能:
 *   - 读取全部传感器反馈 (编码器/电流/IMU/灰度)
 *   - 通过VOFA+输出到上位机验证反馈链路
 *   - 暂停电机控制, 仅做数据采集和显示
 */

#ifndef SENSE_TASK_H
#define SENSE_TASK_H

#include "robot.h"

/**
 * @brief  传感器任务初始化 (创建线程)
 */
void sense_task_init(void);

/**
 * @brief  传感器任务入口 (FreeRTOS线程函数)
 */
void sense_task_entry(void *argument);

#endif /* SENSE_TASK_H */
