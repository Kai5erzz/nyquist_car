/**
 * @file rm_config.h
 * @brief System configuration for the robot
 * @date 2026-04-04
 */

#ifndef RM_CONFIG_H
#define RM_CONFIG_H

#define CPU_FREQUENCY 168  /* CPU主频 (MHz) */

#include "stm32h7xx_hal.h"
#include "cmsis_os.h"

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#define user_free vPortFree
#else
#define user_malloc malloc
#define user_free free
#endif

/* ==================== CAN Bus Configuration ==================== */
#define CAN_CHASSIS    hfdcan1
#define CAN_GIMBAL     hfdcan2
#define CAN_ID_CHASSIS_MOTOR 1
#define CAN_ID_GIMBAL_MOTOR  2

/* ==================== Motor Configuration ==================== */
#define MOTOR1_ID  0x201
#define MOTOR2_ID  0x202
#define MOTOR3_ID  0x203
#define MOTOR4_ID  0x204

/* ==================== Control Parameters ==================== */
/* 底盘最大速度 */
#define MAX_CHASSIS_VX_SPEED  4.0f   // m/s
#define MAX_CHASSIS_VY_SPEED  2.0f   // m/s
#define MAX_CHASSIS_VW_SPEED  360.0f // °/s

/* 电机PID参数 */
#define MOTOR_SPEED_KP  10.0f
#define MOTOR_SPEED_KI  0.0f
#define MOTOR_SPEED_KD  0.0f
#define MOTOR_SPEED_MAX 16000

#define MOTOR_ANGLE_KP  5.0f
#define MOTOR_ANGLE_KI  0.0f
#define MOTOR_ANGLE_KD  0.0f
#define MOTOR_ANGLE_MAX 16000

/* ==================== Feature Toggles ==================== */
#define BSP_USING_DJI_MOTOR
#define BSP_USING_IMU
#define BSP_USING_CAN

#endif // RM_CONFIG_H
