/**
 * @file    drv8870.h
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-05-01
 * @brief   DRV8870DDAR 双PWM电机驱动
 *
 * 功能:
 *   - 双PWM控制DRV8870H桥驱动器
 *   - 支持正转/反转/制动
 *   - 自动限幅保护
 *
 * 电机-PWM映射:
 *   MOTOR1: TIM3 CH1 + CH2
 *   MOTOR2: TIM3 CH3 + CH4
 *   MOTOR3: TIM2 CH1 + CH2
 *   MOTOR4: TIM2 CH3 + CH4
 *
 * 控制逻辑:
 *   正转: IN1=PWM, IN2=0
 *   反转: IN1=0,   IN2=PWM
 *   制动: IN1=IN2=0
 */

#ifndef DRV8870_H
#define DRV8870_H

#include "stm32h7xx_hal.h"

#define DRV8870_MOTOR_NUM  4

/**
 * @brief 单个电机实例结构体
 */
typedef struct {
    TIM_HandleTypeDef *htim;        /**< 定时器句柄 */
    uint32_t channel_in1;           /**< IN1通道 (TIM_CHANNEL_x) */
    uint32_t channel_in2;           /**< IN2通道 (TIM_CHANNEL_x) */
    int16_t  duty;                  /**< 当前占空比 [-max_duty, +max_duty] */
    uint16_t max_duty;              /**< 最大占空比 (ARR值) */
} DRV8870_Motor_t;

/**
 * @brief DRV8870实例结构体 (包含4个电机)
 */
typedef struct {
    DRV8870_Motor_t motor[DRV8870_MOTOR_NUM];
} DRV8870_Instance_t;

extern DRV8870_Instance_t drv8870;

/**
 * @brief  初始化DRV8870, 绑定定时器和通道
 */
void DRV8870_Init(void);

/**
 * @brief  启动所有8路PWM输出
 */
void DRV8870_Start(void);

/**
 * @brief  设置电机占空比
 * @param  index  电机索引 [0, 3]
 * @param  duty   占空比 [-max_duty, +max_duty], 正=正转, 负=反转
 */
void DRV8870_SetDuty(uint8_t index, int16_t duty);

/**
 * @brief  单个电机制动 (IN1=IN2=0)
 * @param  index  电机索引 [0, 3]
 */
void DRV8870_Stop(uint8_t index);

/**
 * @brief  全部电机制动
 */
void DRV8870_StopAll(void);

#endif /* DRV8870_H */
