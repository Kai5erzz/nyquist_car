/**
 * @file    drv8870.c
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-05-01
 * @brief   DRV8870DDAR 双PWM电机驱动实现
 *
 * 硬件连接:
 *   MOTOR1: TIM3 CH1(PA6) + CH2(PA7)
 *   MOTOR2: TIM3 CH3(PB0) + CH4(PB1)
 *   MOTOR3: TIM2 CH1(PA5) + CH2(PB3)
 *   MOTOR4: TIM2 CH3(PA2) + CH4(PA3)
 */

#include "drv8870.h"
#include "tim.h"

DRV8870_Instance_t drv8870;

/**
 * @brief  初始化DRV8870, 绑定定时器句柄和PWM通道
 * @note   4个电机分别映射到TIM2和TIM3的4个通道
 */
void DRV8870_Init(void)
{
    /* MOTOR1: TIM3 CH1 + CH2 */
    drv8870.motor[0].htim       = &htim3;
    drv8870.motor[0].channel_in1 = TIM_CHANNEL_1;
    drv8870.motor[0].channel_in2 = TIM_CHANNEL_2;
    drv8870.motor[0].max_duty   = htim3.Init.Period;
    drv8870.motor[0].duty       = 0;

    /* MOTOR2: TIM3 CH4 + CH3 (IN1/IN2反接) */
    drv8870.motor[1].htim       = &htim3;
    drv8870.motor[1].channel_in1 = TIM_CHANNEL_4;
    drv8870.motor[1].channel_in2 = TIM_CHANNEL_3;
    drv8870.motor[1].max_duty   = htim3.Init.Period;
    drv8870.motor[1].duty       = 0;

    /* MOTOR3: TIM2 CH2 + CH1 (IN1/IN2反接) */
    drv8870.motor[2].htim       = &htim2;
    drv8870.motor[2].channel_in1 = TIM_CHANNEL_2;
    drv8870.motor[2].channel_in2 = TIM_CHANNEL_1;
    drv8870.motor[2].max_duty   = htim2.Init.Period;
    drv8870.motor[2].duty       = 0;

    /* MOTOR4: TIM2 CH4 + CH3 (IN1/IN2反接) */
    drv8870.motor[3].htim       = &htim2;
    drv8870.motor[3].channel_in1 = TIM_CHANNEL_4;
    drv8870.motor[3].channel_in2 = TIM_CHANNEL_3;
    drv8870.motor[3].max_duty   = htim2.Init.Period;
    drv8870.motor[3].duty       = 0;
}

/**
 * @brief  启动所有8路PWM输出 (TIM2 CH1~4 + TIM3 CH1~4)
 */
void DRV8870_Start(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

/**
 * @brief  设置电机占空比
 * @param  index  电机索引 [0, 3]
 * @param  duty   占空比值, 正=正转(IN1>PWM, IN2=0), 负=反转(IN1=0, IN2>PWM)
 * @note   自动限幅到 [-max_duty, +max_duty]
 */
void DRV8870_SetDuty(uint8_t index, int16_t duty)
{
    if (index >= DRV8870_MOTOR_NUM) return;

    DRV8870_Motor_t *m = &drv8870.motor[index];

    if (duty > (int16_t)m->max_duty)  duty = (int16_t)m->max_duty;
    if (duty < -(int16_t)m->max_duty) duty = -(int16_t)m->max_duty;
    m->duty = duty;

    uint16_t pwm_val;
    if (duty >= 0) {
        pwm_val = (uint16_t)duty;
        __HAL_TIM_SET_COMPARE(m->htim, m->channel_in1, pwm_val);
        __HAL_TIM_SET_COMPARE(m->htim, m->channel_in2, 0);
    } else {
        pwm_val = (uint16_t)(-duty);
        __HAL_TIM_SET_COMPARE(m->htim, m->channel_in1, 0);
        __HAL_TIM_SET_COMPARE(m->htim, m->channel_in2, pwm_val);
    }
}

/**
 * @brief  单个电机制动
 * @param  index  电机索引 [0, 3]
 * @note   将IN1和IN2同时置零
 */
void DRV8870_Stop(uint8_t index)
{
    if (index >= DRV8870_MOTOR_NUM) return;
    DRV8870_Motor_t *m = &drv8870.motor[index];
    __HAL_TIM_SET_COMPARE(m->htim, m->channel_in1, 0);
    __HAL_TIM_SET_COMPARE(m->htim, m->channel_in2, 0);
    m->duty = 0;
}

/**
 * @brief  全部4个电机制动
 */
void DRV8870_StopAll(void)
{
    for (uint8_t i = 0; i < DRV8870_MOTOR_NUM; i++) {
        DRV8870_Stop(i);
    }
}
