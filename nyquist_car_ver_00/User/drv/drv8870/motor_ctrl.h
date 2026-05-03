/**
 * @file    motor_ctrl.h
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-05-01
 * @brief   电机控制模块 (TIM6中断驱动, 500Hz)
 *
 * 功能:
 *   - 编码器速度/角度反馈
 *   - 两种控制模式: 恒速度、恒位置
 *   - 位置环级联速度环PID
 *
 * 电机-外设映射:
 *   MOTOR1: LPTIM1编码器(PG11/12), TIM3 CH1/2 PWM
 *   MOTOR2: LPTIM2编码器(PD11/12), TIM3 CH3/4 PWM
 *   MOTOR3: TIM4编码器(PB6/7),    TIM2 CH1/2 PWM
 *   MOTOR4: TIM5编码器(PA0/1),    TIM2 CH3/4 PWM
 */

#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include <stdint.h>

#define MOTOR_NUM  4

/* ==================== 电机参数宏 (用户可修改) ==================== */
#define ENCODER_PPR         13         /**< 编码器线数 */
#define ENCODER_RATIO       20.0f      /**< 减速比 */
#define WHEEL_RADIUS        0.048f     /**< 轮半径 (m) */

/**
 * @brief 电机控制模式枚举
 */
typedef enum {
    MOTOR_MODE_DISABLE = 0, /**< 失能, PWM=0 */
    MOTOR_MODE_SPEED,       /**< 恒速度输出 */
    MOTOR_MODE_POSITION,    /**< 恒位置输出 (级联速度环) */
} Motor_Mode_e;

/**
 * @brief 单个电机数据结构体
 */
typedef struct {
    /* 反馈量 */
    float speed;            /**< 速度反馈 (rad/s) */
    float angle;            /**< 角度反馈 (rad) */

    /* 目标量 */
    float target_speed;     /**< 目标速度 (rad/s) */
    float target_angle;     /**< 目标角度 (rad) */

    /* 输出量 */
    int16_t pwm_out;        /**< PWM输出值 */

    /* 状态 */
    Motor_Mode_e mode;      /**< 当前控制模式 */
} Motor_Data_t;

extern Motor_Data_t motor_data[MOTOR_NUM];

/* ==================== 初始化与控制 ==================== */

/**
 * @brief  初始化电机控制模块
 * @note   初始化DRV8870、编码器、TIM6中断, 清零所有数据
 */
void Motor_Ctrl_Init(void);

/**
 * @brief  电机控制主循环
 * @note   在TIM6中断中调用 (500Hz), 包含: 编码器读取→PID计算→PWM输出
 */
void Motor_Ctrl_Loop(void);

/**
 * @brief  使能电机控制, 启动PWM输出
 */
void Motor_Ctrl_Enable(void);

/**
 * @brief  失能电机控制, 停止所有PWM, 清除PID积分
 */
void Motor_Ctrl_Disable(void);

/* ==================== 反馈函数 ==================== */

/**
 * @brief  获取电机速度
 * @param  index  电机索引 [0, 3]
 * @return 速度 (rad/s)
 */
float Motor_GetSpeed(uint8_t index);

/**
 * @brief  获取电机角度
 * @param  index  电机索引 [0, 3]
 * @return 角度 (rad)
 */
float Motor_GetAngle(uint8_t index);

/* ==================== 控制命令函数 ==================== */

/**
 * @brief  设置恒速度模式
 * @param  index  电机索引 [0, 3]
 * @param  speed  目标速度 (rad/s)
 */
void Motor_SetSpeed(uint8_t index, float speed);

/**
 * @brief  设置恒位置模式
 * @param  index  电机索引 [0, 3]
 * @param  angle  目标角度 (rad)
 */
void Motor_SetPosition(uint8_t index, float angle);

/**
 * @brief  设置电机控制模式
 * @param  index  电机索引 [0, 3]
 * @param  mode   控制模式
 */
void Motor_SetMode(uint8_t index, Motor_Mode_e mode);

/**
 * @brief  直接设置PWM值 (调试用, 会切换到DISABLE模式)
 * @param  index  电机索引 [0, 3]
 * @param  pwm    PWM值 [-max_duty, +max_duty]
 */
void Motor_SetPWM(uint8_t index, int16_t pwm);

#endif /* MOTOR_CTRL_H */
