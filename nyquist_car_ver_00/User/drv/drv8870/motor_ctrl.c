/**
 * @file    motor_ctrl.c
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-05-01
 * @brief   电机控制模块实现
 *
 * 控制架构:
 *   TIM6中断(500Hz)触发 Motor_Ctrl_Loop()
 *     1. 读取编码器 → 计算速度/角度
 *     2. 读取ADC电流 → 计算力矩
 *     3. 根据控制模式计算PID输出
 *     4. 设置DRV8870 PWM
 *
 * PID参数:
 *   速度环: Kp=10, Ki=0.1, Kd=0
 *   位置环: Kp=5, Ki=0, Kd=0.5 (输出为速度命令)
 */

#include "motor_ctrl.h"
#include "drv8870.h"
#include "current_sense.h"
#include "tim.h"
#include "lptim.h"
#include <math.h>

#define RAD_PER_COUNT  (2.0f * M_PI / (ENCODER_PPR * 4.0f * ENCODER_RATIO))
#define CTRL_FREQ      500.0f   /**< 控制频率 (Hz) */

/* 速度环PID参数 */
#define SPEED_KP       120.0f
#define SPEED_KI       10.0f
#define SPEED_KD       0.01f
#define SPEED_OUT_MAX  10000.0f

/* 位置环PID参数 */
#define POS_KP         5.0f
#define POS_KI         0.0f
#define POS_KD         0.5f
#define POS_OUT_MAX    500.0f   /**< 位置环输出限幅 (速度命令) */

Motor_Data_t motor_data[MOTOR_NUM];

static volatile uint8_t motor_enabled = 0;
static int32_t encoder_prev[MOTOR_NUM] = {0};
static int32_t encoder_total[MOTOR_NUM] = {0};
static float speed_integrator[MOTOR_NUM] = {0};
static float pos_integrator[MOTOR_NUM] = {0};

/* ==================== 编码器读取 ==================== */

/**
 * @brief  读取编码器计数值
 * @param  index  电机索引 [0, 3]
 * @return 当前计数值 (有符号16-bit)
 *
 *   MOTOR1: LPTIM1 (PG11/PG12)
 *   MOTOR2: LPTIM2 (PD11/PD12)
 *   MOTOR3: TIM4   (PB6/PB7)
 *   MOTOR4: TIM5   (PA0/PA1)
 */
static int32_t Motor_ReadEncoder(uint8_t index)
{
    switch (index) {
        case 0: return (int16_t)(hlptim1.Instance->CNT);
        case 1: return (int16_t)(hlptim2.Instance->CNT);
        case 2: return (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        case 3: return (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
        default: return 0;
    }
}

/* ==================== PID控制器 ==================== */

/**
 * @brief  通用PI(D)控制器
 * @param  error      误差输入
 * @param  integrator 积分项指针 (保持状态)
 * @param  kp         比例增益
 * @param  ki         积分增益
 * @param  kd         微分增益 (当前未使用)
 * @param  dt         控制周期 (s)
 * @param  out_max    输出限幅
 * @return PID输出值
 */
static float PID_Calculate(float error, float *integrator,
                            float kp, float ki, float kd,
                            float dt, float out_max)
{
    *integrator += error * ki * dt;
    if (*integrator >  out_max) *integrator =  out_max;
    if (*integrator < -out_max) *integrator = -out_max;

    float output = kp * error + *integrator;
    if (output >  out_max) output =  out_max;
    if (output < -out_max) output = -out_max;
    return output;
}

/* ==================== 初始化 ==================== */

/**
 * @brief  初始化电机控制模块
 * @note   依次初始化: DRV8870→电流采样→TIM编码器→LPTIM编码器→TIM6中断
 */
void Motor_Ctrl_Init(void)
{
    DRV8870_Init();
    Current_Sense_Init();

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    HAL_LPTIM_Encoder_Start(&hlptim1, 0xFFFF);
    HAL_LPTIM_Encoder_Start(&hlptim2, 0xFFFF);
    HAL_TIM_Base_Start_IT(&htim6);

    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        motor_data[i].speed = 0;
        motor_data[i].angle = 0;
        motor_data[i].torque = 0;
        motor_data[i].current = 0;
        motor_data[i].target_torque = 0;
        motor_data[i].target_speed = 0;
        motor_data[i].target_angle = 0;
        motor_data[i].pwm_out = 0;
        motor_data[i].mode = MOTOR_MODE_DISABLE;
        encoder_prev[i] = 0;
        encoder_total[i] = 0;
        speed_integrator[i] = 0;
        pos_integrator[i] = 0;
    }
}

/* ==================== 控制主循环 ==================== */

/**
 * @brief  电机控制主循环 (TIM6中断调用, 500Hz)
 *
 * 执行流程:
 *   1. ADC读取4路电流
 *   2. 编码器读取 → 速度/角度计算
 *   3. 力矩计算: torque = current × Kt
 *   4. 根据控制模式执行PID
 *   5. 输出PWM到DRV8870
 */
void Motor_Ctrl_Loop(void)
{
    float dt = 1.0f / CTRL_FREQ;

    /* 1. 读取电流 */
    Current_Sense_ReadAll();

    /* 2. 编码器 → 速度/角度/力矩 */
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        int32_t enc_now = Motor_ReadEncoder(i);
        int32_t delta = enc_now - encoder_prev[i];

        if (delta > 32768)  delta -= 65536;
        if (delta < -32768) delta += 65536;

        encoder_prev[i] = enc_now;
        encoder_total[i] += delta;

        motor_data[i].speed   = (float)delta * RAD_PER_COUNT * CTRL_FREQ;
        motor_data[i].angle   = (float)encoder_total[i] * RAD_PER_COUNT;
        motor_data[i].current = Current_Sense_GetCurrent(i);
        motor_data[i].torque  = motor_data[i].current * MOTOR_KT;
    }

    if (!motor_enabled) return;

    /* 3. 控制模式计算 */
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        switch (motor_data[i].mode) {
            case MOTOR_MODE_TORQUE: {
                /* 力矩环: 简单比例控制, 不用PID */
                /* 电流→力矩: torque = current × Kt */
                /* 力矩→PWM:  pwm = (torque_error / Kt) × (max_duty / V_supply) */
                /* 简化: pwm = torque_error × TORQUE_KP */
                #define TORQUE_KP  3000.0f  /* 力矩比例增益, 可调 */
                float torque_error = motor_data[i].target_torque - motor_data[i].torque;
                motor_data[i].pwm_out = (int16_t)(torque_error * TORQUE_KP);
                break;
            }
            case MOTOR_MODE_SPEED: {
                float speed_error = motor_data[i].target_speed - motor_data[i].speed;
                motor_data[i].pwm_out = (int16_t)PID_Calculate(
                    speed_error, &speed_integrator[i],
                    SPEED_KP, SPEED_KI, SPEED_KD, dt, SPEED_OUT_MAX);
                break;
            }
            case MOTOR_MODE_POSITION: {
                float pos_error = motor_data[i].target_angle - motor_data[i].angle;
                float speed_cmd = PID_Calculate(
                    pos_error, &pos_integrator[i],
                    POS_KP, POS_KI, POS_KD, dt, POS_OUT_MAX);
                float speed_error = speed_cmd - motor_data[i].speed;
                motor_data[i].pwm_out = (int16_t)PID_Calculate(
                    speed_error, &speed_integrator[i],
                    SPEED_KP, SPEED_KI, SPEED_KD, dt, SPEED_OUT_MAX);
                break;
            }
            default:
                motor_data[i].pwm_out = 0;
                break;
        }
        DRV8870_SetDuty(i, motor_data[i].pwm_out);
    }
}

/* ==================== 使能/失能 ==================== */

/**
 * @brief  使能电机控制, 启动PWM输出
 */
void Motor_Ctrl_Enable(void)
{
    motor_enabled = 1;
    DRV8870_Start();
}

/**
 * @brief  失能电机控制, 制动所有电机, 清除PID积分
 */
void Motor_Ctrl_Disable(void)
{
    motor_enabled = 0;
    DRV8870_StopAll();
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        speed_integrator[i] = 0;
        pos_integrator[i] = 0;
    }
}

/* ==================== 反馈函数 ==================== */

/**
 * @brief  获取电机速度
 * @param  index  电机索引 [0, 3]
 * @return 速度 (rad/s), 越界返回0
 */
float Motor_GetSpeed(uint8_t index)
{
    return (index < MOTOR_NUM) ? motor_data[index].speed : 0;
}

/**
 * @brief  获取电机角度
 * @param  index  电机索引 [0, 3]
 * @return 角度 (rad), 越界返回0
 */
float Motor_GetAngle(uint8_t index)
{
    return (index < MOTOR_NUM) ? motor_data[index].angle : 0;
}

/**
 * @brief  获取电动力矩
 * @param  index  电机索引 [0, 3]
 * @return 力矩 (N·m), 越界返回0
 */
float Motor_GetTorque(uint8_t index)
{
    return (index < MOTOR_NUM) ? motor_data[index].torque : 0;
}

/**
 * @brief  获取电机电流
 * @param  index  电机索引 [0, 3]
 * @return 电流 (A), 越界返回0
 */
float Motor_GetCurrent(uint8_t index)
{
    return (index < MOTOR_NUM) ? motor_data[index].current : 0;
}

/* ==================== 控制命令函数 ==================== */

/**
 * @brief  设置恒力矩模式并指定目标力矩
 * @param  index   电机索引 [0, 3]
 * @param  torque  目标力矩 (N·m)
 */
void Motor_SetTorque(uint8_t index, float torque)
{
    if (index >= MOTOR_NUM) return;
    motor_data[index].target_torque = torque;
    motor_data[index].mode = MOTOR_MODE_TORQUE;
}

/**
 * @brief  设置恒速度模式并指定目标速度
 * @param  index  电机索引 [0, 3]
 * @param  speed  目标速度 (rad/s)
 */
void Motor_SetSpeed(uint8_t index, float speed)
{
    if (index >= MOTOR_NUM) return;
    motor_data[index].target_speed = speed;
    motor_data[index].mode = MOTOR_MODE_SPEED;
}

/**
 * @brief  设置恒位置模式并指定目标角度
 * @param  index  电机索引 [0, 3]
 * @param  angle  目标角度 (rad)
 */
void Motor_SetPosition(uint8_t index, float angle)
{
    if (index >= MOTOR_NUM) return;
    motor_data[index].target_angle = angle;
    motor_data[index].mode = MOTOR_MODE_POSITION;
}

/**
 * @brief  设置电机控制模式 (不改变目标值)
 * @param  index  电机索引 [0, 3]
 * @param  mode   控制模式
 */
void Motor_SetMode(uint8_t index, Motor_Mode_e mode)
{
    if (index >= MOTOR_NUM) return;
    motor_data[index].mode = mode;
}

/**
 * @brief  直接设置PWM值 (调试用)
 * @param  index  电机索引 [0, 3]
 * @param  pwm    PWM值 [-max_duty, +max_duty]
 * @note   会将模式切换为DISABLE, 避免被控制循环覆盖
 */
void Motor_SetPWM(uint8_t index, int16_t pwm)
{
    if (index >= MOTOR_NUM) return;
    motor_data[index].mode = MOTOR_MODE_DISABLE;
    DRV8870_SetDuty(index, pwm);
}
