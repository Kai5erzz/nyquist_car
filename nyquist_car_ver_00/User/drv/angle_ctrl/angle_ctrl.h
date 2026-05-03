/**
 * @file    angle_ctrl.h
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-05-03
 * @brief   角度闭环控制器 (PD + 差速输出)
 *
 * 用于直角弯等需要精确转向的场景。
 * 基于 yaw_total (累加角度) 做闭环，输出差速轮速。
 */

#ifndef ANGLE_CTRL_H
#define ANGLE_CTRL_H

#include <stdint.h>

/* ==================== 默认PID参数 ==================== */
#define ANGLE_KP_DEFAULT      15.0f
#define ANGLE_KD_DEFAULT      1.0f
#define ANGLE_OUT_MAX_DEFAULT 150.0f   /* 输出限幅 (rad/s) */

/* ==================== 控制器状态 ==================== */
typedef struct {
    /* 参数 */
    float kp;
    float kd;
    float out_max;
    /* 内部状态 */
    float target;          /* 目标角度 (deg, 累加坐标系) */
    float prev_error;
    /* 输出 */
    float error;           /* 当前误差 (deg) */
    float output;          /* PID输出 (rad/s) */
} AngleCtrl_t;

/* ==================== 接口函数 ==================== */

/**
 * @brief  初始化角度控制器 (使用默认参数)
 * @param  ctrl  控制器指针
 */
void AngleCtrl_Init(AngleCtrl_t *ctrl);

/**
 * @brief  设置目标角度并重置PID状态
 * @param  ctrl    控制器指针
 * @param  target  目标角度 (deg, 累加坐标系)
 */
void AngleCtrl_SetTarget(AngleCtrl_t *ctrl, float target);

/**
 * @brief  计算一步PID输出
 * @param  ctrl     控制器指针
 * @param  current  当前角度 (deg, yaw_total)
 * @return PID输出值 (rad/s), 已限幅
 */
float AngleCtrl_Update(AngleCtrl_t *ctrl, float current);

/**
 * @brief  判断是否到达目标
 * @param  ctrl     控制器指针
 * @param  thresh   误差阈值 (deg)
 * @return 1=到达, 0=未到达
 */
uint8_t AngleCtrl_IsDone(AngleCtrl_t *ctrl, float thresh);

/**
 * @brief  将PID输出转换为左右轮速 (差速)
 * @param  turn   PID输出值
 * @param  left   左轮速度输出指针
 * @param  right  右轮速度输出指针
 */
void AngleCtrl_ToWheelSpeed(float turn, float *left, float *right);

#endif /* ANGLE_CTRL_H */
