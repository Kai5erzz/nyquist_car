/**
 * @file    angle_ctrl.c
 * @author  kaiser
 * @version V1.1.0
 * @date    2026-05-03
 * @brief   角度闭环控制器实现 (带斜坡限制)
 */

#include "angle_ctrl.h"
#include <math.h>

void AngleCtrl_Init(AngleCtrl_t *ctrl)
{
    ctrl->kp = ANGLE_KP_DEFAULT;
    ctrl->kd = ANGLE_KD_DEFAULT;
    ctrl->out_max = ANGLE_OUT_MAX_DEFAULT;
    ctrl->max_step = ANGLE_STEP_MAX; /* 初始化斜坡步进值 */
    
    ctrl->target = 0;
    ctrl->prev_error = 0;
    ctrl->error = 0;
    ctrl->output = 0;
}



float AngleCtrl_Update(AngleCtrl_t *ctrl, float current)
{
    /* 1. 计算误差 */
    float error = ctrl->target - current;
    float d_error = error - ctrl->prev_error;
    ctrl->prev_error = error;
    ctrl->error = error;

    /* 2. 计算原始PD输出 */
    float target_output = ctrl->kp * error + ctrl->kd * d_error;
    
    /* 3. 对原始输出进行绝对限幅 */
    if (target_output >  ctrl->out_max) target_output =  ctrl->out_max;
    if (target_output < -ctrl->out_max) target_output = -ctrl->out_max;

    /* 4. 斜坡限制核心逻辑 (限制加速度，防止轮胎瞬间受力过大而打滑) */
    float out_diff = target_output - ctrl->output;
    
    if (out_diff > ctrl->max_step) {
        /* 如果期望速度比当前速度大很多，则只能以 max_step 的步进加速 */
        ctrl->output += ctrl->max_step; 
    } else if (out_diff < -ctrl->max_step) {
        /* 如果期望速度比当前速度小很多 (减速或反转)，则以 max_step 的步进反向变化 */
        ctrl->output -= ctrl->max_step; 
    } else {
        /* 如果变化量在允许的步进范围内，直接跟随期望值 */
        ctrl->output = target_output;   
    }

    return ctrl->output;
}

uint8_t AngleCtrl_IsDone(AngleCtrl_t *ctrl, float thresh)
{
    return fabsf(ctrl->error) < thresh;
}

void AngleCtrl_SetTarget(AngleCtrl_t *ctrl, float target)
{
    ctrl->target = target;
    ctrl->prev_error = 0;

    /* [修复] 必须清零！ctrl->output 是差速转向值。
       如果不清零，连续的同向弯道会瞬间满载输出，导致严重的单侧打滑和前冲 */
    ctrl->output = 0;
}

void AngleCtrl_ToWheelSpeed(AngleCtrl_t *ctrl, float *left, float *right)
{
    float back_comp = 0.0f;

    /* 动态误差刹车：仅针对 A、C 弯道 (目标误差 error < 0 时触发)
     * 刚入弯时误差最大(-80)，产生最大倒车力主动刹停直行惯性；
     * 随着转弯完成，误差趋零，刹车力平滑消失。
     */
    if (ctrl->error < 0.0f) {
        float brake_factor = 0.5f; /* 刹车系数，可根据底盘重量微调 (建议 1.5 ~ 2.5) */

        // error 为负数，乘出来就是强力的倒车补偿
        back_comp = ctrl->error * brake_factor;

        /* 限制最大刹车力，防止电机瞬间反接电流过载 */
        if (back_comp < -150.0f) {
            back_comp = -150.0f;
        }
    }

    /* ctrl->output 已经是经过斜坡处理的 turn 值 */
    *left  = back_comp - ctrl->output;
    *right = back_comp + ctrl->output;
}

