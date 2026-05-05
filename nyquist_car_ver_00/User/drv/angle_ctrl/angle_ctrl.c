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

void AngleCtrl_SetTarget(AngleCtrl_t *ctrl, float target)
{
    ctrl->target = target;
    ctrl->prev_error = 0;
    /* 注意：切换目标时不要将 ctrl->output 清零，
       保留当前的速度可以利用斜坡函数实现平滑过渡（例如从巡线切过来时的初速度减速） */
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

void AngleCtrl_ToWheelSpeed(float turn, float *left, float *right)
{
    float back_comp = 0.0f;

    /* 判断是否为右转:
     * 右转需要左轮前进(正)、右轮后退(负)，因此此时 turn 必然是【负数】。
     */
    // if (turn < 0.0f) {
    //     /* turn 是负数，乘以正数 comp_factor 后依然是负数，代表向后的拉力 */
    //     // float comp_factor = 0.3f;  /* 比例补偿系数：根据速度动态调节倒车力 (推荐 0.1 ~ 0.3) */
    //     back_comp   = -50.0f; /* 固定死区补偿：只要一检测到右转，立刻给一个基础倒车力，防止起步前冲 */
    //     //
    //     // back_comp = (turn * comp_factor) + base_comp;
    // }

    *left  = back_comp - turn;
    *right = back_comp + turn;
}

