/**
 * @file    angle_ctrl.c
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-05-03
 * @brief   角度闭环控制器实现
 */

#include "angle_ctrl.h"
#include <math.h>

void AngleCtrl_Init(AngleCtrl_t *ctrl)
{
    ctrl->kp = ANGLE_KP_DEFAULT;
    ctrl->kd = ANGLE_KD_DEFAULT;
    ctrl->out_max = ANGLE_OUT_MAX_DEFAULT;
    ctrl->target = 0;
    ctrl->prev_error = 0;
    ctrl->error = 0;
    ctrl->output = 0;
}

void AngleCtrl_SetTarget(AngleCtrl_t *ctrl, float target)
{
    ctrl->target = target;
    ctrl->prev_error = 0;
}

float AngleCtrl_Update(AngleCtrl_t *ctrl, float current)
{
    float error = ctrl->target - current;
    float d_error = error - ctrl->prev_error;
    ctrl->prev_error = error;
    ctrl->error = error;

    float output = ctrl->kp * error + ctrl->kd * d_error;
    if (output >  ctrl->out_max) output =  ctrl->out_max;
    if (output < -ctrl->out_max) output = -ctrl->out_max;
    ctrl->output = output;

    return output;
}

uint8_t AngleCtrl_IsDone(AngleCtrl_t *ctrl, float thresh)
{
    return fabsf(ctrl->error) < thresh;
}

void AngleCtrl_ToWheelSpeed(float turn, float *left, float *right)
{
    *left  = -turn;
    *right =  turn;
}
