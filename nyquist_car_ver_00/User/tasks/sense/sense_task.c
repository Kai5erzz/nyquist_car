/**
 * @file    sense_task.c
 * @author  kaiser
 * @version V1.3.0
 * @date    2026-05-01
 * @brief   MOTOR2 电流/力矩闭环调试任务
 *
 * VOFA+ 8通道:
 *   CH0: 目标力矩 (N·m)
 *   CH1: 实际力矩 (N·m)
 *   CH2: 实际电流 (A)
 *   CH3: PWM输出值
 *   CH4: 力矩误差 (N·m)
 *   CH5: ADC原始值
 *   CH6: ADC零偏值
 *   CH7: TIM6中断计数
 *
 * 周期: 10ms (100Hz)
 */

#include "sense_task.h"
#include "cmsis_os.h"
#include "rm_task.h"
#include "robot.h"
#include "drv/drv8870/motor_ctrl.h"
#include "drv/drv8870/current_sense.h"
#include "drv/vofa/vofa_plus.h"
#include "usart.h"
#include "tim.h"

/* ==================== MOTOR2 目标力矩 (可手动修改) ==================== */
#define MOTOR2_TARGET_TORQUE  0.01f  /* N·m */

/* ==================== 全局调试数据 ==================== */
struct {
    float target_torque;       /**< 目标力矩 (N·m) */
    float actual_torque;       /**< 实际力矩 (N·m) */
    float actual_current;      /**< 实际电流 (A) */
    int16_t pwm_out;           /**< PWM输出值 */
    float torque_error;        /**< 力矩误差 (N·m) */
    uint16_t adc_raw;          /**< ADC原始值 */
    int16_t adc_offset;        /**< ADC零偏值 */
    volatile uint32_t tim6_cnt;/**< TIM6中断计数 */
} debug_data;

/* ==================== Task Handle ==================== */
osThreadId_t senseTaskHandle;

/* ==================== VOFA+ ==================== */
static Vofa_Instance_t sense_vofa;

/* ==================== TIM6中断计数器 ==================== */
volatile uint32_t tim6_irq_count = 0;

/* ==================== 任务入口 ==================== */

__attribute__((noreturn))
void sense_task_entry(void *argument)
{
    uint32_t wake_time = osKernelSysTick();

    /* 使能电机控制, 设置MOTOR2恒力矩模式 */
    Motor_Ctrl_Enable();
    Motor_SetTorque(1, MOTOR2_TARGET_TORQUE);

    for (;;) {
        debug_data.target_torque = MOTOR2_TARGET_TORQUE;
        debug_data.actual_torque = Motor_GetTorque(1);
        debug_data.actual_current = Motor_GetCurrent(1);
        debug_data.pwm_out       = motor_data[1].pwm_out;
        debug_data.torque_error  = debug_data.target_torque - debug_data.actual_torque;
        debug_data.adc_raw       = current_sense.adc_raw[1];
        debug_data.adc_offset    = current_sense.offset[1];
        debug_data.tim6_cnt      = tim6_irq_count;

        Vofa_SetData(&sense_vofa, 0, debug_data.target_torque);
        Vofa_SetData(&sense_vofa, 1, debug_data.actual_torque);
        Vofa_SetData(&sense_vofa, 2, debug_data.actual_current);
        Vofa_SetData(&sense_vofa, 3, (float)debug_data.pwm_out);
        Vofa_SetData(&sense_vofa, 4, debug_data.torque_error);
        Vofa_SetData(&sense_vofa, 5, (float)debug_data.adc_raw);
        Vofa_SetData(&sense_vofa, 6, (float)debug_data.adc_offset);
        Vofa_SetData(&sense_vofa, 7, (float)debug_data.tim6_cnt);
        Vofa_Transmit(&sense_vofa, &huart1);

        vTaskDelayUntil(&wake_time, SENSE_TASK_PERIOD);
    }
}

/* ==================== 任务初始化 ==================== */

void sense_task_init(void)
{
    Vofa_Init(&sense_vofa, 8);

    const osThreadAttr_t sense_task_attributes = {
        .name = "sense_task",
        .stack_size = SENSE_TASK_STACK_SIZE,
        .priority = (osPriority_t) SENSE_TASK_PRIORITY,
    };

    senseTaskHandle = osThreadNew(sense_task_entry, NULL, &sense_task_attributes);
}
