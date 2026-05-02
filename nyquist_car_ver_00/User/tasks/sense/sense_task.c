/**
 * @file    sense_task.c
 * @author  kaiser
 * @version V1.4.0
 * @date    2026-05-01
 * @brief   四电机速度闭环调试任务
 *
 * VOFA+ 8通道:
 *   CH0~3: 4个电机实际速度 (rad/s)
 *   CH4~7: 4个电机PWM输出
 *
 * Watch窗口: debug_data 查看全部数据
 * 周期: 10ms (100Hz)
 */

#include "sense_task.h"
#include "cmsis_os.h"
#include "rm_task.h"
#include "robot.h"
#include "drv/drv8870/motor_ctrl.h"
#include "drv/vofa/vofa_plus.h"
#include "usart.h"
#include "tim.h"
#include "lptim.h"

/* ==================== 四电机目标速度 (可手动修改) ==================== */
#define MOTOR1_TARGET_SPEED  20.0f   /* rad/s */
#define MOTOR2_TARGET_SPEED  -20.0f   /* rad/s */
#define MOTOR3_TARGET_SPEED  20.0f   /* rad/s */
#define MOTOR4_TARGET_SPEED  20.0f   /* rad/s */

/* ==================== 全局调试数据 ==================== */
struct {
    float target_speed[4];    /**< 目标速度 (rad/s) */
    float actual_speed[4];    /**< 实际速度 (rad/s) */
    float speed_error[4];     /**< 速度误差 (rad/s) */
    int16_t pwm_out[4];       /**< PWM输出值 */
    float current[4];         /**< 电流 (A) */
    float angle[4];           /**< 角度 (rad) */
    int16_t enc_raw[4];       /**< 编码器原始计数 */
    volatile uint32_t tim6_cnt;
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

    Motor_Ctrl_Enable();
    Motor_SetSpeed(0, MOTOR1_TARGET_SPEED);
    Motor_SetSpeed(1, MOTOR2_TARGET_SPEED);
    Motor_SetSpeed(2, MOTOR3_TARGET_SPEED);
    Motor_SetSpeed(3, MOTOR4_TARGET_SPEED);

    for (;;) {
        /* 采集四电机数据 */
        debug_data.target_speed[0] = MOTOR1_TARGET_SPEED;
        debug_data.target_speed[1] = MOTOR2_TARGET_SPEED;
        debug_data.target_speed[2] = MOTOR3_TARGET_SPEED;
        debug_data.target_speed[3] = MOTOR4_TARGET_SPEED;

        for (uint8_t i = 0; i < 4; i++) {
            debug_data.actual_speed[i] = Motor_GetSpeed(i);
            debug_data.speed_error[i]  = debug_data.target_speed[i] - debug_data.actual_speed[i];
            debug_data.pwm_out[i]      = motor_data[i].pwm_out;
            debug_data.current[i]      = Motor_GetCurrent(i);
            debug_data.angle[i]        = Motor_GetAngle(i);
        }

        debug_data.enc_raw[0] = (int16_t)(hlptim1.Instance->CNT);
        debug_data.enc_raw[1] = (int16_t)(hlptim2.Instance->CNT);
        debug_data.enc_raw[2] = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        debug_data.enc_raw[3] = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
        debug_data.tim6_cnt   = tim6_irq_count;

        /* VOFA+ 12通道: 4 target + 4 actual + 4 current */
        Vofa_SetData(&sense_vofa, 0,  debug_data.target_speed[0]);
        Vofa_SetData(&sense_vofa, 1,  debug_data.target_speed[1]);
        Vofa_SetData(&sense_vofa, 2,  debug_data.target_speed[2]);
        Vofa_SetData(&sense_vofa, 3,  debug_data.target_speed[3]);
        Vofa_SetData(&sense_vofa, 4,  debug_data.actual_speed[0]);
        Vofa_SetData(&sense_vofa, 5,  debug_data.actual_speed[1]);
        Vofa_SetData(&sense_vofa, 6,  debug_data.actual_speed[2]);
        Vofa_SetData(&sense_vofa, 7,  debug_data.actual_speed[3]);
        Vofa_SetData(&sense_vofa, 8,  debug_data.current[0]);
        Vofa_SetData(&sense_vofa, 9,  debug_data.current[1]);
        Vofa_SetData(&sense_vofa, 10, debug_data.current[2]);
        Vofa_SetData(&sense_vofa, 11, debug_data.current[3]);
        Vofa_Transmit(&sense_vofa, &huart1);

        vTaskDelayUntil(&wake_time, SENSE_TASK_PERIOD);
    }
}

/* ==================== 任务初始化 ==================== */

void sense_task_init(void)
{
    Vofa_Init(&sense_vofa, 12);

    const osThreadAttr_t sense_task_attributes = {
        .name = "sense_task",
        .stack_size = SENSE_TASK_STACK_SIZE,
        .priority = (osPriority_t) SENSE_TASK_PRIORITY,
    };

    senseTaskHandle = osThreadNew(sense_task_entry, NULL, &sense_task_attributes);
}
