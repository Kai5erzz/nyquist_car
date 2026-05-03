/**
 * @file    sense_task.c
 * @author  kaiser
 * @version V5.0.0
 * @date    2026-05-03
 * @brief   巡线 + 直角弯角度环融合 (模块化版本)
 *
 * 流程:
 *   校准 → 巡线 → 检测到弯道 → 角度环转70° → 回到巡线
 *   巡线 → 检测到十字 → 停车
 */

#include "sense_task.h"
#include "cmsis_os.h"
#include "rm_task.h"
#include "robot.h"
#include "drv/drv8870/motor_ctrl.h"
#include "drv/grayscale/grayscale.h"
#include "drv/line_follow/line_follow.h"
#include "drv/angle_ctrl/angle_ctrl.h"
#include "drv/bmi088/bmi088.h"
#include "drv/vofa/vofa_plus.h"
#include "usart.h"
#include "main.h"

/* ==================== 参数 ==================== */
#define LINE_DURATION_MS  10000   /* 巡线总时长 (ms) */
#define ANGLE_TARGET_DEG  80.0f   /* 直角弯目标角度 */
#define ANGLE_ERR_THRESH  3.0f    /* 角度误差阈值 (deg) */
#define ANGLE_TIMEOUT_MS  2000    /* 角度环超时 (ms) */
#define TURN_COOLDOWN_MS  1500    /* 弯道检测冷却 (ms) */

/* ==================== 状态机 ==================== */
typedef enum {
    STATE_IDLE = 0,
    STATE_CALIBRATING,
    STATE_CAL_DONE,
    STATE_LINE_FOLLOWING,
    STATE_ANGLE_TURNING,
    STATE_LINE_STOP,
} SystemState_e;

/* ==================== 调试数据 ==================== */
struct {
    float line_error;
    float left_speed;
    float right_speed;
    uint8_t gray_digital;
    uint8_t line_result;
    float target_angle;
    float current_angle;
    float angle_error;
    float pid_output;
    SystemState_e state;
    float yaw;
    float yaw_total;
    float gyro_z;
} debug_data;

/* ==================== Task Handle ==================== */
osThreadId_t senseTaskHandle;

/* ==================== 模块实例 ==================== */
static Vofa_Instance_t sense_vofa;
static AngleCtrl_t angle_ctrl;

static const float motor_dir_sign[MOTOR_NUM] = {
    -1.0f, 1.0f, -1.0f, -1.0f
};

static int8_t turn_sign = 0;
static uint32_t last_turn_tick = 0;

/* ==================== LED & 蜂鸣器 ==================== */
static void LED_SetAll(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3)
{
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, s0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, s1 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, s2 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, s3 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void Beep(uint32_t duration_ms)
{
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    LED_SetAll(0, 0, 0, 1);
    osDelay(duration_ms);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    LED_SetAll(0, 0, 0, 0);
}

/* ==================== 电机控制封装 ==================== */
static void SetMotorSpeed(float left, float right)
{
    debug_data.left_speed = left;
    debug_data.right_speed = right;
    Motor_SetSpeed(0, right * motor_dir_sign[0]);
    Motor_SetSpeed(1, right * motor_dir_sign[1]);
    Motor_SetSpeed(2, left  * motor_dir_sign[2]);
    Motor_SetSpeed(3, left  * motor_dir_sign[3]);
}

/* ==================== VOFA+ ==================== */
static void Debug_VofaSend(void)
{
    Vofa_SetData(&sense_vofa, 0,  debug_data.line_error);
    Vofa_SetData(&sense_vofa, 1,  debug_data.left_speed);
    Vofa_SetData(&sense_vofa, 2,  debug_data.right_speed);
    Vofa_SetData(&sense_vofa, 3,  (float)debug_data.line_result);
    Vofa_SetData(&sense_vofa, 4,  debug_data.target_angle);
    Vofa_SetData(&sense_vofa, 5,  debug_data.current_angle);
    Vofa_SetData(&sense_vofa, 6,  debug_data.angle_error);
    Vofa_SetData(&sense_vofa, 7,  debug_data.pid_output);
    Vofa_SetData(&sense_vofa, 8,  debug_data.yaw_total);
    Vofa_SetData(&sense_vofa, 9,  (float)debug_data.state);
    Vofa_Transmit(&sense_vofa, &huart2);
}

/* ==================== 任务入口 ==================== */
__attribute__((noreturn))
void sense_task_entry(void *argument)
{
    SystemState_e state = STATE_IDLE;
    uint32_t tick_ms = 0;
    uint32_t state_tick = 0;
    uint32_t angle_start_tick = 0;

    for (;;) {
        uint32_t now = tick_ms;

        debug_data.yaw       = imu_data.yaw;
        debug_data.yaw_total = imu_data.yaw_total;
        debug_data.gyro_z    = imu_data.gyro[2];

        switch (state) {
        /* ==================== IDLE ==================== */
        case STATE_IDLE:
            Motor_Ctrl_Disable();
            LED_SetAll(0, 0, 0, 0);
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
            if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_SET) {
                osDelay(50);
                if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_SET) {
                    Beep(200);
                    state = STATE_CALIBRATING;
                    state_tick = now;
                }
            }
            break;

        /* ==================== 校准 (调用 grayscale 库) ==================== */
        case STATE_CALIBRATING:
            LED_SetAll(0, 1, 0, 0);
            Grayscale_Calibrate();  /* 阻塞 ~5s, 内含采样逻辑 */
            Beep(200);
            LED_SetAll(1, 0, 0, 0);
            state = STATE_CAL_DONE;
            break;

        /* ==================== 等待启动巡线 ==================== */
        case STATE_CAL_DONE:
            if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_SET) {
                osDelay(50);
                if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_SET) {
                    Motor_Ctrl_Enable();
                    LineFollow_Init();
                    AngleCtrl_Init(&angle_ctrl);
                    imu_data.yaw_total = 0;
                    state_tick = now;
                    state = STATE_LINE_FOLLOWING;
                }
            }
            break;

        /* ==================== 巡线 ==================== */
        case STATE_LINE_FOLLOWING: {
            Grayscale_ReadAll();
            debug_data.gray_digital = Grayscale_GetDigitalByte();
            debug_data.line_error = Grayscale_GetLineError();

            LineFollow_Update(debug_data.gray_digital, debug_data.line_error);
            LineFollow_Result_e result = LineFollow_GetResult();
            debug_data.line_result = (uint8_t)result;

            float left, right;
            LineFollow_GetSpeed(&left, &right);
            SetMotorSpeed(left, right);

            /* LED指示 */
            switch (result) {
                case LINE_LEFT_TURN:  LED_SetAll(1, 1, 0, 0); break;
                case LINE_RIGHT_TURN: LED_SetAll(0, 0, 1, 1); break;
                case LINE_CROSS:      LED_SetAll(1, 1, 1, 1); break;
                case LINE_LOST:       LED_SetAll(0, 0, 0, 0); break;
                default: {
                    float e = debug_data.line_error;
                    LED_SetAll(1, (e < -0.3f) ? 1 : 0,
                               (e >  0.3f) ? 1 : 0,
                               (e >= -0.3f && e <= 0.3f) ? 1 : 0);
                    break;
                }
            }

            /* 弯道检测 (带冷却) */
            if (result == LINE_LEFT_TURN && (now - last_turn_tick >= TURN_COOLDOWN_MS)) {
                turn_sign = +1;
                AngleCtrl_SetTarget(&angle_ctrl, imu_data.yaw_total + ANGLE_TARGET_DEG);
                debug_data.target_angle = angle_ctrl.target;
                angle_start_tick = now;
                state = STATE_ANGLE_TURNING;
            }
            else if (result == LINE_RIGHT_TURN && (now - last_turn_tick >= TURN_COOLDOWN_MS)) {
                turn_sign = -1;
                AngleCtrl_SetTarget(&angle_ctrl, imu_data.yaw_total - ANGLE_TARGET_DEG);
                debug_data.target_angle = angle_ctrl.target;
                angle_start_tick = now;
                state = STATE_ANGLE_TURNING;
            }
            else if (result == LINE_CROSS) {
                state = STATE_LINE_STOP;
            }
            else if (now - state_tick >= LINE_DURATION_MS) {
                state = STATE_LINE_STOP;
            }
            break;
        }

        /* ==================== 角度环转弯 ==================== */
        case STATE_ANGLE_TURNING: {
            float turn = AngleCtrl_Update(&angle_ctrl, imu_data.yaw_total);
            debug_data.pid_output = turn;
            debug_data.current_angle = imu_data.yaw_total;
            debug_data.angle_error = angle_ctrl.error;

            float left, right;
            AngleCtrl_ToWheelSpeed(turn, &left, &right);
            SetMotorSpeed(left, right);

            LED_SetAll(
                (turn_sign > 0) ? 1 : 0,
                (turn_sign > 0) ? 1 : 0,
                (turn_sign < 0) ? 1 : 0,
                (turn_sign < 0) ? 1 : 0);

            if (AngleCtrl_IsDone(&angle_ctrl, ANGLE_ERR_THRESH) ||
                (now - angle_start_tick >= ANGLE_TIMEOUT_MS)) {
                last_turn_tick = now;
                LineFollow_Init();
                state = STATE_LINE_FOLLOWING;
            }
            break;
        }

        /* ==================== 停车 ==================== */
        case STATE_LINE_STOP:
            Motor_Ctrl_Disable();
            SetMotorSpeed(0, 0);
            LED_SetAll(1, 1, 1, 1);
            osDelay(1000);
            LED_SetAll(0, 0, 0, 0);
            state = STATE_IDLE;
            break;
        }

        debug_data.state = state;
        Debug_VofaSend();

        tick_ms += SENSE_TASK_PERIOD;
        osDelay(SENSE_TASK_PERIOD);
    }
}

/* ==================== 任务初始化 ==================== */
void sense_task_init(void)
{
    Vofa_Init(&sense_vofa, 10);
    Grayscale_Init();
    LineFollow_Init();
    AngleCtrl_Init(&angle_ctrl);

    const osThreadAttr_t sense_task_attributes = {
        .name = "sense_task",
        .stack_size = SENSE_TASK_STACK_SIZE,
        .priority = (osPriority_t) SENSE_TASK_PRIORITY,
    };

    senseTaskHandle = osThreadNew(sense_task_entry, NULL, &sense_task_attributes);
}
