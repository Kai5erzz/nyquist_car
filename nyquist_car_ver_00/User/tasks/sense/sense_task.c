/**
 * @file    sense_task.c
 * @author  kaiser
 * @version V4.0.0
 * @date    2026-05-03
 * @brief   巡线 + 直角弯角度环融合
 *
 * 流程:
 *   巡线 → 检测到左/右弯 → 角度环转80° → 回到巡线
 *   巡线 → 检测到十字 → 停车
 */

#include "sense_task.h"
#include "cmsis_os.h"
#include "rm_task.h"
#include "robot.h"
#include "drv/drv8870/motor_ctrl.h"
#include "drv/grayscale/grayscale.h"
#include "drv/line_follow/line_follow.h"
#include "drv/bmi088/bmi088.h"
#include "drv/vofa/vofa_plus.h"
#include "usart.h"
#include "main.h"

/* ==================== 巡线参数 ==================== */
#define LINE_DURATION_MS  10000   /* 巡线总时长 (ms) */

/* ==================== 角度闭环PID参数 ==================== */
#define ANGLE_KP          15.0f
#define ANGLE_KD          1.0f
#define ANGLE_OUT_MAX     150.0f   /* 输出限幅 (rad/s) */
#define ANGLE_TARGET_DEG  80.0f    /* 直角弯目标角度 (不必转满90°) */
#define ANGLE_ERR_THRESH  3.0f    /* 角度误差阈值 (deg), 小于此值认为转完 */
#define ANGLE_TIMEOUT_MS  2000    /* 角度环超时保护 (ms) */
#define TURN_COOLDOWN_MS  1500    /* 弯道检测冷却时间 (ms), 防止重复触发 */

/* ==================== 校准参数 ==================== */
#define CAL_SAMPLE_COUNT  100
#define CAL_SETTLE_MS     2000
#define BEEP_DURATION_MS  200

/* ==================== 状态机 ==================== */
typedef enum {
    STATE_IDLE = 0,
    STATE_CAL_WAIT_BLACK,
    STATE_CAL_BLACK,
    STATE_CAL_BEEP1,
    STATE_CAL_WAIT_WHITE,
    STATE_CAL_WHITE,
    STATE_CAL_BEEP2,
    STATE_CAL_DONE,
    STATE_LINE_FOLLOWING,
    STATE_ANGLE_TURNING,
    STATE_LINE_STOP,
} SystemState_e;

/* ==================== 全局调试数据 ==================== */
struct {
    /* 巡线 */
    float line_error;
    float left_speed;
    float right_speed;
    uint8_t gray_digital;
    uint8_t line_result;
    /* 角度环 */
    float target_angle;
    float current_angle;
    float angle_error;
    float pid_output;
    SystemState_e state;
    /* IMU */
    float yaw;
    float yaw_total;
    float gyro_z;
} debug_data;

/* ==================== Task Handle ==================== */
osThreadId_t senseTaskHandle;

/* ==================== VOFA+ ==================== */
static Vofa_Instance_t sense_vofa;

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
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    osDelay(duration_ms);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}

static uint16_t cal_min[GRAYSCALE_CH_NUM];
static uint16_t cal_max[GRAYSCALE_CH_NUM];

static const float motor_dir_sign[MOTOR_NUM] = {
    -1.0f,   /* MOTOR1: 取反 */
     1.0f,   /* MOTOR2: 正常 */
    -1.0f,   /* MOTOR3: 取反 */
    -1.0f,   /* MOTOR4: 取反 */
};

/* ==================== 角度PID ==================== */
static float angle_prev_error = 0;
static float angle_target = 0;       /* 当前角度目标 (deg, 累加坐标系) */
static int8_t turn_sign = 0;         /* +1=左转, -1=右转 */
static uint32_t last_turn_tick = 0;  /* 上次转弯完成的时间戳, 用于冷却 */

static void AnglePID_Reset(float target)
{
    angle_target = target;
    angle_prev_error = 0;
}

static float AnglePID_Calc(float current)
{
    float error = angle_target - current;
    float d_error = error - angle_prev_error;
    angle_prev_error = error;

    float output = ANGLE_KP * error + ANGLE_KD * d_error;
    if (output >  ANGLE_OUT_MAX) output =  ANGLE_OUT_MAX;
    if (output < -ANGLE_OUT_MAX) output = -ANGLE_OUT_MAX;
    return output;
}

/* ==================== 设置差速 (角度环用) ==================== */
static void SetDifferentialSpeed(float turn)
{
    float left  = -turn;
    float right = turn;

    debug_data.left_speed = left;
    debug_data.right_speed = right;

    Motor_SetSpeed(0, right * motor_dir_sign[0]);
    Motor_SetSpeed(1, right * motor_dir_sign[1]);
    Motor_SetSpeed(2, left  * motor_dir_sign[2]);
    Motor_SetSpeed(3, left  * motor_dir_sign[3]);
}

/* ==================== 设置轮速 (巡线用) ==================== */
static void SetWheelSpeed(float left, float right)
{
    debug_data.left_speed = left;
    debug_data.right_speed = right;

    Motor_SetSpeed(0, right * motor_dir_sign[0]);
    Motor_SetSpeed(1, right * motor_dir_sign[1]);
    Motor_SetSpeed(2, left  * motor_dir_sign[2]);
    Motor_SetSpeed(3, left  * motor_dir_sign[3]);
}

/* ==================== VOFA+ 发送 ==================== */
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
    uint32_t tick_ms = 0;          /* 自累加时间戳 (ms) */
    uint32_t state_tick = 0;
    uint32_t angle_start_tick = 0;

    for (;;) {
        uint32_t now = tick_ms;

        /* 更新IMU数据 */
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
                    state = STATE_CAL_WAIT_BLACK;
                    state_tick = now;
                }
            }
            break;

        /* ==================== 校准流程 ==================== */
        case STATE_CAL_WAIT_BLACK:
            LED_SetAll(0, 1, 0, 0);
            if (now - state_tick >= CAL_SETTLE_MS) {
                for (uint8_t i = 0; i < GRAYSCALE_CH_NUM; i++)
                    cal_min[i] = 65535;
                state = STATE_CAL_BLACK;
            }
            break;

        case STATE_CAL_BLACK:
            LED_SetAll(0, 1, 1, 0);
            for (uint16_t n = 0; n < CAL_SAMPLE_COUNT; n++) {
                for (uint8_t i = 0; i < GRAYSCALE_CH_NUM; i++) {
                    uint16_t s = Grayscale_ReadChannel(i);
                    if (s < cal_min[i]) cal_min[i] = s;
                }
                osDelay(1);
            }
            state = STATE_CAL_BEEP1;
            break;

        case STATE_CAL_BEEP1:
            Beep(BEEP_DURATION_MS);
            state = STATE_CAL_WAIT_WHITE;
            state_tick = now;
            break;

        case STATE_CAL_WAIT_WHITE:
            LED_SetAll(0, 0, 1, 0);
            if (now - state_tick >= CAL_SETTLE_MS) {
                for (uint8_t i = 0; i < GRAYSCALE_CH_NUM; i++)
                    cal_max[i] = 0;
                state = STATE_CAL_WHITE;
            }
            break;

        case STATE_CAL_WHITE:
            LED_SetAll(0, 0, 1, 1);
            for (uint16_t n = 0; n < CAL_SAMPLE_COUNT; n++) {
                for (uint8_t i = 0; i < GRAYSCALE_CH_NUM; i++) {
                    uint16_t s = Grayscale_ReadChannel(i);
                    if (s > cal_max[i]) cal_max[i] = s;
                }
                osDelay(1);
            }
            for (uint8_t i = 0; i < GRAYSCALE_CH_NUM; i++) {
                if (cal_max[i] <= cal_min[i])
                    cal_max[i] = cal_min[i] + 1;
                grayscale.cal_min[i] = cal_min[i];
                grayscale.cal_max[i] = cal_max[i];
            }
            grayscale.is_calibrated = 1;
            state = STATE_CAL_BEEP2;
            break;

        case STATE_CAL_BEEP2:
            Beep(BEEP_DURATION_MS);
            LED_SetAll(1, 0, 0, 0);
            state = STATE_CAL_DONE;
            break;

        case STATE_CAL_DONE:
            if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_SET) {
                osDelay(50);
                if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_SET) {
                    Motor_Ctrl_Enable();
                    LineFollow_Init();
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
            SetWheelSpeed(left, right);

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

            /* 左弯道 → 角度环左转 (冷却期内不触发) */
            if (result == LINE_LEFT_TURN && (now - last_turn_tick >= TURN_COOLDOWN_MS)) {
                turn_sign = +1;
                AnglePID_Reset(imu_data.yaw_total + ANGLE_TARGET_DEG);
                angle_start_tick = now;
                debug_data.target_angle = angle_target;
                state = STATE_ANGLE_TURNING;
            }
            /* 右弯道 → 角度环右转 (冷却期内不触发) */
            else if (result == LINE_RIGHT_TURN && (now - last_turn_tick >= TURN_COOLDOWN_MS)) {
                turn_sign = -1;
                AnglePID_Reset(imu_data.yaw_total - ANGLE_TARGET_DEG);
                angle_start_tick = now;
                debug_data.target_angle = angle_target;
                state = STATE_ANGLE_TURNING;
            }
            /* 十字路口 → 停车 */
            else if (result == LINE_CROSS) {
                state = STATE_LINE_STOP;
            }
            /* 巡线时间到 */
            else if (now - state_tick >= LINE_DURATION_MS) {
                state = STATE_LINE_STOP;
            }
            break;
        }

        /* ==================== 角度环转弯 ==================== */
        case STATE_ANGLE_TURNING: {
            debug_data.current_angle = imu_data.yaw_total;
            debug_data.angle_error = angle_target - imu_data.yaw_total;

            float turn = AnglePID_Calc(imu_data.yaw_total);
            debug_data.pid_output = turn;
            SetDifferentialSpeed(turn);

            LED_SetAll(
                (turn_sign > 0) ? 1 : 0,
                (turn_sign > 0) ? 1 : 0,
                (turn_sign < 0) ? 1 : 0,
                (turn_sign < 0) ? 1 : 0);

            /* 转到位 (误差 < 阈值) → 回到巡线 */
            if (fabsf(debug_data.angle_error) < ANGLE_ERR_THRESH) {
                last_turn_tick = now;
                LineFollow_Init();
                state = STATE_LINE_FOLLOWING;
            }
            /* 超时保护 */
            else if (now - angle_start_tick >= ANGLE_TIMEOUT_MS) {
                last_turn_tick = now;
                LineFollow_Init();
                state = STATE_LINE_FOLLOWING;
            }
            break;
        }

        /* ==================== 停车 ==================== */
        case STATE_LINE_STOP:
            Motor_Ctrl_Disable();
            SetWheelSpeed(0, 0);
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

    const osThreadAttr_t sense_task_attributes = {
        .name = "sense_task",
        .stack_size = SENSE_TASK_STACK_SIZE,
        .priority = (osPriority_t) SENSE_TASK_PRIORITY,
    };

    senseTaskHandle = osThreadNew(sense_task_entry, NULL, &sense_task_attributes);
}
