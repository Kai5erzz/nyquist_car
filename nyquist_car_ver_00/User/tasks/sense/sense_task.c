/**
 * @file    sense_task.c
 * @author  kaiser
 * @version V2.4.0
 * @date    2026-05-02
 * @brief   巡线主任务 (纯PD差速控制，大KP强制过弯)
 *
 * 操作流程:
 * 1. 按KEY0 → 灰度黑白校准 (先黑后白, 各响一声)
 * 2. 校准完成 → LED0常亮
 * 3. 按KEY2 → 开始10秒巡线
 */

#include "sense_task.h"
#include "cmsis_os.h"
#include "rm_task.h"
#include "robot.h"
#include "drv/drv8870/motor_ctrl.h"
#include "drv/grayscale/grayscale.h"
#include "drv/vofa/vofa_plus.h"
#include "usart.h"
#include "main.h"
#include <math.h>

/* ==================== 巡线参数 ==================== */
#define LINE_BASE_SPEED   80.0f   /* 基础速度 (rad/s) */
#define LINE_KP           120.0f  /* 巡线比例增益 (加大强制过弯) */
#define LINE_KD           4.0f    /* 巡线微分增益 */
#define LINE_DURATION_MS  10000   /* 巡线时长 (ms) */
#define SEARCH_SPEED      60.0f   /* 彻底丢线时的搜索旋转速度 */

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
    STATE_LINE_RUNNING,
    STATE_LINE_STOP,
} SystemState_e;

/* ==================== 全局调试数据 ==================== */
struct {
    float line_error;
    float base_speed;
    float left_speed;
    float right_speed;
    float motor_speed[4];
    uint8_t gray_digital;
    SystemState_e state;
    uint32_t line_timer;
    float prev_error;
} debug_data;

/* ==================== Task Handle ==================== */
osThreadId_t senseTaskHandle;

/* ==================== VOFA+ ==================== */
static Vofa_Instance_t sense_vofa;
volatile uint32_t tim6_irq_count = 0;

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

/* ==================== 巡线控制 ==================== */
static float last_error_sign = 1.0f;

static void LineFollow_Step(void)
{
    float error = debug_data.line_error;

    /* 记录最后一次明显偏差的方向，防止冲出赛道后不知道往哪找 */
    if (debug_data.gray_digital != 0 && fabsf(error) > 0.5f) {
        last_error_sign = (error > 0) ? 1.0f : -1.0f;
    }

    float d_error = error - debug_data.prev_error;
    debug_data.prev_error = error;

    float left_speed, right_speed;

    if (debug_data.gray_digital == 0) {
        /* 彻底丢线：用最后记忆的方向原地急转找线 */
        left_speed  =  SEARCH_SPEED * last_error_sign;
        right_speed = -SEARCH_SPEED * last_error_sign;
    } else {
        /* 纯 PD 差速控制，大 KP 强制过弯 */
        float turn = LINE_KP * error + LINE_KD * d_error;
        left_speed  = LINE_BASE_SPEED + turn;
        right_speed = LINE_BASE_SPEED - turn;
    }

    debug_data.left_speed  = left_speed;
    debug_data.right_speed = right_speed;

    Motor_SetSpeed(0, right_speed * motor_dir_sign[0]);
    Motor_SetSpeed(1, right_speed * motor_dir_sign[1]);
    Motor_SetSpeed(2, left_speed  * motor_dir_sign[2]);
    Motor_SetSpeed(3, left_speed  * motor_dir_sign[3]);

    debug_data.motor_speed[0] = Motor_GetSpeed(0);
    debug_data.motor_speed[1] = Motor_GetSpeed(1);
    debug_data.motor_speed[2] = Motor_GetSpeed(2);
    debug_data.motor_speed[3] = Motor_GetSpeed(3);
}

/* ==================== VOFA+ 发送 ==================== */
static void Debug_VofaSend(void)
{
    Vofa_SetData(&sense_vofa, 0,  debug_data.line_error);
    Vofa_SetData(&sense_vofa, 1,  debug_data.left_speed);
    Vofa_SetData(&sense_vofa, 2,  debug_data.right_speed);
    Vofa_SetData(&sense_vofa, 3,  debug_data.motor_speed[0]);
    Vofa_SetData(&sense_vofa, 4,  debug_data.motor_speed[1]);
    Vofa_SetData(&sense_vofa, 5,  debug_data.motor_speed[2]);
    Vofa_SetData(&sense_vofa, 6,  debug_data.motor_speed[3]);
    Vofa_SetData(&sense_vofa, 7,  (float)motor_data[0].pwm_out);
    Vofa_SetData(&sense_vofa, 8,  (float)motor_data[1].pwm_out);
    Vofa_SetData(&sense_vofa, 9,  (float)debug_data.gray_digital);
    Vofa_SetData(&sense_vofa, 10, (float)debug_data.state);
    Vofa_SetData(&sense_vofa, 11, (float)tim6_irq_count);
    Vofa_Transmit(&sense_vofa, &huart1);
}

/* ==================== 任务入口 ==================== */
__attribute__((noreturn))
void sense_task_entry(void *argument)
{
    SystemState_e state = STATE_IDLE;
    uint32_t state_tick = 0;

    debug_data.prev_error = 0;

    for (;;) {
        uint32_t now = osKernelGetTickCount();

        switch (state) {
        case STATE_IDLE:
            Motor_Ctrl_Disable();
            LED_SetAll(0, 0, 0, 0);
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
            if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_SET) {
                osDelay(50);
                if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_SET) {
                    state = STATE_CAL_WAIT_BLACK;
                    state_tick = osKernelGetTickCount();
                }
            }
            break;

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
            state_tick = osKernelGetTickCount();
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
                    debug_data.prev_error = 0;
                    state_tick = osKernelGetTickCount();
                    state = STATE_LINE_RUNNING;
                }
            }
            break;

        case STATE_LINE_RUNNING:
            Grayscale_ReadAll();
            debug_data.line_error = Grayscale_GetLineError();
            debug_data.gray_digital = Grayscale_GetDigitalByte();
            LineFollow_Step();
            {
                float e = debug_data.line_error;
                LED_SetAll(1, (e < -0.3f) ? 1 : 0,
                           (e >  0.3f) ? 1 : 0,
                           (e >= -0.3f && e <= 0.3f) ? 1 : 0);
            }
            if (now - state_tick >= LINE_DURATION_MS) {
                state = STATE_LINE_STOP;
            }
            break;

        case STATE_LINE_STOP:
            Motor_Ctrl_Disable();
            LED_SetAll(1, 1, 1, 1);
            osDelay(1000);
            LED_SetAll(0, 0, 0, 0);
            state = STATE_IDLE;
            break;
        }

        debug_data.state = state;
        debug_data.line_timer = now - state_tick;
        Debug_VofaSend();

        osDelay(SENSE_TASK_PERIOD);
    }
}

/* ==================== 任务初始化 ==================== */
void sense_task_init(void)
{
    Vofa_Init(&sense_vofa, 12);
    Grayscale_Init();

    const osThreadAttr_t sense_task_attributes = {
        .name = "sense_task",
        .stack_size = SENSE_TASK_STACK_SIZE,
        .priority = (osPriority_t) SENSE_TASK_PRIORITY,
    };

    senseTaskHandle = osThreadNew(sense_task_entry, NULL, &sense_task_attributes);
}