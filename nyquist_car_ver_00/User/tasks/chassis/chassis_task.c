/**
 * @file    chassis_task.c
 * @author  kaiser
 * @version V1.0.1
 * @date    2026-05-05
 * @brief   底盘控制任务 (巡线 + 直角弯角度环)
 *
 * 流程:
 * 校准 → 巡线 → 检测到弯道 → 角度环转80° → 回到巡线
 * 巡线 → 检测到十字 → 停车
 *
 * V1.0.1 修复:
 *  [FIX-1] 数字0/1的左右映射对调（A/B 转反）
 *  [FIX-2] CROSS_SKIP 也走 STATE_CROSS_FORWARD，避免十字横杠
 *          被误识别为弯道，导致 C/D 在第一个十字后直接飞车
 *  [FIX-3] SKIP 返回巡线时重置 LineFollow PID 并刷新 last_turn_tick
 *          抑制十字残留信号触发假弯道
 *  [FIX-4] CROSS_FORWARD_MS 350 → 250，避免十字过远再转弯导致脱线
 */

#include "chassis_task.h"
#include "cmsis_os.h"
#include "rm_task.h"
#include "uMCN.h"
#include "robot.h"
#include "drv/drv8870/motor_ctrl.h"
#include "drv/grayscale/grayscale.h"
#include "drv/line_follow/line_follow.h"
#include "drv/angle_ctrl/angle_ctrl.h"
#include "drv/bmi088/bmi088.h"
#include "drv/btb_cmd/uart_protocol_dma.h"
#include "usart.h"
#include "main.h"
#include <math.h>

/* ==================== 巡线参数 ==================== */
#define LINE_DURATION_MS  10000   /* 巡线总时长 (ms) */

/* ==================== 角度闭环参数 ==================== */
#define ANGLE_TARGET_DEG  80.0f   /* 改为80度：预留10度裕量，让巡线PID主动吸入线中，防止过冲脱线 */
#define ANGLE_ERR_THRESH  3.0f    /* 角度误差阈值 (deg) */
#define ANGLE_TIMEOUT_MS  2000    /* 角度环超时 (ms) */
#define TURN_COOLDOWN_MS  1500    /* 弯道检测冷却 (ms) */
#define TURN_FORWARD_MS   150     /* 弯道前直行时间 (ms) */
#define CROSS_COOLDOWN_MS 1000    /* 十字路口冷却 (ms) */
#define CROSS_FORWARD_MS  150     /* [FIX-4] 十字前进越线时间：100太短不过线，350太长会脱线，250 折中 */

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
    STATE_CROSS_FORWARD,
    STATE_ANGLE_TURNING,
    STATE_LINE_STOP,
} ChassisState_e;

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
    ChassisState_e state;
    float yaw;
    float yaw_total;
    float gyro_z;
    /* YOLO */
    uint8_t yolo_count;
    uint8_t yolo_digit0;
    float yolo_x0;
    float yolo_y0;
    uint8_t yolo_digit1;
    float yolo_x1;
    float yolo_y1;
    /* 十字路口调试 */
    int8_t dbg_target_digit;
    uint8_t dbg_crossroad_count;
    uint8_t dbg_cross_action;
    uint8_t dbg_line_started;
} chassis_dbg;

/* ==================== Task Handle ==================== */
osThreadId_t chassisTaskHandle;

/* ==================== 模块实例 ==================== */
static AngleCtrl_t angle_ctrl;

/* ==================== Topic ==================== */
MCN_DECLARE(chassis_fdb);
static struct chassis_fdb_msg chassis_fdb_data;

static const float motor_dir_sign[MOTOR_NUM] = {
    -1.0f, 1.0f, -1.0f, -1.0f
};

static int8_t turn_sign = 0;
static uint32_t last_turn_tick = 0;
static uint32_t last_cross_tick = 0;  /* 上次十字处理时间戳, 冷却用 */
static uint8_t forward_for_turn = 0;  /* 1=弯道前前进, 0=十字前前进 */

/* ==================== 十字路口决策 ==================== */
typedef enum {
    CROSS_TURN_NONE = 0,   /* 无待执行转弯 */
    CROSS_TURN_LEFT,       /* 待左转 */
    CROSS_TURN_RIGHT,      /* 待右转 */
    CROSS_SKIP,            /* 跳过当前十字 */
    CROSS_STOP,            /* 停车 */
} CrossAction_e;

static int8_t target_digit = -1;         /* YOLO目标数字 (-1=未收到) */
static uint8_t crossroad_count = 0;      /* 已遇到的十字路口数 */
static CrossAction_e cross_action = CROSS_TURN_NONE; /* 当前十字应执行的动作 */
static uint8_t line_started = 0;         /* 巡线已启动标志 */

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

/* ==================== 校准 ==================== */
static uint16_t cal_min[GRAYSCALE_CH_NUM];
static uint16_t cal_max[GRAYSCALE_CH_NUM];

/* ==================== 电机控制 ==================== */
static void SetMotorSpeed(float left, float right)
{
    chassis_dbg.left_speed = left;
    chassis_dbg.right_speed = right;
    Motor_SetSpeed(0, right * motor_dir_sign[0]);
    Motor_SetSpeed(1, right * motor_dir_sign[1]);
    Motor_SetSpeed(2, left  * motor_dir_sign[2]);
    Motor_SetSpeed(3, left  * motor_dir_sign[3]);
}

/* ==================== 任务入口 ==================== */
__attribute__((noreturn))
void chassis_task_entry(void *argument)
{
    ChassisState_e state = STATE_IDLE;
    uint32_t tick_ms = 0;
    uint32_t state_tick = 0;
    uint32_t angle_start_tick = 0;

    for (;;) {
        uint32_t now = tick_ms;

        /* 更新IMU数据 */
        chassis_dbg.yaw       = imu_data.yaw;
        chassis_dbg.yaw_total = imu_data.yaw_total;
        chassis_dbg.gyro_z    = imu_data.gyro[2];

        /* 接收YOLO数据 */
        if (Flag_NewDataReceived && RxPacket.cmd == BTB_CMD_YOLO_DETECT) {
            Yolo_ParseFromPacket();
            Flag_NewDataReceived = 0;
            chassis_dbg.yolo_count  = yolo_detect.count;
            chassis_dbg.yolo_digit0 = (yolo_detect.count > 0) ? yolo_detect.targets[0].digit : 0;
            chassis_dbg.yolo_x0     = (yolo_detect.count > 0) ? (float)yolo_detect.targets[0].x : 0;
            chassis_dbg.yolo_y0     = (yolo_detect.count > 0) ? (float)yolo_detect.targets[0].y : 0;
            chassis_dbg.yolo_digit1 = (yolo_detect.count > 1) ? yolo_detect.targets[1].digit : 0;
            chassis_dbg.yolo_x1     = (yolo_detect.count > 1) ? (float)yolo_detect.targets[1].x : 0;
            chassis_dbg.yolo_y1     = (yolo_detect.count > 1) ? (float)yolo_detect.targets[1].y : 0;
            /* 只在巡线启动前更新digit, KEY2后锁死 */
            if (!line_started && yolo_detect.count > 0) {
                target_digit = yolo_detect.targets[0].digit;
            }
        }

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
            LED_SetAll(
                (target_digit >= 0) ? ((target_digit >> 0) & 1) : 0,
                (target_digit >= 0) ? ((target_digit >> 1) & 1) : 0,
                0, 0);
            if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_SET) {
                osDelay(50);
                if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_SET) {
                    Motor_Ctrl_Enable();
                    LineFollow_Init();
                    AngleCtrl_Init(&angle_ctrl);
                    imu_data.yaw_total = 0;
                    crossroad_count = 0;
                    cross_action = CROSS_TURN_NONE;
                    line_started = 1;
                    last_cross_tick = now;
                    last_turn_tick  = now;   /* 同步刷新弯道冷却基准 */
                    state_tick = now;
                    state = STATE_LINE_FOLLOWING;
                }
            }
            break;

        /* ==================== 巡线 ==================== */
        case STATE_LINE_FOLLOWING: {
            Grayscale_ReadAll();
            chassis_dbg.gray_digital = Grayscale_GetDigitalByte();
            chassis_dbg.line_error = Grayscale_GetLineError();

            LineFollow_Update(chassis_dbg.gray_digital, chassis_dbg.line_error);
            LineFollow_Result_e result = LineFollow_GetResult();
            chassis_dbg.line_result = (uint8_t)result;

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
                    float e = chassis_dbg.line_error;
                    LED_SetAll(1, (e < -0.3f) ? 1 : 0,
                               (e >  0.3f) ? 1 : 0,
                               (e >= -0.3f && e <= 0.3f) ? 1 : 0);
                    break;
                }
            }

            /* 十字路口决策 (带冷却) */
            if (result == LINE_CROSS && (now - last_cross_tick >= CROSS_COOLDOWN_MS)) {
                crossroad_count++;
                chassis_dbg.dbg_target_digit = target_digit;
                chassis_dbg.dbg_crossroad_count = crossroad_count;
                chassis_dbg.dbg_line_started = line_started;

                if (target_digit < 0) {
                    cross_action = CROSS_SKIP;
                } else {
                    switch (target_digit) {
                        /* [FIX-1] 0/1 的左右映射对调：A/B 转反问题 */
                        case 0: cross_action = (crossroad_count == 1) ? CROSS_TURN_LEFT  : CROSS_STOP; break;
                        case 1: cross_action = (crossroad_count == 1) ? CROSS_TURN_RIGHT : CROSS_STOP; break;
                        case 2: cross_action = (crossroad_count == 1) ? CROSS_SKIP :
                                               (crossroad_count == 2) ? CROSS_TURN_LEFT : CROSS_STOP; break;
                        case 3: cross_action = (crossroad_count == 1) ? CROSS_SKIP :
                                               (crossroad_count == 2) ? CROSS_TURN_RIGHT : CROSS_STOP; break;
                        default: cross_action = CROSS_STOP; break;
                    }
                }

                last_cross_tick = now;
                chassis_dbg.dbg_cross_action = (uint8_t)cross_action;

                /* [FIX-2] 加入 CROSS_SKIP：SKIP 也要走一段直行越过十字横杠，
                 * 否则横杠会被巡线误识别为弯道，导致 C/D 在第一个十字后立刻飞车 */
                if (cross_action == CROSS_TURN_LEFT ||
                    cross_action == CROSS_TURN_RIGHT ||
                    cross_action == CROSS_STOP ||
                    cross_action == CROSS_SKIP) {
                    forward_for_turn = 0;
                    state_tick = now;
                    state = STATE_CROSS_FORWARD;
                }
            }
            /* 弯道检测 (带冷却) → 先直行越过盲区再转弯 */
            else if (result == LINE_LEFT_TURN && (now - last_turn_tick >= TURN_COOLDOWN_MS)) {
                turn_sign = +1;
                forward_for_turn = 1;
                state_tick = now;
                state = STATE_CROSS_FORWARD;
            }
            else if (result == LINE_RIGHT_TURN && (now - last_turn_tick >= TURN_COOLDOWN_MS)) {
                turn_sign = -1;
                forward_for_turn = 1;
                state_tick = now;
                state = STATE_CROSS_FORWARD;
            }
            else if (now - state_tick >= LINE_DURATION_MS) {
                state = STATE_LINE_STOP;
            }
            break;
        }

        /* ==================== 前进状态 (弯道/十字越线) ==================== */
        case STATE_CROSS_FORWARD: {
            uint32_t fwd_ms = forward_for_turn ? TURN_FORWARD_MS : CROSS_FORWARD_MS;
            float base = LINE_BASE_SPEED; // 保持直行越线
            SetMotorSpeed(base, base);

            if (now - state_tick >= fwd_ms) {
                if (forward_for_turn) {
                    /* 弯道直行越线结束，转弯 */
                    AngleCtrl_SetTarget(&angle_ctrl, imu_data.yaw_total - (float)turn_sign * ANGLE_TARGET_DEG);
                    angle_start_tick = now;
                    chassis_dbg.target_angle = angle_ctrl.target;
                    forward_for_turn = 0;
                    state = STATE_ANGLE_TURNING;
                } else if (cross_action == CROSS_TURN_LEFT) {
                    /* 十字左转 */
                    turn_sign = +1;
                    AngleCtrl_SetTarget(&angle_ctrl, imu_data.yaw_total - (float)turn_sign * ANGLE_TARGET_DEG);
                    angle_start_tick = now;
                    chassis_dbg.target_angle = angle_ctrl.target;
                    state = STATE_ANGLE_TURNING;
                } else if (cross_action == CROSS_TURN_RIGHT) {
                    /* 十字右转 */
                    turn_sign = -1;
                    AngleCtrl_SetTarget(&angle_ctrl, imu_data.yaw_total - (float)turn_sign * ANGLE_TARGET_DEG);
                    angle_start_tick = now;
                    chassis_dbg.target_angle = angle_ctrl.target;
                    state = STATE_ANGLE_TURNING;
                } else if (cross_action == CROSS_STOP) {
                    /* 十字停车 */
                    state = STATE_LINE_STOP;
                } else {
                    /* [FIX-3] CROSS_SKIP：已驶过十字盲区，
                     *  - 重置巡线 PID 清掉直行段的积分
                     *  - 刷新 last_turn_tick 抑制十字残留导致的假弯道触发 */
                    LineFollow_Init();
                    last_turn_tick = now;
                    state = STATE_LINE_FOLLOWING;
                }
            }
            break;
        }

            /* ==================== 角度环转弯 ==================== */
        case STATE_ANGLE_TURNING: {
                float turn = AngleCtrl_Update(&angle_ctrl, imu_data.yaw_total);
                chassis_dbg.current_angle = imu_data.yaw_total;
                chassis_dbg.angle_error = angle_ctrl.error;
                chassis_dbg.pid_output = turn;

                float left, right;
                // [修改这里] 传入 &angle_ctrl 而不是 turn
                AngleCtrl_ToWheelSpeed(&angle_ctrl, &left, &right);
                SetMotorSpeed(left, right);

            LED_SetAll(
                (turn_sign > 0) ? 1 : 0,
                (turn_sign > 0) ? 1 : 0,
                (turn_sign < 0) ? 1 : 0,
                (turn_sign < 0) ? 1 : 0);

            if (AngleCtrl_IsDone(&angle_ctrl, ANGLE_ERR_THRESH) ||
                (now - angle_start_tick >= ANGLE_TIMEOUT_MS)) {
                last_turn_tick = now;
                LineFollow_Init(); // 清空巡线的积分与前馈
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

        chassis_dbg.state = state;

        /* 发布底盘反馈 */
        chassis_fdb_data.wheel_speed_l = chassis_dbg.left_speed;
        chassis_fdb_data.wheel_speed_r = chassis_dbg.right_speed;
        mcn_publish(MCN_HUB(chassis_fdb), &chassis_fdb_data);

        tick_ms += CHASSIS_TASK_PERIOD;
        osDelay(CHASSIS_TASK_PERIOD);
    }
}

/* ==================== 任务初始化 ==================== */
void chassis_task_init(void)
{
    Grayscale_Init();
    LineFollow_Init();
    AngleCtrl_Init(&angle_ctrl);

    const osThreadAttr_t chassis_task_attributes = {
        .name = "chassis_task",
        .stack_size = CHASSIS_TASK_STACK_SIZE,
        .priority = (osPriority_t) CHASSIS_TASK_PRIORITY,
    };

    chassisTaskHandle = osThreadNew(chassis_task_entry, NULL, &chassis_task_attributes);
}
