/**
 * @file ins_task.c
 * @brief IMU/INS惯性导航任务
 * @date 2026-04-04
 */

#include "ins_task.h"
#include "cmsis_os.h"
#include "rm_task.h"
#include "uMCN.h"
#include "robot.h"
#include "drv/bmi088/bmi088.h"
#include "drv/oled/OLED.h"
#include "fdcan.h"

/* ==================== Task Handle ==================== */
osThreadId_t insTaskHandle;

/* ==================== Topic Publications ==================== */
MCN_DECLARE(ins_topic);
static struct ins_msg ins_data;

/* ==================== OLED Display ==================== */
#define OLED_DISPLAY_INTERVAL  25  // OLED刷新间隔 (ms)，避免刷新过快

#define IMU_CAN_ID  0x11            // 达妙DM-IMU-L1模块CAN ID

static void ins_oled_display(void);

/* ==================== Publication Push ==================== */
static void ins_pub_push(void)
{
    mcn_publish(MCN_HUB(ins_topic), &ins_data);
}

/* ==================== IMU Data Update ==================== */
static void ins_update(void)
{
    // 从BMI088 IMU读取数据 (通过FDCAN接收，已在IMU_UpdateData中解析)
    ins_data.gyro[0]  = imu_data.gyro[0];
    ins_data.gyro[1]  = imu_data.gyro[1];
    ins_data.gyro[2]  = imu_data.gyro[2];
    ins_data.accel[0] = imu_data.accel[0];
    ins_data.accel[1] = imu_data.accel[1];
    ins_data.accel[2] = imu_data.accel[2];
    ins_data.roll     = imu_data.roll;
    ins_data.pitch    = imu_data.pitch;
    ins_data.yaw      = imu_data.yaw;
}

/**
 * @brief INS任务入口
 */
__attribute__((noreturn))
void ins_task_entry(void *argument)
{
    uint32_t wake_time = osKernelSysTick();
    uint32_t oled_tick = 0;

    for (;;) {
        // 向IMU请求数据 (应答模式: 发请求帧后IMU自动回传)
        IMU_RequestData(&hfdcan1, IMU_CAN_ID, 0x01);  // 请求加速度
        IMU_RequestData(&hfdcan1, IMU_CAN_ID, 0x02);  // 请求角速度
        IMU_RequestData(&hfdcan1, IMU_CAN_ID, 0x03);  // 请求欧拉角
        IMU_RequestData(&hfdcan1, IMU_CAN_ID, 0x04);  // 请求四元数

        ins_update();
        ins_pub_push();

        // OLED刷新 (限速，避免I2C总线拥堵)
        if (osKernelSysTick() - oled_tick >= OLED_DISPLAY_INTERVAL) {
            oled_tick = osKernelSysTick();
            ins_oled_display();
        }

        vTaskDelayUntil(&wake_time, INS_TASK_PERIOD);
    }
}

void ins_task_init(void)
{
    OLED_ShowString(1, 1, "IMU Initializing...");
    OLED_UpdateGRAM();

    const osThreadAttr_t ins_task_attributes = {
        .name = "ins_task",
        .stack_size = INS_TASK_STACK_SIZE,
        .priority = (osPriority_t) INS_TASK_PRIORITY,
    };

    insTaskHandle = osThreadNew(ins_task_entry, NULL, &ins_task_attributes);
}

/* ==================== OLED Display ==================== */
static void ins_oled_display(void)
{
    // 第1行: 姿态角 (Roll, Pitch, Yaw)
    OLED_ShowString(1, 1, "R:");
    OLED_ShowSignedFloat(1, 3, ins_data.roll,  3, 2);
    OLED_ShowString(1, 9, "P:");
    OLED_ShowSignedFloat(1, 11, ins_data.pitch, 3, 2);

    // 第2行: Yaw角
    OLED_ShowString(2, 1, "Y:");
    OLED_ShowSignedFloat(2, 3, ins_data.yaw, 3, 2);

    // 第3行: 角速度 (Gx, Gy, Gz) 单位: rad/s
    OLED_ShowString(3, 1, "G:");
    OLED_ShowSignedFloat(3, 3, ins_data.gyro[0], 2, 3);
    OLED_ShowString(3, 9, "Gy:");
    OLED_ShowSignedFloat(3, 12, ins_data.gyro[1], 2, 3);

    // 第4行: 加速度 (Ax, Ay, Az) 单位: m/s^2
    OLED_ShowString(4, 1, "A:");
    OLED_ShowSignedFloat(4, 3, ins_data.accel[0], 2, 3);
    OLED_ShowString(4, 9, "Az:");
    OLED_ShowSignedFloat(4, 12, ins_data.accel[2], 2, 3);

    OLED_UpdateGRAM();
}
