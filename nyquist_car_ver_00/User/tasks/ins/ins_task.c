/**
 * @file    ins_task.c
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-05-01
 * @brief   IMU/INS惯性导航任务
 *
 * 工作模式: 应答式
 *   每周期发送4个请求帧 → IMU回传数据 → CAN中断解析
 */

#include "ins_task.h"
#include "cmsis_os.h"
#include "rm_task.h"
#include "uMCN.h"
#include "robot.h"
#include "drv/bmi088/bmi088.h"
#include "fdcan.h"

/* ==================== Task Handle ==================== */
osThreadId_t insTaskHandle;

/* ==================== Topic Publications ==================== */
MCN_DECLARE(ins_topic);
static struct ins_msg ins_data;

#define IMU_CAN_ID  0x11

/* ==================== Publication Push ==================== */
static void ins_pub_push(void)
{
    mcn_publish(MCN_HUB(ins_topic), &ins_data);
}

/* ==================== IMU Data Update ==================== */
static void ins_update(void)
{
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
 * @brief INS任务入口 (1ms, 1000Hz)
 */
__attribute__((noreturn))
void ins_task_entry(void *argument)
{
    uint32_t wake_time = osKernelSysTick();

    for (;;) {
        /* 只请求欧拉角 (0x03), 1000Hz */
        IMU_RequestData(&hfdcan1, IMU_CAN_ID, 0x03);

        ins_update();
        ins_pub_push();

        vTaskDelayUntil(&wake_time, INS_TASK_PERIOD);
    }
}

void ins_task_init(void)
{
    /* 初始化IMU数据结构 */
    for (uint8_t i = 0; i < 3; i++) {
        imu_data.accel[i] = 0;
        imu_data.gyro[i] = 0;
        imu_data.gyro_offset[i] = 0;
    }
    for (uint8_t i = 0; i < 4; i++) {
        imu_data.q[i] = 0;
    }
    imu_data.pitch = 0;
    imu_data.yaw = 0;
    imu_data.roll = 0;
    imu_data.temperature = 0;
    imu_data.accel_scale = 1.0f;
    imu_data.g_norm = 9.81f;
    imu_data.timestamp = 0;
    imu_data.data_flags = 0;
    imu_data.is_calibrated = 0;
    imu_data.frame_count = 0;

    const osThreadAttr_t ins_task_attributes = {
        .name = "ins_task",
        .stack_size = INS_TASK_STACK_SIZE,
        .priority = (osPriority_t) INS_TASK_PRIORITY,
    };

    insTaskHandle = osThreadNew(ins_task_entry, NULL, &ins_task_attributes);
}
