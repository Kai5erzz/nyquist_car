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
 * @brief INS任务入口
 * @note  仅采集IMU数据并发布到uMCN, VOFA+输出由sense_task统一处理
 */
__attribute__((noreturn))
void ins_task_entry(void *argument)
{
    uint32_t wake_time = osKernelSysTick();

    for (;;) {
        ins_update();
        ins_pub_push();

        vTaskDelayUntil(&wake_time, INS_TASK_PERIOD);
    }
}

void ins_task_init(void)
{
    /* 初始化IMU (主动上报模式) */
    DM_IMU_Init(&hfdcan1, IMU_CAN_ID);

    const osThreadAttr_t ins_task_attributes = {
        .name = "ins_task",
        .stack_size = INS_TASK_STACK_SIZE,
        .priority = (osPriority_t) INS_TASK_PRIORITY,
    };

    insTaskHandle = osThreadNew(ins_task_entry, NULL, &ins_task_attributes);
}
