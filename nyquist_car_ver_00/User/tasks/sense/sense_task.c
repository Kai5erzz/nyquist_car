/**
 * @file sense_task.c
 * @brief 传感器感知任务
 * @date 2026-04-04
 */

#include "sense_task.h"
#include "rm_task.h"
#include "uMCN.h"
#include "robot.h"

/* ==================== Task Handle ==================== */
osThreadId_t senseTaskHandle;

/* ==================== Topic Subscriptions ==================== */
MCN_DECLARE(sense_cmd);
static McnNode_t sense_cmd_node;

/* ==================== Topic Publications ==================== */
MCN_DECLARE(sense_fdb);
static struct sense_fdb_msg sense_fdb_data;

/* ==================== Local Data ==================== */
static struct sense_cmd_msg sense_cmd_data;

/* ==================== Subscription Init ==================== */
static void sense_sub_init(void)
{
    sense_cmd_node = mcn_subscribe(MCN_HUB(sense_cmd), NULL, NULL);
}

/* ==================== Subscription Pull ==================== */
static void sense_sub_pull(void)
{
    if (mcn_poll(sense_cmd_node)) {
        mcn_copy(MCN_HUB(sense_cmd), sense_cmd_node, &sense_cmd_data);
    }
}

/* ==================== Publication Push ==================== */
static void sense_pub_push(void)
{
    mcn_publish(MCN_HUB(sense_fdb), &sense_fdb_data);
}

/* ==================== Sensor Update ==================== */
static void sense_update(void)
{
    // TODO: 传感器数据采集
    // 温度: sense_fdb_data.temperature
    // 电池电压: sense_fdb_data.battery_voltage
    // 传感器状态: sense_fdb_data.sensor_status
}

/**
 * @brief 传感器任务入口
 */
__attribute__((noreturn))
void sense_task_entry(void *argument)
{
    uint32_t wake_time = osKernelSysTick();

    sense_sub_init();

    for (;;) {
        sense_sub_pull();
        sense_update();
        sense_pub_push();

        vTaskDelayUntil(&wake_time, SENSE_TASK_PERIOD);
    }
}

void sense_task_init(void)
{
    const osThreadAttr_t sense_task_attributes = {
        .name = "sense_task",
        .stack_size = SENSE_TASK_STACK_SIZE,
        .priority = (osPriority_t) SENSE_TASK_PRIORITY,
    };

    senseTaskHandle = osThreadNew(sense_task_entry, NULL, &sense_task_attributes);
}
