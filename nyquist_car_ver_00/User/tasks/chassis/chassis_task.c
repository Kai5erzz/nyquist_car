/**
 * @file chassis_task.c
 * @brief 底盘控制任务
 * @date 2026-04-04
 */

#include "chassis_task.h"
#include "rm_task.h"
#include "uMCN.h"
#include "robot.h"

/* ==================== Task Handle ==================== */
osThreadId_t chassisTaskHandle;

/* ==================== Topic Subscriptions ==================== */
MCN_DECLARE(ins_topic);
MCN_DECLARE(chassis_cmd);

static McnNode_t ins_topic_node;
static McnNode_t chassis_cmd_node;

/* ==================== Topic Publications ==================== */
MCN_DECLARE(chassis_fdb);
static struct chassis_fdb_msg chassis_fdb_data;

/* ==================== Local Data ==================== */
static struct ins_msg ins_data;
static struct chassis_cmd_msg chassis_cmd_data;

/* ==================== Subscription Init ==================== */
static void chassis_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    chassis_cmd_node = mcn_subscribe(MCN_HUB(chassis_cmd), NULL, NULL);
}

/* ==================== Subscription Pull ==================== */
static void chassis_sub_pull(void)
{
    if (mcn_poll(ins_topic_node)) {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins_data);
    }
    if (mcn_poll(chassis_cmd_node)) {
        mcn_copy(MCN_HUB(chassis_cmd), chassis_cmd_node, &chassis_cmd_data);
    }
}

/* ==================== Publication Push ==================== */
static void chassis_pub_push(void)
{
    mcn_publish(MCN_HUB(chassis_fdb), &chassis_fdb_data);
}

/* ==================== Control Loop ==================== */
static void chassis_control_loop(void)
{
    // TODO: 底盘控制逻辑实现
}

/**
 * @brief 底盘任务入口
 */
__attribute__((noreturn))
void chassis_task_entry(void *argument)
{
    uint32_t wake_time = osKernelSysTick();

    chassis_sub_init();

    for (;;) {
        chassis_sub_pull();
        chassis_control_loop();
        chassis_pub_push();

        vTaskDelayUntil(&wake_time, CHASSIS_TASK_PERIOD);
    }
}

void chassis_task_init(void)
{
    const osThreadAttr_t chassis_task_attributes = {
        .name = "chassis_task",
        .stack_size = CHASSIS_TASK_STACK_SIZE,
        .priority = (osPriority_t) CHASSIS_TASK_PRIORITY,
    };

    chassisTaskHandle = osThreadNew(chassis_task_entry, NULL, &chassis_task_attributes);
}
