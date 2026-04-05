/**
 * @file cmd_task.c
 * @brief 命令/遥控器任务
 * @date 2026-04-04
 */

#include "cmd_task.h"
#include "rm_task.h"
#include "uMCN.h"
#include "robot.h"

/* ==================== Task Handle ==================== */
osThreadId_t cmdTaskHandle;

/* ==================== Topic Subscriptions ==================== */
MCN_DECLARE(chassis_fdb);
MCN_DECLARE(motor_fdb);
MCN_DECLARE(ins_topic);
MCN_DECLARE(sense_fdb);

static McnNode_t chassis_fdb_node;
static McnNode_t motor_fdb_node;
static McnNode_t ins_topic_node;
static McnNode_t sense_fdb_node;

/* ==================== Topic Publications ==================== */
MCN_DECLARE(chassis_cmd);
MCN_DECLARE(motor_cmd);
MCN_DECLARE(sense_cmd);

static struct chassis_cmd_msg chassis_cmd_data;
static struct motor_cmd_msg motor_cmd_data;
static struct sense_cmd_msg sense_cmd_data;

/* ==================== Local Data ==================== */
static struct chassis_fdb_msg chassis_fdb_data;
static struct motor_fdb_msg motor_fdb_data;
static struct ins_msg ins_data;
static struct sense_fdb_msg sense_fdb_data;

/* ==================== Subscription Init ==================== */
static void cmd_sub_init(void)
{
    chassis_fdb_node = mcn_subscribe(MCN_HUB(chassis_fdb), NULL, NULL);
    motor_fdb_node = mcn_subscribe(MCN_HUB(motor_fdb), NULL, NULL);
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    sense_fdb_node = mcn_subscribe(MCN_HUB(sense_fdb), NULL, NULL);
}

/* ==================== Subscription Pull ==================== */
static void cmd_sub_pull(void)
{
    if (mcn_poll(chassis_fdb_node)) {
        mcn_copy(MCN_HUB(chassis_fdb), chassis_fdb_node, &chassis_fdb_data);
    }
    if (mcn_poll(motor_fdb_node)) {
        mcn_copy(MCN_HUB(motor_fdb), motor_fdb_node, &motor_fdb_data);
    }
    if (mcn_poll(ins_topic_node)) {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins_data);
    }
    if (mcn_poll(sense_fdb_node)) {
        mcn_copy(MCN_HUB(sense_fdb), sense_fdb_node, &sense_fdb_data);
    }
}

/* ==================== Publication Push ==================== */
static void cmd_pub_push(void)
{
    mcn_publish(MCN_HUB(chassis_cmd), &chassis_cmd_data);
    mcn_publish(MCN_HUB(motor_cmd), &motor_cmd_data);
    mcn_publish(MCN_HUB(sense_cmd), &sense_cmd_data);
}

/* ==================== Control Loop ==================== */
static void cmd_control_loop(void)
{
    // TODO: 遥控器解析和命令生成
    // 1. 读取遥控器数据
    // 2. 解析控制指令
    // 3. 填充 chassis_cmd_data, motor_cmd_data, sense_cmd_data
}

/**
 * @brief CMD任务入口
 */
__attribute__((noreturn))
void cmd_task_entry(void *argument)
{
    uint32_t wake_time = osKernelSysTick();

    cmd_sub_init();

    for (;;) {
        cmd_sub_pull();
        cmd_control_loop();
        cmd_pub_push();

        vTaskDelayUntil(&wake_time, CMD_TASK_PERIOD);
    }
}

/* 建议删除旧的 osThreadDef 宏 */
void cmd_task_init(void)
{
    /* 使用 V2 标准的属性结构体 */
    const osThreadAttr_t cmd_task_attributes = {
        .name = "cmd_task",
        .stack_size = CMD_TASK_STACK_SIZE,
        .priority = (osPriority_t) CMD_TASK_PRIORITY,
    };

    /* 使用 osThreadNew 直接创建 */
    cmdTaskHandle = osThreadNew(cmd_task_entry, NULL, &cmd_task_attributes);
}
