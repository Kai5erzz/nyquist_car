/**
 * @file robot.c
 * @brief Robot initialization and topic definitions
 * @date 2026-04-04
 */

#include "robot.h"
#include "rm_task.h"
#include "uMCN.h"
#include "drv/oled/OLED.h"

/* ==================== Topic Definitions ==================== */
/* 同一个主题名称不可被重复定义 */
MCN_DEFINE(chassis_cmd, sizeof(struct chassis_cmd_msg));
MCN_DEFINE(chassis_fdb, sizeof(struct chassis_fdb_msg));
MCN_DEFINE(ins_topic, sizeof(struct ins_msg));
MCN_DEFINE(motor_cmd, sizeof(struct motor_cmd_msg));
MCN_DEFINE(motor_fdb, sizeof(struct motor_fdb_msg));
MCN_DEFINE(sense_cmd, sizeof(struct sense_cmd_msg));
MCN_DEFINE(sense_fdb, sizeof(struct sense_fdb_msg));

static void mcn_topic_init(void);

void OS_task_init() {
    
}

void robot_init(void)
{
    // 关闭中断,防止在初始化过程中发生中断
    __disable_irq();

    OS_task_init();          // 创建基础任务

    mcn_topic_init();        // 话题注册初始化

    sense_task_init();
    ins_task_init();
    // motor_task_init();
    // chassis_task_init();
    // cmd_task_init();



    // 初始化完成,开启中断
    __enable_irq();
}

/**
 * @brief ipc uMCN 各话题注册
 */
static void mcn_topic_init(void)
{
    mcn_advertise(MCN_HUB(ins_topic), NULL);
    mcn_advertise(MCN_HUB(chassis_cmd), NULL);
    mcn_advertise(MCN_HUB(chassis_fdb), NULL);
    mcn_advertise(MCN_HUB(motor_cmd), NULL);
    mcn_advertise(MCN_HUB(motor_fdb), NULL);
    mcn_advertise(MCN_HUB(sense_cmd), NULL);
    mcn_advertise(MCN_HUB(sense_fdb), NULL);
}
