/**
 * @file motor_task.c
 * @brief 电机控制任务
 * @date 2026-04-04
 */

#include "motor_task.h"
#include "cmsis_os2.h"
#include "rm_task.h"
#include "uMCN.h"
#include "robot.h"

/* ==================== Task Handle ==================== */
osThreadId_t motorTaskHandle;

/* ==================== Topic Subscriptions ==================== */
MCN_DECLARE(motor_cmd);
static McnNode_t motor_cmd_node;

/* ==================== Topic Publications ==================== */
MCN_DECLARE(motor_fdb);
static struct motor_fdb_msg motor_fdb_data;

/* ==================== Local Data ==================== */
static struct motor_cmd_msg motor_cmd_data;

/* ==================== Subscription Init ==================== */
static void motor_sub_init(void)
{
    motor_cmd_node = mcn_subscribe(MCN_HUB(motor_cmd), NULL, NULL);
}

/* ==================== Subscription Pull ==================== */
static void motor_sub_pull(void)
{
    if (mcn_poll(motor_cmd_node)) {
        mcn_copy(MCN_HUB(motor_cmd), motor_cmd_node, &motor_cmd_data);
    }
}

/* ==================== Publication Push ==================== */
static void motor_pub_push(void)
{
    mcn_publish(MCN_HUB(motor_fdb), &motor_fdb_data);
}

/* ==================== Motor Control ==================== */
static void motor_control_loop(void)
{
    // TODO: 电机控制逻辑实现
    // 1. 读取电机编码器反馈
    // 2. PID速度/位置控制
    // 3. 发送CAN命令
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    osDelay(250);
}

/**
 * @brief 电机任务入口
 */
__attribute__((noreturn))
void motor_task_entry(void *argument)
{
    uint32_t wake_time = osKernelSysTick();

    motor_sub_init();

    for (;;) {
        motor_sub_pull();
        motor_control_loop();
        motor_pub_push();

        vTaskDelayUntil(&wake_time, MOTOR_TASK_PERIOD);
    }
}

void motor_task_init(void)
{
    const osThreadAttr_t motor_task_attributes = {
        .name = "motor_task",
        .stack_size = MOTOR_TASK_STACK_SIZE,
        .priority = (osPriority_t) MOTOR_TASK_PRIORITY,
    };

    motorTaskHandle = osThreadNew(motor_task_entry, NULL, &motor_task_attributes);
}
