/**
 * @file robot.h
 * @brief Robot message definitions and system configuration
 * @date 2026-04-04
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <stdint.h>
#include <stdbool.h>
#include "rm_config.h"

/**
 * @brief 机器人初始化,请在开启rtos之前调用
 */
void robot_init(void);

/**
 * @brief 机器人任务,放入实时系统以一定频率运行
 */
void robot_task(void);

/* ==================== Message Structures ==================== */

/**
 * @brief INS(IMU)数据
 */
struct ins_msg {
    float gyro[3];           // 角速度, °/s
    float accel[3];          // 加速度, mg
    float motion_accel_b[3]; // 机体坐标加速度
    float roll;               // 横滚角, °
    float pitch;              // 俯仰角, °
    float yaw;                // 偏航角, °
    float yaw_total_angle;    // 偏航角总角度, °
};

/**
 * @brief 底盘模式
 */
typedef enum {
    CHASSIS_RELAX = 0,        // 底盘失能
    CHASSIS_INIT = 1,         // 底盘归中初始化
    CHASSIS_OPEN_LOOP = 2,    // 底盘开环
    CHASSIS_RECOVERY = 3,     // 底盘倒地自起
    CHASSIS_JUMP = 4,         // 底盘跳跃模式
    CHASSIS_FOLLOW_GIMBAL = 5,// 底盘跟随云台
    CHASSIS_SPIN = 6,         // 底盘陀螺模式
    CHASSIS_AUTO = 7,         // 底盘自动模式
} chassis_mode_e;

/**
 * @brief 腿长等级
 */
typedef enum {
    LEG_LOW = 0,
    LEG_MID = 1,
    LEG_HIG = 2,
} leg_level_e;

/**
 * @brief 腿长变化
 */
typedef enum {
    LENGTH_STAY = 0,
    LENGTH_CHANGE = 1,
} leg_change_e;

/**
 * @brief 底盘命令 (由cmd_task发布, chassis_task订阅)
 */
struct chassis_cmd_msg {
    float vx;                 // 前进方向速度, m/s
    float vx_set;            // 前进速度斜坡过程值
    float vy;                 // 横移方向速度, m/s
    float vy_set;            // 横移方向速度斜坡过程值
    float vw_relative;       // 旋转速度, °/s
    float vw_relative_set;   // 转向速度斜坡过程值
    float vw_fllow_gimbal_temp;  // 底盘跟随yaw角度中间值
    float vw_fllow_gimbal_set;  // 底盘跟随状态下的yaw命令
    float leg_length;        // 腿长
    chassis_mode_e ctrl_mode;// 当前底盘控制模式
    chassis_mode_e last_mode;// 上一次底盘控制模式
    leg_level_e leg_level;  // 腿长等级
    leg_change_e leg_leng_change; // 腿长变化
};

/**
 * @brief 底盘反馈 (由chassis_task发布, cmd_task订阅)
 */
struct chassis_fdb_msg {
    uint8_t leg_state;       // 腿部归中初始化情况
    uint8_t stand_state;     // 机器人站立状态
    float wheel_pos_l;       // 左轮位置
    float wheel_pos_r;       // 右轮位置
    float wheel_speed_l;     // 左轮速度
    float wheel_speed_r;     // 右轮速度
    bool touch_ground;       // 是否触地
};

/**
 * @brief 电机命令 (由cmd_task发布, motor_task订阅)
 */
struct motor_cmd_msg {
    float motor1_speed;     // 电机1速度目标
    float motor2_speed;     // 电机2速度目标
    float motor3_speed;     // 电机3速度目标
    float motor4_speed;     // 电机4速度目标
    float motor1_angle;      // 电机1角度目标
    float motor2_angle;      // 电机2角度目标
    float motor3_angle;      // 电机3角度目标
    float motor4_angle;      // 电机4角度目标
    uint8_t enable;         // 电机使能
};

/**
 * @brief 电机反馈 (由motor_task发布, cmd_task订阅)
 */
struct motor_fdb_msg {
    float motor1_angle;      // 电机1角度反馈
    float motor2_angle;      // 电机2角度反馈
    float motor3_angle;      // 电机3角度反馈
    float motor4_angle;      // 电机4角度反馈
    float motor1_speed;      // 电机1速度反馈
    float motor2_speed;      // 电机2速度反馈
    float motor3_speed;      // 电机3速度反馈
    float motor4_speed;      // 电机4速度反馈
    float motor1_current;    // 电机1电流
    float motor2_current;    // 电机2电流
    float motor3_current;    // 电机3电流
    float motor4_current;    // 电机4电流
};

/**
 * @brief 传感器命令 (由cmd_task发布, sense_task订阅)
 */
struct sense_cmd_msg {
    uint8_t imu_enable;      // IMU使能
    uint8_t gyro_calibrate;  // 陀螺仪校准
    uint8_t accel_calibrate;// 加速度计校准
};

/**
 * @brief 传感器反馈 (由sense_task发布, cmd_task订阅)
 */
struct sense_fdb_msg {
    float temperature;        // 温度
    float battery_voltage;    // 电池电压
    uint8_t sensor_status;   // 传感器状态
};

#endif // ROBOT_H
