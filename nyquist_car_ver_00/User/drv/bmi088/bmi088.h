#ifndef __DM_IMU_L1_H
#define __DM_IMU_L1_H

#include "stm32h7xx_hal.h"

/* ----------------- 线性映射量程宏定义 [cite: 377, 378, 381, 382, 385, 386, 388, 389, 392, 393, 395, 397] ----------------- */
#define ACCEL_CAN_MAX       (235.2f)
#define ACCEL_CAN_MIN       (-235.2f)
#define GYRO_CAN_MAX        (34.88f)
#define GYRO_CAN_MIN        (-34.88f)
#define PITCH_CAN_MAX       (90.0f)
#define PITCH_CAN_MIN       (-90.0f)
#define ROLL_CAN_MAX        (180.0f)
#define ROLL_CAN_MIN        (-180.0f)
#define YAW_CAN_MAX         (180.0f)
#define YAW_CAN_MIN         (-180.0f)
#define Quaternion_CAN_MIN  (-1.0f)
#define Quaternion_CAN_MAX  (1.0f)

/* ----------------- 数据结构定义 ----------------- */
typedef struct {
    float accel[3];    // [0]=X, [1]=Y, [2]=Z (m/s^2)
    float gyro[3];     // [0]=X, [1]=Y, [2]=Z (rad/s)
    float pitch;       // 俯仰角 (deg)
    float yaw;         // 航向角 (deg)
    float roll;        // 横滚角 (deg)
    float q[4];        // 四元数 [0]=W, [1]=X, [2]=Y, [3]=Z
    uint8_t temp;      // IMU温度
} DM_IMU_Data_t;

extern DM_IMU_Data_t imu_data;

/* ----------------- 函数声明 ----------------- */
// 浮点与无符号整型转换工具函数
float uint_to_float(int x_int, float x_min, float x_max, int bits);

// CAN数据请求函数 (针对应答模式)
void IMU_RequestData(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t reg);

// CAN接收数据解析函数 (在 FDCAN Rx FIFO 回调中调用)
void IMU_UpdateData(uint8_t* pData);

#endif /* __DM_IMU_L1_H */