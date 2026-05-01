/**
 * @file    bmi088.h
 * @author  kaiser
 * @version V2.0.0
 * @date    2026-05-01
 * @brief   达妙DM-IMU-L1 BMI088 CAN驱动
 *
 * 功能:
 *   - 主动上报模式: 配置IMU自动发送数据, 无需轮询
 *   - 软件陀螺仪零偏校准 (采样2000次取平均)
 *   - 加速度计标度因数校正 (g_norm归一化)
 *   - 数据时间戳与帧计数 (丢帧检测)
 *   - 数据就绪标志位 (分类型标志)
 *
 * CAN协议:
 *   请求帧: {0xCC, reg, 0x00, 0xDD}
 *   写命令: {0xCC, reg, 0x01, data..., 0xDD}
 *   数据帧类型: 0x01=加速度, 0x02=角速度, 0x03=欧拉角, 0x04=四元数
 */

#ifndef BMI088_H
#define BMI088_H

#include "stm32h7xx_hal.h"

/* ==================== 量程映射宏 ==================== */
#define ACCEL_CAN_MAX       (235.2f)    /**< 加速度量程上限 (m/s^2) */
#define ACCEL_CAN_MIN       (-235.2f)   /**< 加速度量程下限 (m/s^2) */
#define GYRO_CAN_MAX        (34.88f)    /**< 角速度量程上限 (rad/s) */
#define GYRO_CAN_MIN        (-34.88f)   /**< 角速度量程下限 (rad/s) */
#define PITCH_CAN_MAX       (90.0f)     /**< 俯仰角量程上限 (deg) */
#define PITCH_CAN_MIN       (-90.0f)    /**< 俯仰角量程下限 (deg) */
#define ROLL_CAN_MAX        (180.0f)    /**< 横滚角量程上限 (deg) */
#define ROLL_CAN_MIN        (-180.0f)   /**< 横滚角量程下限 (deg) */
#define YAW_CAN_MAX         (180.0f)    /**< 航向角量程上限 (deg) */
#define YAW_CAN_MIN         (-180.0f)   /**< 航向角量程下限 (deg) */
#define Quaternion_CAN_MIN  (-1.0f)     /**< 四元数量程下限 */
#define Quaternion_CAN_MAX  (1.0f)      /**< 四元数量程上限 */

/* ==================== 默认校准值 ==================== */
#define DEFAULT_GX_OFFSET   0.0f       /**< 陀螺仪X轴默认零偏 */
#define DEFAULT_GY_OFFSET   0.0f       /**< 陀螺仪Y轴默认零偏 */
#define DEFAULT_GZ_OFFSET   0.0f       /**< 陀螺仪Z轴默认零偏 */
#define DEFAULT_G_NORM      9.81f      /**< 默认重力加速度 (m/s^2) */
#define IMU_CALI_SAMPLES    2000       /**< 校准采样次数 */

/**
 * @brief CAN数据帧类型枚举
 */
typedef enum {
    IMU_DATA_ACCEL      = 0x01, /**< 加速度数据帧 */
    IMU_DATA_GYRO       = 0x02, /**< 角速度数据帧 */
    IMU_DATA_EULER      = 0x03, /**< 欧拉角数据帧 */
    IMU_DATA_QUATERNION = 0x04, /**< 四元数数据帧 */
} IMU_DataType_e;

/* 数据就绪标志位 */
#define IMU_FLAG_ACCEL_READY      (1 << 0) /**< 加速度数据就绪 */
#define IMU_FLAG_GYRO_READY       (1 << 1) /**< 角速度数据就绪 */
#define IMU_FLAG_EULER_READY      (1 << 2) /**< 欧拉角数据就绪 */
#define IMU_FLAG_QUATERNION_READY (1 << 3) /**< 四元数数据就绪 */
#define IMU_FLAG_ALL_READY        (0x0F)   /**< 全部数据就绪 */

/**
 * @brief IMU数据结构体
 */
typedef struct {
    /* 原始数据 */
    float accel[3];       /**< 加速度 [X,Y,Z] (m/s^2), 已校准 */
    float gyro[3];        /**< 角速度 [X,Y,Z] (rad/s), 已去偏 */
    float pitch;           /**< 俯仰角 (deg) */
    float yaw;             /**< 航向角 (deg) */
    float roll;            /**< 横滚角 (deg) */
    float q[4];            /**< 四元数 [W,X,Y,Z] */
    float temperature;     /**< 温度 (°C) */

    /* 校准参数 */
    float gyro_offset[3]; /**< 陀螺仪零偏 [X,Y,Z] */
    float accel_scale;     /**< 加速度计标度因数 */
    float g_norm;          /**< 重力加速度标定值 (m/s^2) */

    /* 状态 */
    uint32_t timestamp;    /**< 最后更新时间 (HAL_GetTick, ms) */
    uint8_t  data_flags;   /**< 数据就绪标志位 (IMU_FLAG_x) */
    uint8_t  is_calibrated;/**< 是否已完成校准 */
    uint8_t  frame_count;  /**< 帧计数器 (用于检测丢帧) */
} DM_IMU_Data_t;

extern DM_IMU_Data_t imu_data;

/* ==================== 工具函数 ==================== */

/**
 * @brief  无符号整数线性映射为浮点数
 * @param  x_int  输入整数
 * @param  x_min  输出范围下限
 * @param  x_max  输出范围上限
 * @param  bits   整数位宽 (如16表示0~65535)
 * @return 映射后的浮点数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits);

/* ==================== 初始化与配置 ==================== */

/**
 * @brief  初始化IMU (配置主动上报模式)
 * @param  hfdcan  FDCAN句柄
 * @param  can_id  IMU模块CAN ID
 * @note   清零数据结构, 设置默认零偏, 发送主动上报使能命令
 */
void DM_IMU_Init(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id);

/**
 * @brief  配置IMU主动上报模式
 * @param  hfdcan  FDCAN句柄
 * @param  can_id  IMU模块CAN ID
 * @param  enable  1=开启, 0=关闭(恢复应答模式)
 */
void DM_IMU_SetActiveReport(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t enable);

/* ==================== 数据处理 ==================== */

/**
 * @brief  IMU数据解析入口
 * @param  pData  FDCAN接收到的8字节数据
 * @note   根据pData[0]区分帧类型并解析到imu_data全局结构体
 *         在FDCAN_RxFifo0Callback中调用
 */
void DM_IMU_UpdateData(uint8_t *pData);

/**
 * @brief  检查数据是否全部就绪
 * @return 1=全部就绪, 0=部分缺失
 */
uint8_t DM_IMU_IsDataReady(void);

/**
 * @brief  陀螺仪软件零偏校准
 * @note   需要IMU静止, 采样IMU_CALI_SAMPLES次取平均
 *         校准期间阻塞调用线程, 超时则使用默认值
 *         同时校准加速度计标度因数 (accel_scale)
 */
void DM_IMU_CalibrateGyro(void);

/* 兼容旧接口 */
void IMU_RequestData(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t reg);
void IMU_UpdateData(uint8_t *pData);

#endif /* BMI088_H */
