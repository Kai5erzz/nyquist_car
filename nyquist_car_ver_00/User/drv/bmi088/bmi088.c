/**
 * @file    bmi088.c
 * @author  kaiser
 * @version V2.0.0
 * @date    2026-05-01
 * @brief   达妙DM-IMU-L1 BMI088 CAN驱动实现
 *
 * 数据流:
 *   主动上报模式: IMU自动发送 → FDCAN中断 → DM_IMU_UpdateData()解析
 *   应答模式:     IMU_RequestData()请求 → IMU回传 → 解析
 *
 * 校准流程:
 *   DM_IMU_CalibrateGyro() → 采样2000次 → 计算gyro_offset和accel_scale
 */

#include "bmi088.h"
#include <math.h>

/** IMU全局数据实例 */
DM_IMU_Data_t imu_data;

/**
 * @brief  无符号整数线性映射为浮点数 (内部实现)
 * @param  x_int  输入整数
 * @param  x_min  输出范围下限
 * @param  x_max  输出范围上限
 * @param  bits   整数位宽
 * @return 映射后的浮点数
 */
static float uint_to_float_impl(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief  无符号整数线性映射为浮点数 (公共接口)
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    return uint_to_float_impl(x_int, x_min, x_max, bits);
}

/**
 * @brief  发送CAN写命令到IMU模块
 * @param  hfdcan  FDCAN句柄
 * @param  can_id  IMU CAN ID
 * @param  reg     目标寄存器地址
 * @param  data    写入数据指针
 * @param  len     数据长度 (最大4字节)
 */
static void DM_IMU_SendCmd(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id,
                            uint8_t reg, uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};

    tx_data[0] = 0xCC;
    tx_data[1] = reg;
    tx_data[2] = 0x01;
    for (uint8_t i = 0; i < len && i < 4; i++) {
        tx_data[3 + i] = data[i];
    }
    tx_data[7] = 0xDD;

    tx_header.Identifier = can_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0) {
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);
    }
}

/**
 * @brief  配置IMU主动上报模式
 * @param  hfdcan  FDCAN句柄
 * @param  can_id  IMU CAN ID
 * @param  enable  1=开启主动上报, 0=关闭
 */
void DM_IMU_SetActiveReport(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t enable)
{
    uint8_t data = enable ? 0x01 : 0x00;
    DM_IMU_SendCmd(hfdcan, can_id, 0x05, &data, 1);
}

/**
 * @brief  初始化IMU
 * @param  hfdcan  FDCAN句柄
 * @param  can_id  IMU CAN ID
 * @note   清零数据, 设置默认零偏, 使能主动上报
 */
void DM_IMU_Init(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id)
{
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
    imu_data.yaw_total = 0;
    imu_data.roll_total = 0;
    imu_data.pitch_total = 0;
    imu_data.temperature = 0;
    imu_data.accel_scale = 1.0f;
    imu_data.g_norm = DEFAULT_G_NORM;
    imu_data.timestamp = 0;
    imu_data.data_flags = 0;
    imu_data.is_calibrated = 0;
    imu_data.frame_count = 0;

    imu_data.gyro_offset[0] = DEFAULT_GX_OFFSET;
    imu_data.gyro_offset[1] = DEFAULT_GY_OFFSET;
    imu_data.gyro_offset[2] = DEFAULT_GZ_OFFSET;

    DM_IMU_SetActiveReport(hfdcan, can_id, 1);
}

/**
 * @brief  向IMU发送请求帧 (应答模式)
 * @param  hfdcan  FDCAN句柄
 * @param  can_id  IMU CAN ID
 * @param  reg     请求寄存器地址 (0x01=加速度, 0x02=角速度, 0x03=欧拉角, 0x04=四元数)
 */
void IMU_RequestData(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t reg)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};

    tx_data[0] = 0xCC;
    tx_data[1] = reg;
    tx_data[2] = 0x00;
    tx_data[3] = 0xDD;

    tx_header.Identifier = can_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0) {
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);
    }
}

/* ==================== 数据解析 ==================== */

/**
 * @brief  解析加速度数据帧 (pData[0]=0x01)
 * @param  pData  8字节CAN数据
 * @note   同时提取温度值 (pData[1])
 */
static void DM_IMU_ParseAccel(uint8_t *pData)
{
    uint16_t raw[3];
    raw[0] = (pData[3] << 8) | pData[2];
    raw[1] = (pData[5] << 8) | pData[4];
    raw[2] = (pData[7] << 8) | pData[6];

    imu_data.accel[0] = uint_to_float_impl(raw[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16) * imu_data.accel_scale;
    imu_data.accel[1] = uint_to_float_impl(raw[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16) * imu_data.accel_scale;
    imu_data.accel[2] = uint_to_float_impl(raw[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16) * imu_data.accel_scale;
    imu_data.temperature = (float)pData[1];
    imu_data.data_flags |= IMU_FLAG_ACCEL_READY;
}

/**
 * @brief  解析角速度数据帧 (pData[0]=0x02)
 * @param  pData  8字节CAN数据
 * @note   自动减去gyro_offset进行零偏补偿
 */
static void DM_IMU_ParseGyro(uint8_t *pData)
{
    uint16_t raw[3];
    raw[0] = (pData[3] << 8) | pData[2];
    raw[1] = (pData[5] << 8) | pData[4];
    raw[2] = (pData[7] << 8) | pData[6];

    imu_data.gyro[0] = uint_to_float_impl(raw[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16) - imu_data.gyro_offset[0];
    imu_data.gyro[1] = uint_to_float_impl(raw[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16) - imu_data.gyro_offset[1];
    imu_data.gyro[2] = uint_to_float_impl(raw[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16) - imu_data.gyro_offset[2];
    imu_data.data_flags |= IMU_FLAG_GYRO_READY;
}

/**
 * @brief  解析欧拉角数据帧 (pData[0]=0x03)
 * @param  pData  8字节CAN数据
 * @note   pitch=俯仰[-90,90], yaw=航向[-180,180], roll=横滚[-180,180]
 *         同时更新累加角度 (yaw_total/roll_total/pitch_total), 用于角度闭环
 */
static void DM_IMU_ParseEuler(uint8_t *pData)
{
    static float prev_yaw = 0, prev_roll = 0, prev_pitch = 0;
    static uint8_t first_frame = 1;

    uint16_t raw[3];
    raw[0] = (pData[3] << 8) | pData[2];
    raw[1] = (pData[5] << 8) | pData[4];
    raw[2] = (pData[7] << 8) | pData[6];

    float new_pitch = uint_to_float_impl(raw[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
    float new_yaw   = uint_to_float_impl(raw[1], YAW_CAN_MIN,   YAW_CAN_MAX,   16);
    float new_roll  = uint_to_float_impl(raw[2], ROLL_CAN_MIN,  ROLL_CAN_MAX,  16);

    if (first_frame) {
        /* 首帧: 初始化累加角度, 不计算delta */
        imu_data.yaw_total   = new_yaw;
        imu_data.roll_total  = new_roll;
        imu_data.pitch_total = new_pitch;
        first_frame = 0;
    } else {
        /* 计算delta并处理环绕 (±180 → ∓180 的跳变) */
        float d_yaw = new_yaw - prev_yaw;
        if (d_yaw > 180.0f)  d_yaw -= 360.0f;
        if (d_yaw < -180.0f) d_yaw += 360.0f;
        imu_data.yaw_total += d_yaw;

        float d_roll = new_roll - prev_roll;
        if (d_roll > 180.0f)  d_roll -= 360.0f;
        if (d_roll < -180.0f) d_roll += 360.0f;
        imu_data.roll_total += d_roll;

        float d_pitch = new_pitch - prev_pitch;
        /* pitch范围[-90,90], 环绕阈值用180 */
        if (d_pitch > 180.0f)  d_pitch -= 360.0f;
        if (d_pitch < -180.0f) d_pitch += 360.0f;
        imu_data.pitch_total += d_pitch;
    }

    prev_yaw   = new_yaw;
    prev_roll  = new_roll;
    prev_pitch = new_pitch;

    imu_data.pitch = new_pitch;
    imu_data.yaw   = new_yaw;
    imu_data.roll  = new_roll;
    imu_data.data_flags |= IMU_FLAG_EULER_READY;
}

/**
 * @brief  解析四元数数据帧 (pData[0]=0x04)
 * @param  pData  8字节CAN数据
 * @note   14-bit精度, 4个分量打包在7字节中
 */
static void DM_IMU_ParseQuaternion(uint8_t *pData)
{
    int w = (pData[1] << 6) | ((pData[2] & 0xF8) >> 2);
    int x = ((pData[2] & 0x03) << 12) | (pData[3] << 4) | ((pData[4] & 0xF0) >> 4);
    int y = ((pData[4] & 0x0F) << 10) | (pData[5] << 2) | ((pData[6] & 0xC0) >> 6);
    int z = ((pData[6] & 0x3F) << 8) | pData[7];

    imu_data.q[0] = uint_to_float_impl(w, Quaternion_CAN_MIN, Quaternion_CAN_MAX, 14);
    imu_data.q[1] = uint_to_float_impl(x, Quaternion_CAN_MIN, Quaternion_CAN_MAX, 14);
    imu_data.q[2] = uint_to_float_impl(y, Quaternion_CAN_MIN, Quaternion_CAN_MAX, 14);
    imu_data.q[3] = uint_to_float_impl(z, Quaternion_CAN_MIN, Quaternion_CAN_MAX, 14);
    imu_data.data_flags |= IMU_FLAG_QUATERNION_READY;
}

/**
 * @brief  IMU数据解析入口
 * @param  pData  FDCAN接收到的8字节数据
 * @note   根据pData[0]路由到对应解析函数, 更新时间戳和帧计数
 */
void DM_IMU_UpdateData(uint8_t *pData)
{
    switch (pData[0]) {
        case IMU_DATA_ACCEL:      DM_IMU_ParseAccel(pData);      break;
        case IMU_DATA_GYRO:       DM_IMU_ParseGyro(pData);       break;
        case IMU_DATA_EULER:      DM_IMU_ParseEuler(pData);      break;
        case IMU_DATA_QUATERNION: DM_IMU_ParseQuaternion(pData); break;
        default: break;
    }
    imu_data.frame_count++;
    imu_data.timestamp = HAL_GetTick();
}

/**
 * @brief  兼容旧接口, 转发到DM_IMU_UpdateData
 */
void IMU_UpdateData(uint8_t *pData)
{
    DM_IMU_UpdateData(pData);
}

/**
 * @brief  检查数据是否全部就绪
 * @return 1=加速度+角速度+欧拉角+四元数全部就绪, 0=部分缺失
 */
uint8_t DM_IMU_IsDataReady(void)
{
    return (imu_data.data_flags & IMU_FLAG_ALL_READY) == IMU_FLAG_ALL_READY;
}

/**
 * @brief  陀螺仪软件零偏校准
 * @note   校准流程:
 *   1. 临时清除gyro_offset, 读取原始值
 *   2. 采样IMU_CALI_SAMPLES次, 累加gyro和g_norm
 *   3. 计算平均值作为offset和g_norm
 *   4. 由g_norm推算accel_scale = 9.81 / g_norm
 *   5. 超时则使用默认值, 不阻塞系统启动
 */
void DM_IMU_CalibrateGyro(void)
{
    float sum[3] = {0, 0, 0};
    float sum_g_norm = 0;
    uint16_t count = 0;

    float old_offset[3];
    for (uint8_t i = 0; i < 3; i++) {
        old_offset[i] = imu_data.gyro_offset[i];
        imu_data.gyro_offset[i] = 0;
    }

    for (uint16_t i = 0; i < IMU_CALI_SAMPLES; i++) {
        uint32_t start = HAL_GetTick();
        while (imu_data.data_flags == 0) {
            if (HAL_GetTick() - start > 100) {
                for (uint8_t j = 0; j < 3; j++) {
                    imu_data.gyro_offset[j] = old_offset[j];
                }
                imu_data.accel_scale = 1.0f;
                return;
            }
        }

        sum[0] += imu_data.gyro[0];
        sum[1] += imu_data.gyro[1];
        sum[2] += imu_data.gyro[2];

        float g = sqrtf(imu_data.accel[0] * imu_data.accel[0] +
                        imu_data.accel[1] * imu_data.accel[1] +
                        imu_data.accel[2] * imu_data.accel[2]);
        sum_g_norm += g;

        imu_data.data_flags = 0;
        count++;
        HAL_Delay(1);
    }

    if (count > 0) {
        imu_data.gyro_offset[0] = sum[0] / (float)count;
        imu_data.gyro_offset[1] = sum[1] / (float)count;
        imu_data.gyro_offset[2] = sum[2] / (float)count;
        imu_data.g_norm = sum_g_norm / (float)count;
        imu_data.accel_scale = 9.81f / imu_data.g_norm;
        imu_data.is_calibrated = 1;
    } else {
        for (uint8_t i = 0; i < 3; i++) {
            imu_data.gyro_offset[i] = old_offset[i];
        }
    }
}
