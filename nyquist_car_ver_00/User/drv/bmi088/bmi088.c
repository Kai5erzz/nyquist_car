#include "bmi088.h"

// 全局结构体实例
DM_IMU_Data_t imu_data;

/**
 * @brief  无符号整数转换为浮点数函数 [cite: 443]
 * @note   将给定的无符号整数在指定范围内进行线性映射 [cite: 445]
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief  向 IMU 发送请求帧 (适用于应答模式)
 * @param  hfdcan: FDCAN 句柄 (例如 &hfdcan1)
 * @param  can_id: 上位机设置的 CAN ID [cite: 172]
 * @param  reg:    要读取或操作的寄存器地址 (例如 0x01 请求加速度) [cite: 183]
 */
void IMU_RequestData(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t reg)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0}; 
    
    // 构造请求数据域 [cite: 174]
    tx_data[0] = 0xCC;
    tx_data[1] = reg;       // RID (寄存器)
    tx_data[2] = 0x00;      // 00=读, 01=写 (此处默认为读请求)
    tx_data[3] = 0xDD;

    tx_header.Identifier = can_id;
    tx_header.IdType = FDCAN_STANDARD_ID;      // 标准帧 [cite: 170]
    tx_header.TxFrameType = FDCAN_DATA_FRAME;  // 数据帧 [cite: 169]
    tx_header.DataLength = FDCAN_DLC_BYTES_8;  // DLC 为 8 [cite: 171]
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;    // 模块使用标准CAN，非FD模式
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    // 发送数据到 TX FIFO
    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);
    }
}

/**
 * @brief  解析加速度计数据 [cite: 191]
 */
static void IMU_UpdateAccel(uint8_t* pData)
{
    uint16_t accel[3];
    imu_data.temp = pData[1]; // 获取温度
    
    accel[0] = (pData[3] << 8) | pData[2];
    accel[1] = (pData[5] << 8) | pData[4];
    accel[2] = (pData[7] << 8) | pData[6];
    
    imu_data.accel[0] = uint_to_float(accel[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    imu_data.accel[1] = uint_to_float(accel[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    imu_data.accel[2] = uint_to_float(accel[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
}

/**
 * @brief  解析角速度数据 [cite: 193]
 */
static void IMU_UpdateGyro(uint8_t* pData)
{
    uint16_t gyro[3];
    
    gyro[0] = (pData[3] << 8) | pData[2];
    gyro[1] = (pData[5] << 8) | pData[4];
    gyro[2] = (pData[7] << 8) | pData[6];
    
    imu_data.gyro[0] = uint_to_float(gyro[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    imu_data.gyro[1] = uint_to_float(gyro[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    imu_data.gyro[2] = uint_to_float(gyro[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
}

/**
 * @brief  解析欧拉角数据 [cite: 195]
 */
static void IMU_UpdateEuler(uint8_t* pData)
{
    uint16_t euler[3];
    
    euler[0] = (pData[3] << 8) | pData[2]; // Pitch
    euler[1] = (pData[5] << 8) | pData[4]; // Yaw
    euler[2] = (pData[7] << 8) | pData[6]; // Roll
    
    imu_data.pitch = uint_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
    imu_data.yaw   = uint_to_float(euler[1], YAW_CAN_MIN,   YAW_CAN_MAX,   16);
    imu_data.roll  = uint_to_float(euler[2], ROLL_CAN_MIN,  ROLL_CAN_MAX,  16);
}

/**
 * @brief  解析四元数数据 [cite: 197]
 */
static void IMU_UpdateQuaternion(uint8_t* pData)
{
    // 四元数数据为 14 位映射值 [cite: 199]
    int w = (pData[1] << 6) | ((pData[2] & 0xF8) >> 2);
    int x = ((pData[2] & 0x03) << 12) | (pData[3] << 4) | ((pData[4] & 0xF0) >> 4);
    int y = ((pData[4] & 0x0F) << 10) | (pData[5] << 2) | ((pData[6] & 0xC0) >> 6);
    int z = ((pData[6] & 0x3F) << 8) | pData[7];
    
    imu_data.q[0] = uint_to_float(w, Quaternion_CAN_MIN, Quaternion_CAN_MAX, 14);
    imu_data.q[1] = uint_to_float(x, Quaternion_CAN_MIN, Quaternion_CAN_MAX, 14);
    imu_data.q[2] = uint_to_float(y, Quaternion_CAN_MIN, Quaternion_CAN_MAX, 14);
    imu_data.q[3] = uint_to_float(z, Quaternion_CAN_MIN, Quaternion_CAN_MAX, 14);
}

/**
 * @brief  IMU 顶层数据路由函数
 * @note   将此函数放置于 FDCAN_RxFifo0Callback 或 FDCAN_RxFifo1Callback 中
 * @param  pData: FDCAN 接收到的 8 字节数据指针
 */
void IMU_UpdateData(uint8_t* pData)
{
    // 依据 DATA[0] 区分数据帧类型 [cite: 191, 193, 195, 197]
    switch(pData[0])
    {
        case 0x01:
            IMU_UpdateAccel(pData);
            break;
        case 0x02:
            IMU_UpdateGyro(pData);
            break;
        case 0x03:
            IMU_UpdateEuler(pData);
            break;
        case 0x04:
            IMU_UpdateQuaternion(pData);
            break;
        default:
            break;
    }
}