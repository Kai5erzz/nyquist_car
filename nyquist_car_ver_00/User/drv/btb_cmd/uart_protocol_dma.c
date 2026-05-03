//
// Created by 欧克 on 2026/5/2.
//

#include "uart_protocol_dma.h"
#include "usart.h"
#include <string.h>

/* --- 全局变量定义 --- */
ProtocolPacket_t RxPacket;     // 暴露给外部的数据包
uint8_t Flag_NewDataReceived = 0; // 暴露给外部的标志位

/* --- 内部私有变量 --- */
static uint8_t DMA_RxBuffer[DMA_RX_BUFFER_SIZE]; // DMA专属搬运缓冲区
static uint8_t DMA_TxBuffer[DMA_RX_BUFFER_SIZE]; // DMA专属发送缓冲区

/* 接收状态机枚举 */
typedef enum {
    STATE_WAIT_HEADER1 = 0,
    STATE_WAIT_HEADER2,
    STATE_WAIT_ID,
    STATE_WAIT_CMD,
    STATE_WAIT_LEN,
    STATE_WAIT_DATA,
    STATE_WAIT_CHECKSUM
} ParseState_t;

static ParseState_t s_rxState = STATE_WAIT_HEADER1;
static uint8_t s_rxChecksum = 0;
static uint8_t s_rxDataIndex = 0;
static ProtocolPacket_t s_tempPacket;

/* --- 内部私有函数：计算累加校验和 --- */
static uint8_t Calc_Checksum(uint8_t id, uint8_t cmd, uint8_t len, const uint8_t *data) {
    uint8_t sum = id + cmd + len;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/* --- 内部私有函数：状态机解析单字节 --- */
static void Parse_Byte(uint8_t byte) {
    switch (s_rxState) {
        case STATE_WAIT_HEADER1:
            if (byte == PROTOCOL_HEADER1) s_rxState = STATE_WAIT_HEADER2;
            break;

        case STATE_WAIT_HEADER2:
            if (byte == PROTOCOL_HEADER2) s_rxState = STATE_WAIT_ID;
            else if (byte != PROTOCOL_HEADER1) s_rxState = STATE_WAIT_HEADER1;
            break;

        case STATE_WAIT_ID:
            s_tempPacket.id = byte;
            s_rxState = STATE_WAIT_CMD;
            break;

        case STATE_WAIT_CMD:
            s_tempPacket.cmd = byte;
            s_rxState = STATE_WAIT_LEN;
            break;

        case STATE_WAIT_LEN:
            if (byte <= PROTOCOL_MAX_DATA) {
                s_tempPacket.len = byte;
                s_rxDataIndex = 0;
                s_rxState = (byte == 0) ? STATE_WAIT_CHECKSUM : STATE_WAIT_DATA;
            } else {
                s_rxState = STATE_WAIT_HEADER1; // 数据超长，复位状态机
            }
            break;

        case STATE_WAIT_DATA:
            s_tempPacket.data[s_rxDataIndex++] = byte;
            if (s_rxDataIndex >= s_tempPacket.len) s_rxState = STATE_WAIT_CHECKSUM;
            break;

        case STATE_WAIT_CHECKSUM:
            s_rxChecksum = Calc_Checksum(s_tempPacket.id, s_tempPacket.cmd, s_tempPacket.len, s_tempPacket.data);
            if (byte == s_rxChecksum) {
                // 校验通过，拷贝到全局结构体，通知主函数
                memcpy(&RxPacket, &s_tempPacket, sizeof(ProtocolPacket_t));
                Flag_NewDataReceived = 1;
            }
            s_rxState = STATE_WAIT_HEADER1; // 解析完成，准备接下一帧
            break;

        default:
            s_rxState = STATE_WAIT_HEADER1;
            break;
    }
}

/* ================== 外部接口函数 ================== */

/* 1. 初始化 DMA 接收 */
void Protocol_Init_DMA(UART_HandleTypeDef *huart) {
    // 开启 串口 DMA 接收 + 空闲中断 (Receive To Idle)
    HAL_UARTEx_ReceiveToIdle_DMA(huart, DMA_RxBuffer, DMA_RX_BUFFER_SIZE);
}

/* 2. DMA 组包与发送 */
void Protocol_Send_DMA(UART_HandleTypeDef *huart, uint8_t cmd, const uint8_t *pData, uint8_t len) {
    if (len > PROTOCOL_MAX_DATA) return;

    uint16_t index = 0;
    DMA_TxBuffer[index++] = PROTOCOL_HEADER1;
    DMA_TxBuffer[index++] = PROTOCOL_HEADER2;
    DMA_TxBuffer[index++] = PROTOCOL_MY_ID;
    DMA_TxBuffer[index++] = cmd;
    DMA_TxBuffer[index++] = len;

    for (uint8_t i = 0; i < len; i++) {
        DMA_TxBuffer[index++] = pData[i];
    }

    DMA_TxBuffer[index] = Calc_Checksum(PROTOCOL_MY_ID, cmd, len, pData);
    index++;

    // 调用 DMA 发送 (注意：连续发送需确保上一次发送已完成，可判断 huart->gState)
    HAL_UART_Transmit_DMA(huart, DMA_TxBuffer, index);
}

/* 3. DMA 空闲中断回调处理（喂给状态机） */
void Protocol_DMA_RxEvent_Handler(UART_HandleTypeDef *huart, uint16_t Size) {
    // Size 就是 DMA 刚刚自动搬运了多少个字节
    // 我们把这些字节挨个喂给状态机
    for (uint16_t i = 0; i < Size; i++) {
        Parse_Byte(DMA_RxBuffer[i]);
    }

    // 极其重要：一波数据处理完后，必须重新开启 DMA 接收，准备接下一波！
    HAL_UARTEx_ReceiveToIdle_DMA(huart, DMA_RxBuffer, DMA_RX_BUFFER_SIZE);
}
/**************************使用案例************************/
//
//  #include "uart_protocol_dma.h" // 引入我们刚刚写的库
//
// // 假设这些是你单片机里实时读取或计算出的四个电机速度
// int16_t motor1_speed = 1000;   // 正转 1000
// int16_t motor2_speed = -500;   // 反转 500
// int16_t motor3_speed = 0;      // 停止
// int16_t motor4_speed = 2000;   // 正转 2000
//
// /**
//  * @brief  打包并发送四个电机的速度给上位机
//  */
// void Report_Motor_Speeds(void) {
//     uint8_t payload[8]; // 准备一个 8 字节的数组作为载荷
//
//     // ----------------------------------------------------
//     // 开始“劈数据” (依然采用大端模式：先发高位，再发低位)
//     // ----------------------------------------------------
//
//     // 电机1
//     payload[0] = (uint8_t)(motor1_speed >> 8);   // 把前8位推到最右边提取出来 (高位)
//     payload[1] = (uint8_t)(motor1_speed & 0xFF); // 戴上 0xFF 面具，只保留后8位 (低位)
//
//     // 电机2 (负数在单片机底层是以补码存储的，位运算完全兼容，不用担心)
//     payload[2] = (uint8_t)(motor2_speed >> 8);
//     payload[3] = (uint8_t)(motor2_speed & 0xFF);
//
//     // 电机3
//     payload[4] = (uint8_t)(motor3_speed >> 8);
//     payload[5] = (uint8_t)(motor3_speed & 0xFF);
//
//     // 电机4
//     payload[6] = (uint8_t)(motor4_speed >> 8);
//     payload[7] = (uint8_t)(motor4_speed & 0xFF);
//
//     // ----------------------------------------------------
//     // 调用协议库发送
//     // ----------------------------------------------------
//     // 假设我们约定 0x30 这个命令码代表 "单片机上报电机速度"
//     Protocol_Send_DMA(&huart1, 0x30, payload, 8);
// }
