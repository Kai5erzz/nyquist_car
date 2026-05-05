//
// Created by 欧克 on 2026/5/2.
// 改为中断逐字节接收
//

#include "uart_protocol_dma.h"
#include "usart.h"
#include <string.h>

/* --- 全局变量定义 --- */
volatile ProtocolPacket_t RxPacket;
volatile uint8_t Flag_NewDataReceived = 0;
YoloDetect_t yolo_detect;

/* --- 中断接收单字节缓冲 --- */
static uint8_t rx_byte;

/* 接收状态机 */
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

/* DMA发送缓冲 */
static uint8_t DMA_TxBuffer[DMA_RX_BUFFER_SIZE];

/* 校验和 */
static uint8_t Calc_Checksum(uint8_t id, uint8_t cmd, uint8_t len, const uint8_t *data) {
    uint8_t sum = id + cmd + len;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/* 状态机解析单字节 */
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
                s_rxState = STATE_WAIT_HEADER1;
            }
            break;

        case STATE_WAIT_DATA:
            s_tempPacket.data[s_rxDataIndex++] = byte;
            if (s_rxDataIndex >= s_tempPacket.len) s_rxState = STATE_WAIT_CHECKSUM;
            break;

        case STATE_WAIT_CHECKSUM:
            s_rxChecksum = Calc_Checksum(s_tempPacket.id, s_tempPacket.cmd, s_tempPacket.len, s_tempPacket.data);
            if (byte == s_rxChecksum) {
                memcpy((void *)&RxPacket, &s_tempPacket, sizeof(ProtocolPacket_t));
                Flag_NewDataReceived = 1;
            }
            s_rxState = STATE_WAIT_HEADER1;
            break;

        default:
            s_rxState = STATE_WAIT_HEADER1;
            break;
    }
}

/* ================== 外部接口函数 ================== */

/* 1. 初始化: 开启中断接收 (逐字节) */
void Protocol_Init_DMA(UART_HandleTypeDef *huart) {
    s_rxState = STATE_WAIT_HEADER1;
    HAL_UART_Receive_IT(huart, &rx_byte, 1);
}

/* 2. 发送 (保留DMA发送) */
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

    HAL_UART_Transmit_DMA(huart, DMA_TxBuffer, index);
}

/* 3. UART接收完成回调 (每收到1字节触发) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        Parse_Byte(rx_byte);
        /* 继续接收下一字节 */
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
}

/* 4. UART错误回调 (ORE/FE/NE等) */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        /* 清除错误标志, 重启接收 */
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        s_rxState = STATE_WAIT_HEADER1;
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
}

/* 看门狗: 中断接收模式下不需要, 保留空实现兼容头文件 */
void Protocol_DMA_Watchdog(UART_HandleTypeDef *huart) {
    (void)huart;
}

/* ==================== YOLO 数据解析 ==================== */
/**
 * @brief  解析YOLO识别数据包
 * @note   数据格式: 每目标5字节 [digit, xH, xL, yH, yL], 大端序
 */
void Yolo_ParseFromPacket(void)
{
    ProtocolPacket_t local;
    __disable_irq();
    memcpy(&local, (const void *)&RxPacket, sizeof(ProtocolPacket_t));
    __enable_irq();

    yolo_detect.count = 0;

    uint8_t num_targets = local.len / 5;
    if (num_targets > YOLO_MAX_TARGETS) num_targets = YOLO_MAX_TARGETS;

    for (uint8_t i = 0; i < num_targets; i++) {
        uint8_t offset = i * 5;
        yolo_detect.targets[i].digit = local.data[offset];
        yolo_detect.targets[i].x = (local.data[offset + 1] << 8) | local.data[offset + 2];
        yolo_detect.targets[i].y = (local.data[offset + 3] << 8) | local.data[offset + 4];
        yolo_detect.count++;
    }
}
