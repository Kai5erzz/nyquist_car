//
// Created by 欧克 on 2026/5/2.
//

#ifndef CAR_CHASSIS_UART_PROTOCOL_DMA_H
#define CAR_CHASSIS_UART_PROTOCOL_DMA_H

#include <stdint.h>
#include <stdbool.h>
// 请根据你的芯片型号替换头文件，例如stm32f4xx_hal.h 或 stm32g4xx_hal.h
#include "stm32h7xx_hal.h"

/* --- 协议参数配置 --- */
#define PROTOCOL_HEADER1    0x55
#define PROTOCOL_HEADER2    0xAA
#define PROTOCOL_MY_ID      0x01   // 本机设备ID
#define PROTOCOL_MAX_DATA   128     // 最大支持的数据载荷长度
#define DMA_RX_BUFFER_SIZE  256    // DMA接收缓冲区大小（需大于单帧最大长度）

/* --- 数据包结构体 --- */
typedef struct {
    uint8_t id;                       // 设备ID
    uint8_t cmd;                      // 命令码
    uint8_t len;                      // 数据长度
    uint8_t data[PROTOCOL_MAX_DATA];  // 数据载荷
} ProtocolPacket_t;

/* --- 外部引用的全局变量 (供 main.c 使用) --- */
extern ProtocolPacket_t RxPacket;     // 解析成功的完整数据包
extern uint8_t Flag_NewDataReceived;  // 新数据接收标志位

/* --- 函数声明 --- */

/**
 * @brief  组包发送函数：将用户数据封装为协议帧并使用DMA发送
 * @param  huart      串口句柄
 * @param  cmd        命令码
 * @param  pData      要发送的数据指针
 * @param  len        数据长度
 */
void Protocol_Send_DMA(UART_HandleTypeDef *huart, uint8_t cmd, const uint8_t *pData, uint8_t len);

/**
 * @brief  初始化串口DMA空闲接收（在main函数初始化完成后调用一次）
 * @param  huart      串口句柄
 */
void Protocol_Init_DMA(UART_HandleTypeDef *huart);

/**
 * @brief  DMA空闲中断回调处理函数（在 HAL_UARTEx_RxEventCallback 中调用）
 * @param  huart      串口句柄
 * @param  Size       DMA本次接收到的字节数
 */
void Protocol_DMA_RxEvent_Handler(UART_HandleTypeDef *huart, uint16_t Size);
#endif //CAR_CHASSIS_UART_PROTOCOL_DMA_H