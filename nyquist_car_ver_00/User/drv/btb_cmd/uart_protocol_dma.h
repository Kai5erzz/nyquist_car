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

/* ==================== 命令码定义 ==================== */
#define BTB_CMD_YOLO_DETECT   0x01   /* YOLO识别结果 */

/* ==================== YOLO 数据结构 ==================== */
#define YOLO_MAX_TARGETS      2      /* 最多识别2个目标 */

/**
 * @brief 单个YOLO目标
 */
typedef struct {
    uint8_t  digit;          /**< 识别的数字 (0-9) */
    uint16_t x;              /**< 中心X坐标 (0-415) */
    uint16_t y;              /**< 中心Y坐标 (0-415) */
} YoloTarget_t;

/**
 * @brief YOLO识别结果
 */
typedef struct {
    uint8_t      count;                /**< 有效目标数量 (0-2) */
    YoloTarget_t targets[YOLO_MAX_TARGETS]; /**< 目标数组 */
} YoloDetect_t;

extern YoloDetect_t yolo_detect;       /**< 最新识别结果 */

/**
 * @brief  解析YOLO识别数据 (从RxPacket.data解析到yolo_detect)
 * @note   在收到 cmd=BTB_CMD_YOLO_DETECT 时调用
 *         数据格式: [digit, xH, xL, yH, yL] × N (每目标5字节, 大端)
 */
void Yolo_ParseFromPacket(void);

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