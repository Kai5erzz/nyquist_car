/**
 * @file    vofa_plus.c
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-05-01
 * @brief   VOFA+ JustFloat 协议驱动实现
 *
 * 协议格式:
 *   [ch0 float][ch1 float]...[chN float][0x00 0x00 0x80 0x7F]
 *   帧尾为IEEE 754 NaN, VOFA+据此识别帧边界
 */

#include "vofa_plus.h"
#include <string.h>

/** JustFloat协议固定帧尾 */
static const uint8_t VOFA_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};

/**
 * @brief  初始化VOFA+实例
 * @param  vofa           实例指针
 * @param  channel_count  通道数 (上限VOFA_MAX_CHANNELS)
 * @note   清零数据缓冲并计算发送长度
 */
void Vofa_Init(Vofa_Instance_t *vofa, uint8_t channel_count)
{
    if (vofa == NULL) return;
    vofa->channel_count = (channel_count > VOFA_MAX_CHANNELS) ? VOFA_MAX_CHANNELS : channel_count;
    memset(&vofa->packet, 0, sizeof(Vofa_Packet_t));
    vofa->tx_length = vofa->channel_count * sizeof(float) + sizeof(VOFA_TAIL);
}

/**
 * @brief  设置指定通道的浮点数据
 * @param  vofa           实例指针
 * @param  channel_index  通道索引 (0起始, 需 < channel_count)
 * @param  data           待发送的float值
 */
void Vofa_SetData(Vofa_Instance_t *vofa, uint8_t channel_index, float data)
{
    if (vofa == NULL || channel_index >= vofa->channel_count) return;
    vofa->packet.channels[channel_index] = data;
}

/**
 * @brief  阻塞发送JustFloat数据帧
 * @param  vofa   实例指针
 * @param  huart  UART句柄 (由调用者传入)
 * @note   在通道数据后追加4字节帧尾, 通过HAL_UART_Transmit阻塞发出
 */
void Vofa_Transmit(Vofa_Instance_t *vofa, UART_HandleTypeDef *huart)
{
    if (vofa == NULL || huart == NULL) return;

    uint8_t *tail_ptr = (uint8_t *)(&vofa->packet.channels[vofa->channel_count]);
    tail_ptr[0] = VOFA_TAIL[0];
    tail_ptr[1] = VOFA_TAIL[1];
    tail_ptr[2] = VOFA_TAIL[2];
    tail_ptr[3] = VOFA_TAIL[3];

    HAL_UART_Transmit(huart, (uint8_t *)&vofa->packet, vofa->tx_length, 2);
}
