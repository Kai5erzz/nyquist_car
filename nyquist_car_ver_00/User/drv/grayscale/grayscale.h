/**
 * @file    grayscale.h
 * @author  kaiser
 * @version V2.0.0
 * @date    2026-05-01
 * @brief   8通道灰度巡线传感器驱动
 *
 * 功能:
 *   - 3-bit地址选通8通道 (AD0/AD1/AD2)
 *   - ADC读取模拟值
 *   - 黑/白校准, 自动计算每通道上下界
 *   - 归一化插值巡线 (连续位置输出)
 *
 * 硬件:
 *   AD0(PC6) AD1(PA8) AD2(PH8) → 3-bit地址选择
 *   ADC1_INP8 (PC5) → 模拟输入
 *
 * 校准流程:
 *   Grayscale_Calibrate()
 *     → osDelay → 采样黑线 (每通道取min)
 *     → osDelay → 采样白底 (每通道取max)
 *     → 计算归一化参数
 *
 * 平滑算法:
 *   normalized[i] = (cal_max[i] - raw[i]) / (cal_max[i] - cal_min[i])
 *   line_error = Σ(i × normalized[i]) / Σ(normalized[i])
 *   全通道参与加权, 中间半响应通道自然产生插值效果
 */

#ifndef GRAYSCALE_H
#define GRAYSCALE_H

#include <stdint.h>

#define GRAYSCALE_CH_NUM    8
#define GRAYSCALE_CAL_SAMPLES 100    /**< 每次校准采样次数 */

/**
 * @brief 灰度传感器数据结构体
 */
typedef struct {
    /* 原始数据 */
    uint16_t raw[GRAYSCALE_CH_NUM];      /**< 各通道ADC原始值 */
    float    normalized[GRAYSCALE_CH_NUM];/**< 归一化值 [0.0, 1.0], 1=黑线, 0=白底 */
    uint8_t  digital[GRAYSCALE_CH_NUM];  /**< 数字化结果: 1=黑线, 0=白底 */
    uint8_t  digital_byte;               /**< 8通道打包为1字节 (bit0=ch0) */

    /* 校准数据 */
    uint16_t cal_min[GRAYSCALE_CH_NUM];  /**< 黑线ADC值 (每通道最小值) */
    uint16_t cal_max[GRAYSCALE_CH_NUM];  /**< 白底ADC值 (每通道最大值) */
    uint8_t  is_calibrated;              /**< 校准完成标志 */
} Grayscale_t;

extern Grayscale_t grayscale;

/**
 * @brief  初始化灰度传感器模块
 * @note   清零数据, 校准值设为默认 (min=0, max=65535)
 */
void Grayscale_Init(void);

/**
 * @brief  读取单个通道ADC原始值
 * @param  ch  通道号 [0, 7]
 * @return 16-bit ADC原始值
 */
uint16_t Grayscale_ReadChannel(uint8_t ch);

/**
 * @brief  读取全部8通道, 计算归一化值和数字化结果
 * @note   完整扫描约 0.5ms
 */
void Grayscale_ReadAll(void);

/**
 * @brief  一键校准函数 (阻塞, 约5秒)
 *
 * 流程:
 *   1. 串口提示 "Place on BLACK line" → osDelay 2秒
 *   2. 采样 GRAYSCALE_CAL_SAMPLES 次, 每通道取最小值
 *   3. 串口提示 "Place on WHITE surface" → osDelay 2秒
 *   4. 采样 GRAYSCALE_CAL_SAMPLES 次, 每通道取最大值
 *   5. 计算归一化参数, 设置 is_calibrated=1
 *
 * @note   需要在RTOS任务中调用 (使用osDelay)
 *         黑线上ADC值低, 白底上ADC值高
 */
void Grayscale_Calibrate(void);

/**
 * @brief  获取单通道归一化值
 * @param  ch  通道号 [0, 7]
 * @return [0.0, 1.0], 1=黑线, 0=白底
 */
float Grayscale_GetNormalized(uint8_t ch);

/**
 * @brief  获取单通道数字化结果
 * @param  ch  通道号 [0, 7]
 * @return 1=黑线, 0=白底
 */
uint8_t Grayscale_GetDigital(uint8_t ch);

/**
 * @brief  获取8通道打包字节
 * @return bit0=ch0, bit7=ch7
 */
uint8_t Grayscale_GetDigitalByte(void);

/**
 * @brief  获取巡线偏差 (归一化插值法)
 *
 * 算法:
 *   normalized[i] = (cal_max[i] - raw[i]) / (cal_max[i] - cal_min[i])
 *   line_error = Σ(i × normalized[i]) / Σ(normalized[i])
 *
 * 权重位置: [0, 1, 2, 3, 4, 5, 6, 7]
 *   居中偏移后: [-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]
 *
 * @return 连续偏差值, 0=居中, <0偏左, >0偏右
 *         全白时返回0
 */
float Grayscale_GetLineError(void);

#endif /* GRAYSCALE_H */
