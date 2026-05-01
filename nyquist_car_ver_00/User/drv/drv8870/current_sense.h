/**
 * @file    current_sense.h
 * @author  kaiser
 * @version V1.1.0
 * @date    2026-05-01
 * @brief   ADC电流采样模块 (含零偏校准)
 *
 * 功能:
 *   - 4通道ADC独立采样电机电流
 *   - 启动时自动校准零偏 (电机静止时采样取平均)
 *   - 电流 = (raw - offset) × gain
 *
 * ADC通道映射:
 *   MOTOR1: ADC CH18 (PA4) → MOTOR_ISEN1
 *   MOTOR2: ADC CH10 (PC0) → MOTOR_ISEN2
 *   MOTOR3: ADC CH11 (PC1) → MOTOR_ISEN3
 *   MOTOR4: ADC CH4  (PC4) → MOTOR_ISEN4
 */

#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdint.h>

#define CURRENT_SENSE_NUM    4
#define ADC_RESOLUTION       65535.0f
#define ADC_VREF             3.3f
#define R_SENSE              0.150f
#define CURRENT_SENSE_GAIN   (ADC_VREF / (ADC_RESOLUTION * R_SENSE))
#define CURRENT_CALI_SAMPLES 256        /**< 零偏校准采样次数 */

/**
 * @brief 电流采样数据结构体
 */
typedef struct {
    float    current[CURRENT_SENSE_NUM];    /**< 校准后电流值 (A) */
    uint16_t adc_raw[CURRENT_SENSE_NUM];    /**< ADC原始值 */
    int16_t  offset[CURRENT_SENSE_NUM];     /**< 零偏ADC值 (启动时校准) */
} Current_Sense_t;

extern Current_Sense_t current_sense;

/**
 * @brief  初始化并校准零偏
 * @note   电机必须静止, 采样CURRENT_CALI_SAMPLES次取平均作为offset
 */
void Current_Sense_Init(void);

/**
 * @brief  读取全部4个电机电流 (已减去零偏)
 */
void Current_Sense_ReadAll(void);

/**
 * @brief  获取指定电机电流 (A)
 */
float Current_Sense_GetCurrent(uint8_t index);

#endif /* CURRENT_SENSE_H */
