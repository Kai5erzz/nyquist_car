/**
 * @file    current_sense.c
 * @author  kaiser
 * @version V1.1.0
 * @date    2026-05-01
 * @brief   ADC电流采样实现 (含零偏校准)
 *
 * 零偏校准:
 *   启动时电机静止, 采样N次取平均作为offset
 *   运行时: current = (raw - offset) × gain
 */

#include "current_sense.h"
#include "adc.h"

Current_Sense_t current_sense;

static const uint32_t adc_channels[CURRENT_SENSE_NUM] = {
    ADC_CHANNEL_18,
    ADC_CHANNEL_10,
    ADC_CHANNEL_11,
    ADC_CHANNEL_4,
};

static uint16_t ADC_ReadChannel(uint32_t channel)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    uint16_t val = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    __set_PRIMASK(primask);
    return val;
}

/**
 * @brief  初始化并校准零偏
 * @note   电机静止时调用, 采样CURRENT_CALI_SAMPLES次取平均
 */
void Current_Sense_Init(void)
{
    int32_t sum[CURRENT_SENSE_NUM] = {0};

    /* 多次采样取平均 */
    for (uint16_t n = 0; n < CURRENT_CALI_SAMPLES; n++) {
        for (uint8_t i = 0; i < CURRENT_SENSE_NUM; i++) {
            sum[i] += ADC_ReadChannel(adc_channels[i]);
        }
    }

    for (uint8_t i = 0; i < CURRENT_SENSE_NUM; i++) {
        current_sense.offset[i] = (int16_t)(sum[i] / CURRENT_CALI_SAMPLES);
        current_sense.current[i] = 0;
        current_sense.adc_raw[i] = 0;
    }
}

/**
 * @brief  读取全部4个电机电流 (已减零偏)
 */
void Current_Sense_ReadAll(void)
{
    for (uint8_t i = 0; i < CURRENT_SENSE_NUM; i++) {
        current_sense.adc_raw[i] = ADC_ReadChannel(adc_channels[i]);
        int16_t corrected = (int16_t)current_sense.adc_raw[i] - current_sense.offset[i];
        current_sense.current[i] = (float)corrected * CURRENT_SENSE_GAIN;
    }
}

float Current_Sense_GetCurrent(uint8_t index)
{
    if (index >= CURRENT_SENSE_NUM) return 0;
    return current_sense.current[index];
}
