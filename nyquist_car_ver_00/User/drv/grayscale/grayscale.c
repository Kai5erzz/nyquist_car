/**
 * @file    grayscale.c
 * @author  kaiser
 * @version V2.0.0
 * @date    2026-05-01
 * @brief   8通道灰度巡线传感器驱动实现
 *
 * 归一化插值算法:
 *   每通道独立校准黑/白ADC值后:
 *     normalized[i] = (cal_max[i] - raw[i]) / (cal_max[i] - cal_min[i])
 *   黑线上 → normalized≈1, 白底上 → normalized≈0
 *   中间过渡通道的normalized在0~1之间, 自然产生亚像素插值效果
 *   所有通道加权平均得到连续位置, 比纯digital方法平滑得多
 */

#include "grayscale.h"
#include "adc.h"
#include "main.h"
#include "cmsis_os.h"

Grayscale_t grayscale;

/** 巡线偏差权重表 */
static const float line_weights[GRAYSCALE_CH_NUM] = {
    -3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f
};

/**
 * @brief  设置多路选择器地址
 * @param  ch  通道号 [0, 7]
 */
static void Grayscale_SetChannel(uint8_t ch)
{
    HAL_GPIO_WritePin(AD0_GPIO_Port, AD0_Pin, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD1_GPIO_Port, AD1_Pin, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD2_GPIO_Port, AD2_Pin, (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  读取ADC通道8的模拟值
 * @return 16-bit ADC原始值
 * @note   关中断保护, 防止与电流采样ADC读取冲突
 */
static uint16_t Grayscale_ADC_Read(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = ADC_CHANNEL_8;
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
 * @brief  初始化灰度传感器模块
 * @note   校准值初始化为满量程 (min=0, max=65535), 等效于不校准
 */
void Grayscale_Init(void)
{
    for (uint8_t i = 0; i < GRAYSCALE_CH_NUM; i++) {
        grayscale.raw[i] = 0;
        grayscale.normalized[i] = 0;
        grayscale.digital[i] = 0;
        grayscale.cal_min[i] = 0;
        grayscale.cal_max[i] = 65535;
    }
    grayscale.digital_byte = 0;
    grayscale.is_calibrated = 0;
}

/**
 * @brief  读取单个通道
 * @param  ch  通道号 [0, 7]
 * @return ADC原始值 (16-bit)
 */
uint16_t Grayscale_ReadChannel(uint8_t ch)
{
    if (ch >= GRAYSCALE_CH_NUM) return 0;
    Grayscale_SetChannel(ch);
    volatile uint32_t delay = 100;
    while (delay--);
    return Grayscale_ADC_Read();
}

/**
 * @brief  读取全部8通道并计算归一化值和数字化结果
 *
 * 归一化: normalized = (cal_max - raw) / (cal_max - cal_min)
 *   - 黑线上ADC低 → cal_max - raw 大 → normalized 接近 1
 *   - 白底上ADC高 → cal_max - raw 小 → normalized 接近 0
 *   - 限幅到 [0.0, 1.0]
 *
 * 数字化: normalized > 0.5 → 1(黑线), 否则 → 0(白底)
 */
void Grayscale_ReadAll(void)
{
    grayscale.digital_byte = 0;
    for (uint8_t i = 0; i < GRAYSCALE_CH_NUM; i++) {
        grayscale.raw[i] = Grayscale_ReadChannel(i);

        /* 归一化 */
        float span = (float)(grayscale.cal_max[i] - grayscale.cal_min[i]);
        if (span < 1.0f) span = 1.0f;
        float val = ((float)grayscale.cal_max[i] - (float)grayscale.raw[i]) / span;
        if (val < 0.0f) val = 0.0f;
        if (val > 1.0f) val = 1.0f;
        grayscale.normalized[i] = val;

        /* 数字化 (基于归一化值, 阈值0.5) */
        grayscale.digital[i] = (val > 0.5f) ? 1 : 0;
        grayscale.digital_byte |= (grayscale.digital[i] << i);
    }
}

/**
 * @brief  一键校准 (阻塞, 约5秒)
 *
 * 流程:
 *   1. 提示放黑线 → osDelay 2秒
 *   2. 采样N次, 每通道取ADC最小值 (黑线上ADC值最低)
 *   3. 提示放白底 → osDelay 2秒
 *   4. 采样N次, 每通道取ADC最大值 (白底上ADC值最高)
 *   5. 标记校准完成
 *
 * @note   必须在RTOS任务中调用
 */
void Grayscale_Calibrate(void)
{
    uint16_t sample;

    /* ===== 阶段1: 采样黑线 ===== */
    /* 提示 (可通过VOFA+或LED观察) */
    osDelay(2000);

    /* 初始化为极值 */
    for (uint8_t i = 0; i < GRAYSCALE_CH_NUM; i++) {
        grayscale.cal_min[i] = 65535;
    }

    /* 采样, 取每通道最小值 */
    for (uint16_t n = 0; n < GRAYSCALE_CAL_SAMPLES; n++) {
        for (uint8_t i = 0; i < GRAYSCALE_CH_NUM; i++) {
            sample = Grayscale_ReadChannel(i);
            if (sample < grayscale.cal_min[i]) {
                grayscale.cal_min[i] = sample;
            }
        }
    }

    /* ===== 阶段2: 采样白底 ===== */
    osDelay(2000);

    /* 初始化为极值 */
    for (uint8_t i = 0; i < GRAYSCALE_CH_NUM; i++) {
        grayscale.cal_max[i] = 0;
    }

    /* 采样, 取每通道最大值 */
    for (uint16_t n = 0; n < GRAYSCALE_CAL_SAMPLES; n++) {
        for (uint8_t i = 0; i < GRAYSCALE_CH_NUM; i++) {
            sample = Grayscale_ReadChannel(i);
            if (sample > grayscale.cal_max[i]) {
                grayscale.cal_max[i] = sample;
            }
        }
    }

    /* 防御: 确保 max > min */
    for (uint8_t i = 0; i < GRAYSCALE_CH_NUM; i++) {
        if (grayscale.cal_max[i] <= grayscale.cal_min[i]) {
            grayscale.cal_max[i] = grayscale.cal_min[i] + 1;
        }
    }

    grayscale.is_calibrated = 1;
}

/**
 * @brief  获取单通道归一化值
 * @param  ch  通道号 [0, 7]
 * @return [0.0, 1.0], 1=黑线, 0=白底
 */
float Grayscale_GetNormalized(uint8_t ch)
{
    return (ch < GRAYSCALE_CH_NUM) ? grayscale.normalized[ch] : 0;
}

/**
 * @brief  获取单通道数字化结果
 * @param  ch  通道号 [0, 7]
 * @return 1=黑线, 0=白底
 */
uint8_t Grayscale_GetDigital(uint8_t ch)
{
    return (ch < GRAYSCALE_CH_NUM) ? grayscale.digital[ch] : 0;
}

/**
 * @brief  获取8通道打包字节
 * @return bit0=ch0, bit7=ch7
 */
uint8_t Grayscale_GetDigitalByte(void)
{
    return grayscale.digital_byte;
}

/**
 * @brief  获取巡线偏差 (归一化插值法)
 *
 * 算法:
 *   position = Σ(i × normalized[i]) / Σ(normalized[i])
 *   居中偏移: error = position - 3.5
 *
 *   normalized[i] ∈ [0, 1], 黑线通道值大, 白底通道值小
 *   中间过渡通道的半响应值自然参与插值
 *   结果为连续值, 不再是离散的8级跳变
 *
 * @return 偏差值, 0=居中, <0偏左, >0偏右, 全白返回0
 */
float Grayscale_GetLineError(void)
{
    float weighted_sum = 0;
    float weight_sum = 0;

    for (uint8_t i = 0; i < GRAYSCALE_CH_NUM; i++) {
        weighted_sum += (float)i * grayscale.normalized[i];
        weight_sum += grayscale.normalized[i];
    }

    if (weight_sum < 0.01f) return 0;
    return (weighted_sum / weight_sum) - 3.5f;
}
