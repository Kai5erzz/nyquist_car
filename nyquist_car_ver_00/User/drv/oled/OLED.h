/**
 * @file OLED.h
 * @brief OLED显示驱动 - 硬件I2C版本
 * @date 2026-04-05
 */

#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

/* OLED I2C地址 (7位地址: 0x3C, 8位地址: 0x78) */
#define OLED_I2C_ADDR         0x78
// 直接使用0x78作为写地址，不需要再次左移
#define OLED_I2C_ADDR_WRITE   OLED_I2C_ADDR
#define OLED_I2C_ADDR_READ    (OLED_I2C_ADDR | 0x01)

/* OLED命令 */
#define OLED_CMD              0x00
#define OLED_DATA             0x40

/* ==================== 基础显示函数 ==================== */

/**
 * @brief OLED初始化
 */
void OLED_Init(void);

/**
 * @brief OLED清屏
 */
void OLED_Clear(void);

/**
 * @brief 清空显存缓冲区
 */
void OLED_ClearGRAM(void);

/**
 * @brief 更新显存到OLED屏幕
 */
void OLED_UpdateGRAM(void);

/**
 * @brief OLED显示一个字符
 * @param Line 行位置，范围：1~4
 * @param Column 列位置，范围：1~16
 * @param Char 要显示的字符
 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);

/**
 * @brief OLED显示字符串
 * @param Line 起始行位置，范围：1~4
 * @param Column 起始列位置，范围：1~16
 * @param String 要显示的字符串
 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);

/**
 * @brief OLED显示数字（十进制，无符号）
 * @param Line 起始行位置，范围：1~4
 * @param Column 起始列位置，范围：1~16
 * @param Number 要显示的数字，范围：0~4294967295
 * @param Length 要显示数字的长度，范围：1~10
 */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/**
 * @brief OLED显示数字（十进制，带符号）
 * @param Line 起始行位置，范围：1~4
 * @param Column 起始列位置，范围：1~16
 * @param Number 要显示的数字，范围：-2147483648~2147483647
 * @param Length 要显示数字的长度，范围：1~10
 */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);

/**
 * @brief OLED显示数字（十六进制）
 * @param Line 起始行位置，范围：1~4
 * @param Column 起始列位置，范围：1~16
 * @param Number 要显示的数字，范围：0~0xFFFFFFFF
 * @param Length 要显示数字的长度，范围：1~8
 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/**
 * @brief OLED显示数字（二进制）
 * @param Line 起始行位置，范围：1~4
 * @param Column 起始列位置，范围：1~16
 * @param Number 要显示的数字
 * @param Length 要显示数字的长度，范围：1~16
 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/* ==================== 浮点数显示函数 ==================== */

/**
 * @brief OLED显示浮点数（无符号）
 * @param Line 起始行位置，范围：1~4
 * @param Column 起始列位置，范围：1~16
 * @param Number 要显示的浮点数
 * @param IntLength 整数部分长度
 * @param FraLength 小数部分长度
 */
void OLED_ShowFloat(uint8_t Line, uint8_t Column, float Number, uint8_t IntLength, uint8_t FraLength);

/**
 * @brief OLED显示浮点数（带符号）
 * @param Line 起始行位置，范围：1~4
 * @param Column 起始列位置，范围：1~16
 * @param Number 要显示的浮点数
 * @param IntLength 整数部分长度
 * @param FraLength 小数部分长度
 */
void OLED_ShowSignedFloat(uint8_t Line, uint8_t Column, float Number, uint8_t IntLength, uint8_t FraLength);

/* ==================== 图形绘制函数 ==================== */

/**
 * @brief 画点
 * @param X 横坐标，范围：0~127
 * @param Y 纵坐标，范围：0~63
 * @param Color 颜色，1为点亮，0为熄灭
 */
void OLED_DrawPoint(uint8_t X, uint8_t Y, uint8_t Color);

/**
 * @brief 画线
 * @param X0,Y0 起点坐标
 * @param X1,Y1 终点坐标
 * @param Color 颜色
 */
void OLED_DrawLine(uint8_t X0, uint8_t Y0, uint8_t X1, uint8_t Y1, uint8_t Color);

/**
 * @brief 画矩形（空心）
 * @param X,Y 左上角坐标
 * @param Width 宽度
 * @param Height 高度
 * @param Color 颜色
 */
void OLED_DrawRectangle(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height, uint8_t Color);

/**
 * @brief 画矩形（实心）
 * @param X,Y 左上角坐标
 * @param Width 宽度
 * @param Height 高度
 * @param Color 颜色
 */
void OLED_DrawFilledRectangle(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height, uint8_t Color);

/**
 * @brief 画圆
 * @param X,Y 圆心坐标
 * @param Radius 半径
 * @param Color 颜色
 */
void OLED_DrawCircle(uint8_t X, uint8_t Y, uint8_t Radius, uint8_t Color);

/**
 * @brief 画圆（实心）
 * @param X,Y 圆心坐标
 * @param Radius 半径
 * @param Color 颜色
 */
void OLED_DrawFilledCircle(uint8_t X, uint8_t Y, uint8_t Radius, uint8_t Color);

/* ==================== 实用显示函数 ==================== */

/**
 * @brief 绘制进度条
 * @param X,Y 左上角坐标
 * @param Width 宽度
 * @param Height 高度
 * @param Progress 进度值，范围：0~100
 */
void OLED_DrawProgressBar(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height, uint8_t Progress);

/**
 * @brief 绘制电池图标
 * @param X,Y 左上角坐标
 * @param Level 电量等级，范围：0~100
 */
void OLED_DrawBattery(uint8_t X, uint8_t Y, uint8_t Level);

/**
 * @brief 反白显示指定区域
 * @param X,Y 左上角坐标
 * @param Width 宽度
 * @param Height 高度
 */
void OLED_Invert(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height);

/**
 * @brief 显示图像
 * @param X,Y 左上角坐标
 * @param Width 宽度（必须是8的倍数）
 * @param Height 高度（必须是8的倍数）
 * @param Image 图像数据数组
 */
void OLED_ShowImage(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image);

/* ==================== 高级功能 ==================== */

/**
 * @brief 格式化输出（类似printf）
 * @param Line 行位置，范围：1~4
 * @param Column 列位置，范围：1~16
 * @param format 格式化字符串
 */
void OLED_Printf(uint8_t Line, uint8_t Column, const char *format, ...);

#endif /* __OLED_H */
