/**
 * @file OLED.c
 * @brief OLED显示驱动实现 - 硬件I2C版本
 * @date 2026-04-05
 */

#include "OLED.h"
#include "OLED_Font.h"
#include "i2c.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

/* OLED显存缓冲区 128x64 = 8192 bits = 1024 bytes */
static uint8_t OLED_GRAM[128][8];

/**
 * @brief OLED写命令
 */
static void OLED_WriteCommand(uint8_t Command)
{
    uint8_t buf[2] = {OLED_CMD, Command};
    HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR_WRITE, buf, 2, 10);
}

/**
 * @brief OLED写数据
 */
static void OLED_WriteData(uint8_t Data)
{
    uint8_t buf[2] = {OLED_DATA, Data};
    HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR_WRITE, buf, 2, 10);
}

/**
 * @brief OLED设置光标位置
 * @param Y 以左上角为原点，向下方向的坐标，范围：0~7
 * @param X 以左上角为原点，向右方向的坐标，范围：0~127
 */
static void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);                   /* 设置Y位置 */
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));   /* 设置X位置高4位 */
    OLED_WriteCommand(0x00 | (X & 0x0F));           /* 设置X位置低4位 */
}

/**
 * @brief OLED初始化
 */
void OLED_Init(void)
{
    /* 上电延时 */
    HAL_Delay(100);

    OLED_WriteCommand(0xAE);    /* 关闭显示 */

    OLED_WriteCommand(0xD5);   /* 设置显示时钟分频比/振荡器频率 */
    OLED_WriteCommand(0x80);

    OLED_WriteCommand(0xA8);    /* 设置多路复用率 */
    OLED_WriteCommand(0x3F);

    OLED_WriteCommand(0xD3);    /* 设置显示偏移 */
    OLED_WriteCommand(0x00);

    OLED_WriteCommand(0x40);   /* 设置显示开始行 */

    OLED_WriteCommand(0xA1);   /* 设置左右方向，0xA1正常 0xA0左右反置 */

    OLED_WriteCommand(0xC8);   /* 设置上下方向，0xC8正常 0xC0上下反置 */

    OLED_WriteCommand(0xDA);   /* 设置COM引脚硬件配置 */
    OLED_WriteCommand(0x12);

    OLED_WriteCommand(0x81);   /* 设置对比度控制 */
    OLED_WriteCommand(0xCF);

    OLED_WriteCommand(0xD9);   /* 设置预充电周期 */
    OLED_WriteCommand(0xF1);

    OLED_WriteCommand(0xDB);   /* 设置VCOMH取消选择级别 */
    OLED_WriteCommand(0x30);

    OLED_WriteCommand(0xA4);   /* 设置整个显示打开/关闭 */

    OLED_WriteCommand(0xA6);   /* 设置正常/倒转显示 */

    OLED_WriteCommand(0x8D);   /* 设置充电泵 */
    OLED_WriteCommand(0x14);

    OLED_WriteCommand(0xAF);   /* 开启显示 */

    OLED_Clear();
    OLED_ClearGRAM();
}

/**
 * @brief OLED清屏
 */
void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++) {
        OLED_SetCursor(j, 0);
        for (i = 0; i < 128; i++) {
            OLED_WriteData(0x00);
        }
    }
}

/**
 * @brief 清空显存缓冲区
 */
void OLED_ClearGRAM(void)
{
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
}

/**
 * @brief OLED显示一个字符 (修改为写入GRAM缓存)
 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    // 将行列坐标转换为GRAM的X(0~127)和Page(0~7)坐标
    uint8_t x = (Column - 1) * 8;
    uint8_t page = (Line - 1) * 2; 

    // 防止越界
    if (x >= 128 || page >= 7) return;

    for (i = 0; i < 8; i++) {
        OLED_GRAM[x + i][page] = OLED_F8x16[Char - ' '][i];           /* 上半部分内容写入GRAM */
        OLED_GRAM[x + i][page + 1] = OLED_F8x16[Char - ' '][i + 8];   /* 下半部分内容写入GRAM */
    }
}

/**
 * @brief OLED显示字符串
 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

/**
 * @brief OLED次方函数
 */
static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) {
        Result *= X;
    }
    return Result;
}

/**
 * @brief OLED显示数字（十进制，无符号）
 */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
 * @brief OLED显示数字（十进制，带符号）
 */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t Number1;
    if (Number >= 0) {
        OLED_ShowChar(Line, Column, '+');
        Number1 = Number;
    } else {
        OLED_ShowChar(Line, Column, '-');
        Number1 = -Number;
    }
    for (i = 0; i < Length; i++) {
        OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
 * @brief OLED显示数字（十六进制）
 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++) {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10) {
            OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        } else {
            OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
}

/**
 * @brief OLED显示数字（二进制）
 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
}

/* ==================== 浮点数显示功能 ==================== */

/**
 * @brief OLED显示浮点数（无符号）
 */
void OLED_ShowFloat(uint8_t Line, uint8_t Column, float Number, uint8_t IntLength, uint8_t FraLength)
{
    uint8_t i;
    uint32_t IntPart, FraPart;

    if (Number < 0) Number = 0;

    IntPart = (uint32_t)Number;
    FraPart = (uint32_t)((Number - IntPart) * OLED_Pow(10, FraLength));

    /* 显示整数部分 */
    for (i = 0; i < IntLength; i++) {
        OLED_ShowChar(Line, Column + i, IntPart / OLED_Pow(10, IntLength - i - 1) % 10 + '0');
    }

    /* 显示小数点 */
    OLED_ShowChar(Line, Column + IntLength, '.');

    /* 显示小数部分 */
    for (i = 0; i < FraLength; i++) {
        OLED_ShowChar(Line, Column + IntLength + 1 + i, FraPart / OLED_Pow(10, FraLength - i - 1) % 10 + '0');
    }
}

/**
 * @brief OLED显示浮点数（带符号）
 */
void OLED_ShowSignedFloat(uint8_t Line, uint8_t Column, float Number, uint8_t IntLength, uint8_t FraLength)
{
    uint8_t i;
    uint32_t IntPart, FraPart;
    float AbsNumber;

    if (Number >= 0) {
        OLED_ShowChar(Line, Column, '+');
        AbsNumber = Number;
    } else {
        OLED_ShowChar(Line, Column, '-');
        AbsNumber = -Number;
    }

    IntPart = (uint32_t)AbsNumber;
    FraPart = (uint32_t)((AbsNumber - IntPart) * OLED_Pow(10, FraLength));

    /* 显示整数部分 */
    for (i = 0; i < IntLength; i++) {
        OLED_ShowChar(Line, Column + 1 + i, IntPart / OLED_Pow(10, IntLength - i - 1) % 10 + '0');
    }

    /* 显示小数点 */
    OLED_ShowChar(Line, Column + 1 + IntLength, '.');

    /* 显示小数部分 */
    for (i = 0; i < FraLength; i++) {
        OLED_ShowChar(Line, Column + IntLength + 2 + i, FraPart / OLED_Pow(10, FraLength - i - 1) % 10 + '0');
    }
}

/* ==================== 图形绘制功能 ==================== */

/**
 * @brief 更新显存到OLED屏幕
 */
void OLED_UpdateGRAM(void)
{
    uint8_t i, j;
    for (i = 0; i < 8; i++) {
        OLED_SetCursor(i, 0);
        for (j = 0; j < 128; j++) {
            OLED_WriteData(OLED_GRAM[j][i]);
        }
    }
}

/**
 * @brief 画点
 */
void OLED_DrawPoint(uint8_t X, uint8_t Y, uint8_t Color)
{
    if (X >= 128 || Y >= 64) return;

    if (Color)
        OLED_GRAM[X][Y / 8] |= (1 << (Y % 8));
    else
        OLED_GRAM[X][Y / 8] &= ~(1 << (Y % 8));
}

/**
 * @brief 画线（Bresenham算法）
 */
void OLED_DrawLine(uint8_t X0, uint8_t Y0, uint8_t X1, uint8_t Y1, uint8_t Color)
{
    int16_t dx = X1 > X0 ? X1 - X0 : X0 - X1;
    int16_t dy = Y1 > Y0 ? Y1 - Y0 : Y0 - Y1;
    int16_t sx = X0 < X1 ? 1 : -1;
    int16_t sy = Y0 < Y1 ? 1 : -1;
    int16_t err = dx - dy;
    int16_t e2;

    while (1) {
        OLED_DrawPoint(X0, Y0, Color);

        if (X0 == X1 && Y0 == Y1) break;

        e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            X0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            Y0 += sy;
        }
    }
}

/**
 * @brief 画矩形（空心）
 */
void OLED_DrawRectangle(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height, uint8_t Color)
{
    OLED_DrawLine(X, Y, X + Width - 1, Y, Color);
    OLED_DrawLine(X, Y, X, Y + Height - 1, Color);
    OLED_DrawLine(X + Width - 1, Y, X + Width - 1, Y + Height - 1, Color);
    OLED_DrawLine(X, Y + Height - 1, X + Width - 1, Y + Height - 1, Color);
}

/**
 * @brief 画矩形（实心）
 */
void OLED_DrawFilledRectangle(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height, uint8_t Color)
{
    uint8_t i, j;
    for (i = 0; i < Width; i++) {
        for (j = 0; j < Height; j++) {
            OLED_DrawPoint(X + i, Y + j, Color);
        }
    }
}

/**
 * @brief 画圆（中点圆算法）
 */
void OLED_DrawCircle(uint8_t X, uint8_t Y, uint8_t Radius, uint8_t Color)
{
    int16_t x = 0;
    int16_t y = Radius;
    int16_t d = 3 - 2 * Radius;

    while (x <= y) {
        OLED_DrawPoint(X + x, Y + y, Color);
        OLED_DrawPoint(X - x, Y + y, Color);
        OLED_DrawPoint(X + x, Y - y, Color);
        OLED_DrawPoint(X - x, Y - y, Color);
        OLED_DrawPoint(X + y, Y + x, Color);
        OLED_DrawPoint(X - y, Y + x, Color);
        OLED_DrawPoint(X + y, Y - x, Color);
        OLED_DrawPoint(X - y, Y - x, Color);

        if (d < 0)
            d = d + 4 * x + 6;
        else {
            d = d + 4 * (x - y) + 10;
            y--;
        }
        x++;
    }
}

/**
 * @brief 画圆（实心）
 */
void OLED_DrawFilledCircle(uint8_t X, uint8_t Y, uint8_t Radius, uint8_t Color)
{
    int16_t x = 0;
    int16_t y = Radius;
    int16_t d = 3 - 2 * Radius;

    while (x <= y) {
        /* 使用水平线填充 */
        OLED_DrawLine(X - x, Y + y, X + x, Y + y, Color);
        OLED_DrawLine(X - x, Y - y, X + x, Y - y, Color);
        OLED_DrawLine(X - y, Y + x, X + y, Y + x, Color);
        OLED_DrawLine(X - y, Y - x, X + y, Y - x, Color);

        if (d < 0)
            d = d + 4 * x + 6;
        else {
            d = d + 4 * (x - y) + 10;
            y--;
        }
        x++;
    }
}

/* ==================== 实用显示功能 ==================== */

/**
 * @brief 绘制进度条
 */
void OLED_DrawProgressBar(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height, uint8_t Progress)
{
    uint8_t FillWidth;

    if (Progress > 100) Progress = 100;

    /* 画边框 */
    OLED_DrawRectangle(X, Y, Width, Height, 1);

    /* 计算填充宽度 */
    FillWidth = (Width - 2) * Progress / 100;

    /* 填充进度 */
    if (FillWidth > 0)
        OLED_DrawFilledRectangle(X + 1, Y + 1, FillWidth, Height - 2, 1);

    /* 清空未填充部分 */
    if (FillWidth < Width - 2)
        OLED_DrawFilledRectangle(X + 1 + FillWidth, Y + 1, Width - 2 - FillWidth, Height - 2, 0);
}

/**
 * @brief 绘制电池图标
 */
void OLED_DrawBattery(uint8_t X, uint8_t Y, uint8_t Level)
{
    uint8_t FillWidth;

    if (Level > 100) Level = 100;

    /* 画电池外壳 18x10 */
    OLED_DrawRectangle(X, Y, 18, 10, 1);
    /* 画电池正极 */
    OLED_DrawFilledRectangle(X + 18, Y + 3, 2, 4, 1);

    /* 计算填充宽度 */
    FillWidth = 16 * Level / 100;

    /* 填充电量 */
    if (FillWidth > 0)
        OLED_DrawFilledRectangle(X + 1, Y + 1, FillWidth, 8, 1);

    /* 清空未填充部分 */
    if (FillWidth < 16)
        OLED_DrawFilledRectangle(X + 1 + FillWidth, Y + 1, 16 - FillWidth, 8, 0);
}

/**
 * @brief 反白显示指定区域
 */
void OLED_Invert(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height)
{
    uint8_t i, j;
    for (i = 0; i < Width; i++) {
        for (j = 0; j < Height; j++) {
            uint8_t x = X + i;
            uint8_t y = Y + j;
            if (x >= 128 || y >= 64) continue;

            /* 翻转像素 */
            OLED_GRAM[x][y / 8] ^= (1 << (y % 8));
        }
    }
}

/**
 * @brief 显示图像
 */
void OLED_ShowImage(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image)
{
    uint8_t i, j, page;
    uint8_t PageStart = Y / 8;
    uint8_t PageCount = Height / 8;

    for (page = 0; page < PageCount; page++) {
        for (i = 0; i < Width; i++) {
            if (X + i < 128 && PageStart + page < 8) {
                OLED_GRAM[X + i][PageStart + page] = Image[page * Width + i];
            }
        }
    }
}

/* ==================== 高级功能 ==================== */

/**
 * @brief 格式化输出（类似printf）
 */
void OLED_Printf(uint8_t Line, uint8_t Column, const char *format, ...)
{
    char buffer[32];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    OLED_ShowString(Line, Column, buffer);
}
