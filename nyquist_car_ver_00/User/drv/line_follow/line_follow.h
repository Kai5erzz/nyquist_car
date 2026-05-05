/**
 * @file    line_follow.h
 * @author  kaiser
 * @version V1.1.0
 * @date    2026-05-03
 * @brief   巡线库 - 分数式弯道/十字检测，多重证据交叉验证
 *
 * 改进点 (vs V1.0):
 *   - 弯道改用 score-based 连续帧确认，替换原比例窗口
 *   - 引入 line_error 作为方向二次证据
 *   - 严格非对称约束 (对侧 mask 必须 0 位激活)
 *   - 弯道总位数上限，防止"半个十字"被误判为弯道
 *   - 优先级仲裁: CROSS > TURN > STRAIGHT 互相抑制
 */

#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

/* ==================== 巡线PID参数 ==================== */
#define LINE_KP             150.0f   /**< 比例增益 */
#define LINE_KD             3.0f    /**< 微分增益 */
#define LINE_BASE_SPEED     50.0f   /**< 直行基础速度 (rad/s) */
#define LINE_SEARCH_SPEED   70.0f   /**< 丢线搜索旋转速度 (rad/s) */
#define LINE_TURN_SPEED     100.0f   /**< 弯道基础速度 (rad/s) */
#define LINE_TURN_RATIO     0.5f    /**< 弯道内侧速度比例 */

/* ==================== 检测掩码 ==================== */
#define LEFT_TURN_MASK      0x07    /**< 左侧3位 (bit0-2) */
#define RIGHT_TURN_MASK     0xE0    /**< 右侧3位 (bit5-7) */

/* ==================== 模式判定阈值 ==================== */
#define LEFT_BITS_MIN          2    /**< 左转模式: 左mask中至少几位激活 */
#define RIGHT_BITS_MIN         2    /**< 右转模式: 右mask中至少几位激活 */
#define TOTAL_BITS_MAX         5    /**< 弯道帧总位数上限 (>5则归为十字候选) */
#define CROSS_POPCOUNT         6    /**< 十字帧: 同时激活的位数阈值 */
#define ERROR_CONFIRM_BIAS     1.0f /**< 加权误差需达到此值才二次证实方向 */

/* ==================== 分数式确认参数 ==================== */
/**
 * 调参建议:
 *   - TURN_CONFIRM_THRESHOLD 越大越保守 (不易误触发, 但触发越晚)
 *     默认 3 = 大约连续 3 帧强左/右模式才触发 (相邻一帧反向证据会扣分)
 *     如果发现还是误触发, 可调到 4 或 5
 *     如果发现弯道触发太晚导致过头, 可调到 2
 *   - TURN_SCORE_MAX 控制饱和上限, 避免长期累积导致退出弯道时延迟
 */
#define TURN_CONFIRM_THRESHOLD  3   /**< 弯道分数 >= 此值确认 */
#define CROSS_CONFIRM_THRESHOLD 4   /**< 十字分数 >= 此值确认 (cross每帧+2, 即2帧确认) */
#define TURN_SCORE_MAX          8   /**< 分数饱和上限 */

/* ==================== 检测结果 ==================== */
typedef enum {
    LINE_STRAIGHT = 0,  /**< 直行 */
    LINE_LEFT_TURN,     /**< 左弯道 */
    LINE_RIGHT_TURN,    /**< 右弯道 */
    LINE_CROSS,         /**< 十字路口 */
    LINE_LOST           /**< 丢线 */
} LineFollow_Result_e;

/* ==================== 接口函数 ==================== */

/**
 * @brief  初始化巡线库 (复位所有分数与状态)
 *
 * 注意: 每次从角度环转回巡线时必须调用一次, 防止上一段累积的分数误触发
 */
void LineFollow_Init(void);

/**
 * @brief  更新巡线状态 (每周期调用一次)
 * @param  digital_byte  灰度数字字节 (bit0=最左, bit7=最右, 1=黑线)
 * @param  line_error    灰度加权误差 [-3.5, +3.5], 负=线在左, 正=线在右
 */
void LineFollow_Update(uint8_t digital_byte, float line_error);

/**
 * @brief  获取当前检测结果
 */
LineFollow_Result_e LineFollow_GetResult(void);

/**
 * @brief  根据检测结果计算左右轮速
 * @param  left   左轮速度输出指针 (rad/s)
 * @param  right  右轮速度输出指针 (rad/s)
 */
void LineFollow_GetSpeed(float *left, float *right);

#endif /* LINE_FOLLOW_H */
