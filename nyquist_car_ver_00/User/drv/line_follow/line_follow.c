/**
 * @file    line_follow.c
 * @author  kaiser
 * @version V1.1.0
 * @date    2026-05-03
 * @brief   巡线库实现 - 分数式确认 + 多重证据交叉验证
 *
 * ============================================================================
 * 检测算法说明
 * ============================================================================
 *
 * 【模式判定 (单帧)】
 *   每来一帧, 先判断它符合哪种"候选模式":
 *
 *   - LEFT 候选帧: 必须同时满足
 *       (a) LEFT_TURN_MASK 中至少 LEFT_BITS_MIN 位激活
 *       (b) RIGHT_TURN_MASK 中 0 位激活      ← 严格非对称
 *       (c) 总激活位数 <= TOTAL_BITS_MAX     ← 排除十字
 *       (d) line_error <= -ERROR_CONFIRM_BIAS ← 误差二次证实
 *
 *   - RIGHT 候选帧: 镜像
 *
 *   - CROSS 候选帧: 总激活位数 >= CROSS_POPCOUNT
 *
 *   - 其他: 中性帧
 *
 * 【分数累积/衰减】
 *   - CROSS 候选: cross_score +2, left/right_score -1
 *   - LEFT  候选: left_score  +1, right_score -2 (强惩罚反向), cross_score -1
 *   - RIGHT 候选: 镜像
 *   - 中性帧:    所有分数 -1 (慢衰减)
 *   - LOST帧:    所有分数 -1
 *
 *   反向帧扣2分, 意味着 1 帧反向证据可以抵消 2 帧同向积累 → 强抗噪
 *
 * 【优先级仲裁】
 *   CROSS_score >= 阈值 → LINE_CROSS
 *   else LEFT_score >= 阈值 → LINE_LEFT_TURN
 *   else RIGHT_score >= 阈值 → LINE_RIGHT_TURN
 *   else 全0且digital==0 → LINE_LOST
 *   else → LINE_STRAIGHT
 *
 * 【为什么能解决"短线误触发"】
 *   短侧刺(如0x1C 持续1-2帧): bits=3, 但 lc=1 (不到 LEFT_BITS_MIN=2),
 *   或者 err 不达 1.0, 直接被否决, 分数不增长.
 *   即使个别帧侥幸通过, 也凑不齐连续 3 帧的分数门槛.
 *
 * 【为什么能解决"转反"】
 *   原方案 ratio>=0.6 + ratio<=0.2 一组数据可能被噪声歪曲.
 *   新方案要求 digital 与 line_error 方向一致, 两套传感口径互相佐证,
 *   除非两套全错(几乎不可能), 否则不会反向触发.
 * ============================================================================
 */

#include "line_follow.h"
#include <math.h>

/* ==================== 内部状态 ==================== */
static uint8_t  current_digital;            /**< 当前帧 digital_byte */
static float    current_error;              /**< 当前帧 line_error */
static float    prev_error;                 /**< 上一帧 line_error (用于 D 项) */
static float    last_error_sign;            /**< 丢线后的搜索方向 (-1=左, +1=右) */
static LineFollow_Result_e current_result;  /**< 当前检测结果 */

/* 分数累加器 (核心) */
static uint8_t  left_score;
static uint8_t  right_score;
static uint8_t  cross_score;

/* LOST 方向回溯的轻量缓冲 (仅用于丢线后判断往哪边搜索) */
#define LOST_LOOKBACK_FRAMES   6
static uint8_t  recent_digital[LOST_LOOKBACK_FRAMES];
static uint8_t  recent_index;

/* ==================== 内部工具函数 ==================== */

/** 计算 uint8 中置位位数 (popcount) */
static uint8_t popcount8(uint8_t x)
{
    uint8_t c = 0;
    while (x) { c += x & 1; x >>= 1; }
    return c;
}

/** 分数加 (饱和到 TURN_SCORE_MAX) */
static uint8_t score_inc(uint8_t s, uint8_t step)
{
    uint16_t r = (uint16_t)s + step;
    return (r > TURN_SCORE_MAX) ? TURN_SCORE_MAX : (uint8_t)r;
}

/** 分数减 (饱和到 0) */
static uint8_t score_dec(uint8_t s, uint8_t step)
{
    return (s < step) ? 0 : (s - step);
}

/** 是否符合"左转候选帧" */
static uint8_t is_left_pattern(uint8_t b, float err)
{
    uint8_t lc = popcount8(b & LEFT_TURN_MASK);
    uint8_t rc = popcount8(b & RIGHT_TURN_MASK);
    uint8_t tc = popcount8(b);
    return (lc >= LEFT_BITS_MIN)
        && (rc == 0)
        && (tc <= TOTAL_BITS_MAX)
        && (err <= -ERROR_CONFIRM_BIAS);
}

/** 是否符合"右转候选帧" */
static uint8_t is_right_pattern(uint8_t b, float err)
{
    uint8_t lc = popcount8(b & LEFT_TURN_MASK);
    uint8_t rc = popcount8(b & RIGHT_TURN_MASK);
    uint8_t tc = popcount8(b);
    return (rc >= RIGHT_BITS_MIN)
        && (lc == 0)
        && (tc <= TOTAL_BITS_MAX)
        && (err >= ERROR_CONFIRM_BIAS);
}

/** 是否符合"十字候选帧" */
static uint8_t is_cross_pattern(uint8_t b)
{
    return popcount8(b) >= CROSS_POPCOUNT;
}

/* ==================== 接口函数实现 ==================== */

void LineFollow_Init(void)
{
    for (uint8_t i = 0; i < LOST_LOOKBACK_FRAMES; i++) {
        recent_digital[i] = 0;
    }
    recent_index = 0;

    current_digital = 0;
    current_error = 0;
    prev_error = 0;
    last_error_sign = 1.0f;
    current_result = LINE_STRAIGHT;

    left_score = 0;
    right_score = 0;
    cross_score = 0;
}

void LineFollow_Update(uint8_t digital_byte, float line_error)
{
    current_digital = digital_byte;
    current_error = line_error;

    /* 推入回溯缓冲区 (用于 LOST 方向决策) */
    recent_digital[recent_index] = digital_byte;
    recent_index = (recent_index + 1) % LOST_LOOKBACK_FRAMES;

    /* ---- 1. 丢线检测 ---- */
    if (digital_byte == 0) {
        /* 仅在"刚丢线"瞬间做方向回溯, 避免持续覆盖 */
        if (current_result != LINE_LOST) {
            /* 跳过最近 2 帧 (扫线污染期), 从第 3 帧往前找清晰方向证据 */
            for (uint8_t k = 3; k <= LOST_LOOKBACK_FRAMES; k++) {
                uint8_t idx  = (recent_index + LOST_LOOKBACK_FRAMES - k) % LOST_LOOKBACK_FRAMES;
                uint8_t past = recent_digital[idx];
                uint8_t pl = popcount8(past & LEFT_TURN_MASK);
                uint8_t pr = popcount8(past & RIGHT_TURN_MASK);
                if (pl >= 2 && pr == 0) { last_error_sign = -1.0f; break; }
                if (pr >= 2 && pl == 0) { last_error_sign = +1.0f; break; }
            }
        }
        current_result = LINE_LOST;
        /* 衰减分数, 防止 LOST 期间残留分数在恢复后立刻触发 */
        left_score  = score_dec(left_score,  1);
        right_score = score_dec(right_score, 1);
        cross_score = score_dec(cross_score, 1);
        return;
    }

    /* ---- 2. 单帧模式判定 ---- */
    uint8_t cross_now = is_cross_pattern(digital_byte);
    /* 注意: cross 优先, 一旦判为十字候选, 就不再当作弯道候选 */
    uint8_t left_now  = !cross_now && is_left_pattern(digital_byte, line_error);
    uint8_t right_now = !cross_now && is_right_pattern(digital_byte, line_error);

    /* ---- 3. 分数累积 / 反向惩罚 ---- */
    if (cross_now) {
        cross_score = score_inc(cross_score, 2);
        left_score  = score_dec(left_score,  1);
        right_score = score_dec(right_score, 1);
    } else if (left_now) {
        left_score  = score_inc(left_score,  1);
        right_score = score_dec(right_score, 2);   /* 反向强惩罚 */
        cross_score = score_dec(cross_score, 1);
        last_error_sign = -1.0f;                    /* 同步更新搜索方向记忆 */
    } else if (right_now) {
        right_score = score_inc(right_score, 1);
        left_score  = score_dec(left_score,  2);   /* 反向强惩罚 */
        cross_score = score_dec(cross_score, 1);
        last_error_sign = +1.0f;
    } else {
        /* 中性帧: 全部慢衰减 */
        left_score  = score_dec(left_score,  1);
        right_score = score_dec(right_score, 1);
        cross_score = score_dec(cross_score, 1);
    }

    /* ---- 4. 优先级仲裁 ---- */
    if (cross_score >= CROSS_CONFIRM_THRESHOLD) {
        current_result = LINE_CROSS;
    } else if (left_score >= TURN_CONFIRM_THRESHOLD) {
        current_result = LINE_LEFT_TURN;
    } else if (right_score >= TURN_CONFIRM_THRESHOLD) {
        current_result = LINE_RIGHT_TURN;
    } else {
        current_result = LINE_STRAIGHT;
    }
}

LineFollow_Result_e LineFollow_GetResult(void)
{
    return current_result;
}

void LineFollow_GetSpeed(float *left, float *right)
{
    float d_error = current_error - prev_error;
    prev_error = current_error;

    switch (current_result) {
    case LINE_STRAIGHT: {
        float turn = LINE_KP * current_error + LINE_KD * d_error;
        *left  = LINE_BASE_SPEED + turn;
        *right = LINE_BASE_SPEED - turn;
        break;
    }
    case LINE_LEFT_TURN:
        /* 强制左转: 左慢右快
         * (注: 在你的 sense_task 中此时会立即切到角度环, 这里值通常用不到) */
        *left  = LINE_TURN_SPEED * LINE_TURN_RATIO;
        *right = LINE_TURN_SPEED;
        break;
    case LINE_RIGHT_TURN:
        *left  = LINE_TURN_SPEED;
        *right = LINE_TURN_SPEED * LINE_TURN_RATIO;
        break;
    case LINE_CROSS:
        *left  = 0;
        *right = 0;
        break;
    case LINE_LOST:
        /* 用回溯锁定的方向原地旋转搜索 */
        *left  =  LINE_SEARCH_SPEED * last_error_sign;
        *right = -LINE_SEARCH_SPEED * last_error_sign;
        break;
    default:
        *left  = 0;
        *right = 0;
        break;
    }
}
