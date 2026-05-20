/*
 * bsp_encoder.c
 * 编码器采集实现：
 * - 基于 TIM 编码器模式读取 CNT
 * - 使用 ARR 回卷差分计算 delta_count
 * - 输出标准单位速度 rad/s
 */
#include "bsp_encoder.h"

#include <limits.h>

/* 2π 常量，用于计数到角速度的单位换算 */
#define BSP_ENCODER_TWO_PI  (6.28318530717958647692f)

/* 私有工具函数声明 */
static uint8_t Encoder_IsNaN(float value);
static uint8_t Encoder_IsBindingValid(const BSP_Encoder_t *enc);
static int32_t Encoder_ComputeWrappedDelta(uint32_t current_cnt,
                                           uint32_t last_cnt,
                                           uint32_t arr_value);
static int32_t Encoder_SaturatingAddInt32(int32_t lhs, int32_t rhs);

/* 初始化编码器实例：校验参数并启动 HAL 编码器接口 */
void BSP_Encoder_Init(BSP_Encoder_t *enc)
{
    if (enc == (BSP_Encoder_t *)0)
    {
        return;
    }

    enc->last_cnt = 0U;
    enc->last_delta_count = 0;
    enc->accum_count = 0;
    enc->speed_rad_s = 0.0f;
    enc->hw_ready = 0U;
    enc->started = 0U;

    if (Encoder_IsBindingValid(enc) == 0U)
    {
        return;
    }

    if (HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL) != HAL_OK)
    {
        return;
    }

    enc->last_cnt = __HAL_TIM_GET_COUNTER(enc->htim);
    enc->hw_ready = 1U;
    enc->started = 1U;
}

/* 周期更新：读取 CNT，完成回卷差分与速度换算 */
void BSP_Encoder_Update(BSP_Encoder_t *enc)
{
    uint32_t current_cnt;
    uint32_t arr_value;
    int32_t delta_count;
    float scale;

    if (enc == (BSP_Encoder_t *)0)
    {
        return;
    }

    if ((enc->hw_ready == 0U) || (enc->started == 0U))
    {
        return;
    }

    if (Encoder_IsBindingValid(enc) == 0U)
    {
        return;
    }

    current_cnt = __HAL_TIM_GET_COUNTER(enc->htim);
    arr_value = __HAL_TIM_GET_AUTORELOAD(enc->htim);
    delta_count = Encoder_ComputeWrappedDelta(current_cnt, enc->last_cnt, arr_value);
    enc->last_cnt = current_cnt;

    if (enc->reverse_flag != 0U)
    {
        delta_count = -delta_count;
    }

    enc->last_delta_count = delta_count;
    enc->accum_count = Encoder_SaturatingAddInt32(enc->accum_count, delta_count);

    scale = (BSP_ENCODER_TWO_PI / (float)enc->counts_per_rev) * enc->update_hz;
    enc->speed_rad_s = (float)delta_count * scale;
}

/* 获取当前周期换算后的角速度（rad/s） */
float BSP_Encoder_GetSpeedRadS(const BSP_Encoder_t *enc)
{
    if (enc == (const BSP_Encoder_t *)0)
    {
        return 0.0f;
    }

    return enc->speed_rad_s;
}

/* 获取上一更新周期的增量计数 */
int32_t BSP_Encoder_GetDeltaCount(const BSP_Encoder_t *enc)
{
    if (enc == (const BSP_Encoder_t *)0)
    {
        return 0;
    }

    return enc->last_delta_count;
}

/* 获取累计计数（带饱和保护） */
int32_t BSP_Encoder_GetAccumCount(const BSP_Encoder_t *enc)
{
    if (enc == (const BSP_Encoder_t *)0)
    {
        return 0;
    }

    return enc->accum_count;
}

/* 重置运行态累计值，不改硬件配置 */
void BSP_Encoder_Reset(BSP_Encoder_t *enc)
{
    if (enc == (BSP_Encoder_t *)0)
    {
        return;
    }

    enc->last_delta_count = 0;
    enc->accum_count = 0;
    enc->speed_rad_s = 0.0f;

    if (enc->htim != (TIM_HandleTypeDef *)0)
    {
        enc->last_cnt = __HAL_TIM_GET_COUNTER(enc->htim);
    }
    else
    {
        enc->last_cnt = 0U;
    }
}

/* 简单 NaN 判定，避免依赖额外数学库 */
static uint8_t Encoder_IsNaN(float value)
{
    uint8_t is_nan = 0U;

    if (value != value)
    {
        is_nan = 1U;
    }

    return is_nan;
}

/* 检查实例参数是否可用于运行 */
static uint8_t Encoder_IsBindingValid(const BSP_Encoder_t *enc)
{
    uint8_t valid = 1U;

    if (enc == (const BSP_Encoder_t *)0)
    {
        valid = 0U;
    }
    else if (enc->htim == (TIM_HandleTypeDef *)0)
    {
        valid = 0U;
    }
    else if (enc->counts_per_rev <= 0)
    {
        valid = 0U;
    }
    else if (Encoder_IsNaN(enc->update_hz) != 0U)
    {
        valid = 0U;
    }
    else if (enc->update_hz <= 0.0f)
    {
        valid = 0U;
    }
    else
    {
        /* 绑定参数有效 */
    }

    return valid;
}

/* 基于 ARR 周期做回卷差分，支持跨 0/ARR 边界 */
static int32_t Encoder_ComputeWrappedDelta(uint32_t current_cnt,
                                           uint32_t last_cnt,
                                           uint32_t arr_value)
{
    uint64_t period_counts;
    int64_t raw_delta;
    int64_t half_period;

    period_counts = (uint64_t)arr_value + 1ULL;
    raw_delta = (int64_t)current_cnt - (int64_t)last_cnt;

    if (period_counts > 1ULL)
    {
        half_period = (int64_t)(period_counts / 2ULL);

        if (raw_delta > half_period)
        {
            raw_delta -= (int64_t)period_counts;
        }
        else if (raw_delta < (-half_period))
        {
            raw_delta += (int64_t)period_counts;
        }
        else
        {
            /* 未发生回卷 */
        }
    }

    if (raw_delta > (int64_t)INT32_MAX)
    {
        raw_delta = (int64_t)INT32_MAX;
    }
    else if (raw_delta < (int64_t)INT32_MIN)
    {
        raw_delta = (int64_t)INT32_MIN;
    }
    else
    {
        /* 差值位于范围内 */
    }

    return (int32_t)raw_delta;
}

/* int32 饱和加法，避免累计值溢出翻转 */
static int32_t Encoder_SaturatingAddInt32(int32_t lhs, int32_t rhs)
{
    int64_t sum;

    sum = (int64_t)lhs + (int64_t)rhs;
    if (sum > (int64_t)INT32_MAX)
    {
        sum = (int64_t)INT32_MAX;
    }
    else if (sum < (int64_t)INT32_MIN)
    {
        sum = (int64_t)INT32_MIN;
    }
    else
    {
        /* 求和位于范围内 */
    }

    return (int32_t)sum;
}
