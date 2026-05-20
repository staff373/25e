/*
 * bsp_encoder.h
 * 编码器底层采集接口（STM32F4 + HAL TIM Encoder）
 * 说明：
 * 1) 本模块只输出测量值（delta_count / accum_count / rad/s）。
 * 2) 建议在外部 1kHz 调度周期调用 BSP_Encoder_Update。
 */
#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 编码器实例句柄：支持多实例（左轮/右轮等） */
typedef struct
{
    /* 硬件与标定参数 */
    TIM_HandleTypeDef *htim;
    int32_t counts_per_rev;
    float update_hz;
    uint8_t reverse_flag;

    /* 运行态数据 */
    uint32_t last_cnt;
    int32_t last_delta_count;
    int32_t accum_count;
    float speed_rad_s;

    /* 状态标志 */
    uint8_t hw_ready;
    uint8_t started;
} BSP_Encoder_t;

/* 生命周期与数据访问接口 */
void BSP_Encoder_Init(BSP_Encoder_t *enc);
void BSP_Encoder_Update(BSP_Encoder_t *enc);
float BSP_Encoder_GetSpeedRadS(const BSP_Encoder_t *enc);
int32_t BSP_Encoder_GetDeltaCount(const BSP_Encoder_t *enc);
int32_t BSP_Encoder_GetAccumCount(const BSP_Encoder_t *enc);
void BSP_Encoder_Reset(BSP_Encoder_t *enc);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ENCODER_H */
