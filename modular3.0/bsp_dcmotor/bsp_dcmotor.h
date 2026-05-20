/*
 * bsp_dcmotor.h
 * 直流电机底层驱动接口（STM32F4 + HAL）
 * 说明：
 * 1) 本模块只做占空比与方向输出，不包含 PID 与速度环。
 * 2) 对外统一百分比占空比语义：[-100.0, 100.0]。
 */
#ifndef BSP_DCMOTOR_H
#define BSP_DCMOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 默认停机策略：主动刹车 + 100% 制动占空比 */
#define DCMOTOR_DEFAULT_STOP_USE_ACTIVE_BRAKE   (1U)
#define DCMOTOR_DEFAULT_BRAKE_DUTY_PERCENT      (100.0f)

/* 电机实例句柄：一台电机对应一个 DCMotor_t */
typedef struct
{
    /* PWM 输出资源 */
    TIM_HandleTypeDef *htim;
    uint32_t tim_channel;

    /* 主方向脚配置 */
    GPIO_TypeDef *dir_gpio_port;
    uint16_t dir_pin;
    GPIO_PinState dir_forward_state;
    GPIO_PinState dir_reverse_state;

    /* 可选辅助方向脚（全配或全不配） */
    GPIO_TypeDef *dir_aux_gpio_port;
    uint16_t dir_aux_pin;
    GPIO_PinState dir_aux_forward_state;
    GPIO_PinState dir_aux_reverse_state;

    /* 接线反向补偿：1 表示翻转正反逻辑 */
    uint8_t reverse_flag;

    /* 停机策略参数 */
    uint8_t stop_use_active_brake;
    float brake_duty_percent;
    GPIO_PinState brake_dir_state;
    GPIO_PinState brake_aux_dir_state;
} DCMotor_t;

/* 初始化、占空比下发、停机 */
void BSP_DCMotor_Init(DCMotor_t *motor);
void BSP_DCMotor_SetDuty(DCMotor_t *motor, float duty_cycle);
void BSP_DCMotor_Stop(DCMotor_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* BSP_DCMOTOR_H */
