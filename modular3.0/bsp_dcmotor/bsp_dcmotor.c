#include "bsp_dcmotor.h"

#define DCMOTOR_DUTY_MIN_PERCENT   (-100.0f)
#define DCMOTOR_DUTY_MAX_PERCENT   (100.0f)

static uint8_t DCMotor_IsNaN(float value);
static float DCMotor_ClampFloat(float value, float min_value, float max_value);
static uint8_t DCMotor_IsChannelValid(uint32_t channel);
static uint8_t DCMotor_IsHwBindingValid(const DCMotor_t *motor);
static uint8_t DCMotor_IsAuxConfigured(const DCMotor_t *motor);
static void DCMotor_ApplyDirectionBySign(DCMotor_t *motor, uint8_t is_forward);
static uint32_t DCMotor_DutyPercentToCompare(const DCMotor_t *motor, float duty_percent);

void BSP_DCMotor_Init(DCMotor_t *motor)
{
    if (DCMotor_IsHwBindingValid(motor) == 0U)
    {
        return;
    }

    if (DCMotor_IsNaN(motor->brake_duty_percent) != 0U)
    {
        motor->brake_duty_percent = DCMOTOR_DEFAULT_BRAKE_DUTY_PERCENT;
    }

    motor->brake_duty_percent = DCMotor_ClampFloat(motor->brake_duty_percent, 0.0f, 100.0f);

    if (motor->stop_use_active_brake > 1U)
    {
        motor->stop_use_active_brake = DCMOTOR_DEFAULT_STOP_USE_ACTIVE_BRAKE;
    }

    __HAL_TIM_SET_COMPARE(motor->htim, motor->tim_channel, 0U);

    HAL_GPIO_WritePin(motor->dir_gpio_port, motor->dir_pin, motor->dir_forward_state);
    if (DCMotor_IsAuxConfigured(motor) != 0U)
    {
        HAL_GPIO_WritePin(motor->dir_aux_gpio_port,
                          motor->dir_aux_pin,
                          motor->dir_aux_forward_state);
    }

    (void)HAL_TIM_PWM_Start(motor->htim, motor->tim_channel);
}

void BSP_DCMotor_SetDuty(DCMotor_t *motor, float duty_cycle)
{
    float clamped_duty;
    uint8_t is_forward;
    uint32_t ccr_value;

    if (DCMotor_IsHwBindingValid(motor) == 0U)
    {
        return;
    }

    if (DCMotor_IsNaN(duty_cycle) != 0U)
    {
        duty_cycle = 0.0f;
    }

    clamped_duty = DCMotor_ClampFloat(duty_cycle, DCMOTOR_DUTY_MIN_PERCENT, DCMOTOR_DUTY_MAX_PERCENT);

    if (clamped_duty > 0.0f)
    {
        is_forward = 1U;
    }
    else if (clamped_duty < 0.0f)
    {
        is_forward = 0U;
    }
    else
    {
        __HAL_TIM_SET_COMPARE(motor->htim, motor->tim_channel, 0U);
        return;
    }

    DCMotor_ApplyDirectionBySign(motor, is_forward);

    ccr_value = DCMotor_DutyPercentToCompare(motor, clamped_duty);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->tim_channel, ccr_value);
}

void BSP_DCMotor_Stop(DCMotor_t *motor)
{
    uint32_t brake_ccr;

    if (DCMotor_IsHwBindingValid(motor) == 0U)
    {
        return;
    }

    if (motor->stop_use_active_brake != 0U)
    {
        if (DCMotor_IsNaN(motor->brake_duty_percent) != 0U)
        {
            motor->brake_duty_percent = DCMOTOR_DEFAULT_BRAKE_DUTY_PERCENT;
        }

        motor->brake_duty_percent = DCMotor_ClampFloat(motor->brake_duty_percent, 0.0f, 100.0f);

        HAL_GPIO_WritePin(motor->dir_gpio_port, motor->dir_pin, motor->brake_dir_state);
        if (DCMotor_IsAuxConfigured(motor) != 0U)
        {
            HAL_GPIO_WritePin(motor->dir_aux_gpio_port,
                              motor->dir_aux_pin,
                              motor->brake_aux_dir_state);
        }

        brake_ccr = DCMotor_DutyPercentToCompare(motor, motor->brake_duty_percent);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->tim_channel, brake_ccr);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(motor->htim, motor->tim_channel, 0U);
    }
}

static uint8_t DCMotor_IsNaN(float value)
{
    uint8_t is_nan = 0U;

    if (value != value)
    {
        is_nan = 1U;
    }

    return is_nan;
}

static float DCMotor_ClampFloat(float value, float min_value, float max_value)
{
    float clamped = value;

    if (clamped < min_value)
    {
        clamped = min_value;
    }

    if (clamped > max_value)
    {
        clamped = max_value;
    }

    return clamped;
}

static uint8_t DCMotor_IsChannelValid(uint32_t channel)
{
    uint8_t valid = 0U;

    if ((channel == TIM_CHANNEL_1) ||
        (channel == TIM_CHANNEL_2) ||
        (channel == TIM_CHANNEL_3) ||
        (channel == TIM_CHANNEL_4))
    {
        valid = 1U;
    }

    return valid;
}

static uint8_t DCMotor_IsHwBindingValid(const DCMotor_t *motor)
{
    uint8_t valid = 1U;

    if (motor == (const DCMotor_t *)0)
    {
        valid = 0U;
    }
    else if (motor->htim == (TIM_HandleTypeDef *)0)
    {
        valid = 0U;
    }
    else if (DCMotor_IsChannelValid(motor->tim_channel) == 0U)
    {
        valid = 0U;
    }
    else if (motor->dir_gpio_port == (GPIO_TypeDef *)0)
    {
        valid = 0U;
    }
    else if (motor->dir_pin == 0U)
    {
        valid = 0U;
    }
    else if (((motor->dir_aux_gpio_port == (GPIO_TypeDef *)0) && (motor->dir_aux_pin != 0U)) ||
             ((motor->dir_aux_gpio_port != (GPIO_TypeDef *)0) && (motor->dir_aux_pin == 0U)))
    {
        /* 辅助方向脚需要“全配”或“全不配” */
        valid = 0U;
    }
    else
    {
        /* 绑定参数有效 */
    }

    return valid;
}

static uint8_t DCMotor_IsAuxConfigured(const DCMotor_t *motor)
{
    uint8_t configured = 0U;

    if (motor == (const DCMotor_t *)0)
    {
        return 0U;
    }

    if ((motor->dir_aux_gpio_port != (GPIO_TypeDef *)0) &&
        (motor->dir_aux_pin != 0U))
    {
        configured = 1U;
    }

    return configured;
}

static void DCMotor_ApplyDirectionBySign(DCMotor_t *motor, uint8_t is_forward)
{
    GPIO_PinState dir_state;
    GPIO_PinState aux_dir_state;

    if (motor->reverse_flag != 0U)
    {
        is_forward = (is_forward == 0U) ? 1U : 0U;
    }

    if (is_forward != 0U)
    {
        dir_state = motor->dir_forward_state;
        aux_dir_state = motor->dir_aux_forward_state;
    }
    else
    {
        dir_state = motor->dir_reverse_state;
        aux_dir_state = motor->dir_aux_reverse_state;
    }

    HAL_GPIO_WritePin(motor->dir_gpio_port, motor->dir_pin, dir_state);
    if (DCMotor_IsAuxConfigured(motor) != 0U)
    {
        HAL_GPIO_WritePin(motor->dir_aux_gpio_port, motor->dir_aux_pin, aux_dir_state);
    }
}

static uint32_t DCMotor_DutyPercentToCompare(const DCMotor_t *motor, float duty_percent)
{
    float abs_duty;
    uint32_t arr_value;
    uint32_t ccr_value;

    if (motor == (const DCMotor_t *)0)
    {
        return 0U;
    }

    abs_duty = duty_percent;
    if (DCMotor_IsNaN(abs_duty) != 0U)
    {
        abs_duty = 0.0f;
    }

    if (abs_duty < 0.0f)
    {
        abs_duty = -abs_duty;
    }

    abs_duty = DCMotor_ClampFloat(abs_duty, 0.0f, 100.0f);

    arr_value = __HAL_TIM_GET_AUTORELOAD(motor->htim);
    ccr_value = (uint32_t)((abs_duty * (float)arr_value / 100.0f) + 0.5f);

    if (ccr_value > arr_value)
    {
        ccr_value = arr_value;
    }

    return ccr_value;
}
