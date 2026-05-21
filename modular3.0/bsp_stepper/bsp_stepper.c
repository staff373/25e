#include "bsp_stepper.h"

#include "main.h"
#include "tim.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t tim_channel;
    uint32_t tim_clk_hz;

    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_PinState dir_positive_state;
    GPIO_PinState dir_negative_state;

    GPIO_TypeDef *en_port;
    uint16_t en_pin;
    GPIO_PinState en_active_state;
    GPIO_PinState en_inactive_state;

    float min_speed_sps;
    float max_speed_limit_sps;
    float default_accel_sps2;

    volatile BSP_StepperState_t state;
    volatile uint8_t busy;
    volatile uint8_t hold_enabled;
    volatile uint8_t stop_requested;
    volatile uint8_t hw_ready;
    volatile int32_t position_steps;
    volatile int32_t direction_sign;
    volatile uint32_t total_steps;
    volatile uint32_t remaining_steps;

    float current_speed_sps;
    float max_speed_sps;
    float accel_sps2;
} Stepper_AxisRuntime_t;

static Stepper_AxisRuntime_t g_stepper_axis[STEPPER_AXIS_COUNT];

static uint8_t Stepper_IsAxisValid(uint8_t axis);
static uint32_t Stepper_AbsSteps(int32_t steps);
static float Stepper_AbsFloat(float value);
static float Stepper_ClampFloat(float value, float min_value, float max_value);
static void Stepper_LoadAxisConfig(uint8_t axis);
static uint8_t Stepper_IsHwBindingValid(const Stepper_AxisRuntime_t *axis_obj);
static void Stepper_ApplyEnable(uint8_t axis, uint8_t enable);
static void Stepper_ApplyDirection(uint8_t axis);
static uint8_t Stepper_ConfigPulseTimer(uint8_t axis, float pulse_hz, uint8_t force_update_event);
static uint8_t Stepper_StartPulse(uint8_t axis);
static void Stepper_StopPulse(uint8_t axis, BSP_StepperState_t final_state);
static void Stepper_OnPeriodElapsed(uint8_t axis);
static void Stepper_UpdateSpeedForNextStep(uint8_t axis);

void BSP_Stepper_Init(void)
{
    uint8_t axis;

    for (axis = 0U; axis < STEPPER_AXIS_COUNT; axis++)
    {
        Stepper_LoadAxisConfig(axis);
        g_stepper_axis[axis].hw_ready = Stepper_IsHwBindingValid(&g_stepper_axis[axis]);
        g_stepper_axis[axis].state = (g_stepper_axis[axis].hw_ready != 0U) ? STEPPER_STATE_IDLE : STEPPER_STATE_DISABLED;
        g_stepper_axis[axis].busy = 0U;
        g_stepper_axis[axis].hold_enabled = 0U;
        g_stepper_axis[axis].stop_requested = 0U;
        g_stepper_axis[axis].position_steps = 0;
        g_stepper_axis[axis].direction_sign = 1;
        g_stepper_axis[axis].total_steps = 0U;
        g_stepper_axis[axis].remaining_steps = 0U;
        g_stepper_axis[axis].current_speed_sps = 0.0f;
        g_stepper_axis[axis].max_speed_sps = g_stepper_axis[axis].min_speed_sps;
        g_stepper_axis[axis].accel_sps2 = g_stepper_axis[axis].default_accel_sps2;

        if (g_stepper_axis[axis].hw_ready != 0U)
        {
            Stepper_ApplyDirection(axis);
            Stepper_ApplyEnable(axis, 0U);
        }
    }
}

uint8_t BSP_Stepper_MoveSteps(uint8_t axis, int32_t steps, float max_speed_sps, float accel_sps2)
{
    uint32_t step_count;
    float speed;
    float accel;

    if (Stepper_IsAxisValid(axis) == 0U)
    {
        return 0U;
    }

    if ((g_stepper_axis[axis].hw_ready == 0U) || (g_stepper_axis[axis].busy != 0U))
    {
        return 0U;
    }

    step_count = Stepper_AbsSteps(steps);
    if (step_count == 0U)
    {
        g_stepper_axis[axis].state = STEPPER_STATE_DONE;
        return 1U;
    }

    speed = max_speed_sps;
    if (speed <= STEPPER_SPEED_EPSILON_SPS)
    {
        speed = g_stepper_axis[axis].max_speed_limit_sps;
    }
    speed = Stepper_ClampFloat(speed,
                               g_stepper_axis[axis].min_speed_sps,
                               g_stepper_axis[axis].max_speed_limit_sps);

    accel = accel_sps2;
    if (accel <= STEPPER_SPEED_EPSILON_SPS)
    {
        accel = g_stepper_axis[axis].default_accel_sps2;
    }

    g_stepper_axis[axis].direction_sign = (steps >= 0) ? 1 : -1;
    g_stepper_axis[axis].total_steps = step_count;
    g_stepper_axis[axis].remaining_steps = step_count;
    g_stepper_axis[axis].stop_requested = 0U;
    g_stepper_axis[axis].max_speed_sps = speed;
    g_stepper_axis[axis].accel_sps2 = accel;
    g_stepper_axis[axis].current_speed_sps = g_stepper_axis[axis].min_speed_sps;

    Stepper_ApplyDirection(axis);

    if (Stepper_StartPulse(axis) == 0U)
    {
        g_stepper_axis[axis].remaining_steps = 0U;
        g_stepper_axis[axis].total_steps = 0U;
        g_stepper_axis[axis].current_speed_sps = 0.0f;
        g_stepper_axis[axis].state = STEPPER_STATE_ERROR;
        return 0U;
    }

    return 1U;
}

void BSP_Stepper_Stop(uint8_t axis)
{
    if (Stepper_IsAxisValid(axis) == 0U)
    {
        return;
    }

    if ((g_stepper_axis[axis].hw_ready == 0U) || (g_stepper_axis[axis].busy == 0U))
    {
        return;
    }

    g_stepper_axis[axis].stop_requested = 1U;
    g_stepper_axis[axis].state = STEPPER_STATE_STOPPING;
}

void BSP_Stepper_EmergencyStop(uint8_t axis)
{
    if (Stepper_IsAxisValid(axis) == 0U)
    {
        return;
    }

    if (g_stepper_axis[axis].hw_ready == 0U)
    {
        return;
    }

    g_stepper_axis[axis].remaining_steps = 0U;
    g_stepper_axis[axis].total_steps = 0U;
    g_stepper_axis[axis].current_speed_sps = 0.0f;
    g_stepper_axis[axis].stop_requested = 0U;
    Stepper_StopPulse(axis, STEPPER_STATE_IDLE);
}

void BSP_Stepper_SetHoldEnabled(uint8_t axis, uint8_t enabled)
{
    if (Stepper_IsAxisValid(axis) == 0U)
    {
        return;
    }

    if (g_stepper_axis[axis].hw_ready == 0U)
    {
        return;
    }

    g_stepper_axis[axis].hold_enabled = (enabled != 0U) ? 1U : 0U;
    if (g_stepper_axis[axis].busy == 0U)
    {
        Stepper_ApplyEnable(axis, g_stepper_axis[axis].hold_enabled);
    }
}

uint8_t BSP_Stepper_GetHoldEnabled(uint8_t axis)
{
    if (Stepper_IsAxisValid(axis) == 0U)
    {
        return 0U;
    }

    return g_stepper_axis[axis].hold_enabled;
}

uint8_t BSP_Stepper_IsBusy(uint8_t axis)
{
    if (Stepper_IsAxisValid(axis) == 0U)
    {
        return 0U;
    }

    return g_stepper_axis[axis].busy;
}

uint8_t BSP_Stepper_IsReady(uint8_t axis)
{
    if (Stepper_IsAxisValid(axis) == 0U)
    {
        return 0U;
    }

    return g_stepper_axis[axis].hw_ready;
}

int32_t BSP_Stepper_GetPosition(uint8_t axis)
{
    if (Stepper_IsAxisValid(axis) == 0U)
    {
        return 0;
    }

    return g_stepper_axis[axis].position_steps;
}

void BSP_Stepper_SetPosition(uint8_t axis, int32_t position_steps)
{
    if (Stepper_IsAxisValid(axis) == 0U)
    {
        return;
    }

    if (g_stepper_axis[axis].busy != 0U)
    {
        return;
    }

    g_stepper_axis[axis].position_steps = position_steps;
}

int32_t BSP_Stepper_GetRemaining(uint8_t axis)
{
    if (Stepper_IsAxisValid(axis) == 0U)
    {
        return 0;
    }

    if (g_stepper_axis[axis].remaining_steps > 2147483647UL)
    {
        return 2147483647L;
    }

    return (int32_t)g_stepper_axis[axis].remaining_steps;
}

BSP_StepperState_t BSP_Stepper_GetState(uint8_t axis)
{
    if (Stepper_IsAxisValid(axis) == 0U)
    {
        return STEPPER_STATE_DISABLED;
    }

    return g_stepper_axis[axis].state;
}

void BSP_Stepper_GetStatus(uint8_t axis, BSP_StepperStatus_t *status)
{
    if ((Stepper_IsAxisValid(axis) == 0U) || (status == (BSP_StepperStatus_t *)0))
    {
        return;
    }

    status->hw_ready = g_stepper_axis[axis].hw_ready;
    status->busy = g_stepper_axis[axis].busy;
    status->hold_enabled = g_stepper_axis[axis].hold_enabled;
    status->state = g_stepper_axis[axis].state;
    status->position_steps = g_stepper_axis[axis].position_steps;
    status->direction_sign = g_stepper_axis[axis].direction_sign;
    status->total_steps = g_stepper_axis[axis].total_steps;
    status->remaining_steps = g_stepper_axis[axis].remaining_steps;
    status->current_speed_sps = g_stepper_axis[axis].current_speed_sps;
    status->max_speed_sps = g_stepper_axis[axis].max_speed_sps;
    status->accel_sps2 = g_stepper_axis[axis].accel_sps2;
}

const char *BSP_Stepper_GetStateName(BSP_StepperState_t state)
{
    switch (state)
    {
    case STEPPER_STATE_DISABLED:
        return "DISABLED";
    case STEPPER_STATE_IDLE:
        return "IDLE";
    case STEPPER_STATE_MOVING:
        return "MOVING";
    case STEPPER_STATE_STOPPING:
        return "STOPPING";
    case STEPPER_STATE_DONE:
        return "DONE";
    case STEPPER_STATE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

void BSP_Stepper_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t axis;

    if (htim == (TIM_HandleTypeDef *)0)
    {
        return;
    }

    for (axis = 0U; axis < STEPPER_AXIS_COUNT; axis++)
    {
        if ((g_stepper_axis[axis].hw_ready != 0U) &&
            (g_stepper_axis[axis].busy != 0U) &&
            (g_stepper_axis[axis].htim == htim))
        {
            Stepper_OnPeriodElapsed(axis);
            break;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    BSP_Stepper_TIM_PeriodElapsedCallback(htim);
}

static uint8_t Stepper_IsAxisValid(uint8_t axis)
{
    return (uint8_t)(axis < STEPPER_AXIS_COUNT);
}

static uint32_t Stepper_AbsSteps(int32_t steps)
{
    uint32_t value;

    if (steps >= 0)
    {
        value = (uint32_t)steps;
    }
    else
    {
        value = (uint32_t)(-(steps + 1)) + 1U;
    }

    return value;
}

static float Stepper_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float Stepper_ClampFloat(float value, float min_value, float max_value)
{
    float result = value;

    if (result < min_value)
    {
        result = min_value;
    }
    if (result > max_value)
    {
        result = max_value;
    }

    return result;
}

static void Stepper_LoadAxisConfig(uint8_t axis)
{
    Stepper_AxisRuntime_t *obj = &g_stepper_axis[axis];

    if (axis == STEPPER_AXIS_X)
    {
        obj->htim = &htim9;
        obj->tim_channel = TIM_CHANNEL_2;
        obj->tim_clk_hz = 168000000U;
        obj->dir_port = X_DIR_GPIO_Port;
        obj->dir_pin = X_DIR_Pin;
        obj->en_port = X_EN_GPIO_Port;
        obj->en_pin = X_EN_Pin;
        obj->min_speed_sps = STEPPER_X_MIN_SPEED_SPS;
        obj->max_speed_limit_sps = STEPPER_X_MAX_SPEED_SPS;
        obj->default_accel_sps2 = STEPPER_X_ACCEL_SPS2;
    }
    else
    {
        obj->htim = &htim12;
        obj->tim_channel = TIM_CHANNEL_1;
        obj->tim_clk_hz = 84000000U;
        obj->dir_port = Y_DIR_GPIO_Port;
        obj->dir_pin = Y_DIR_Pin;
        obj->en_port = Y_EN_GPIO_Port;
        obj->en_pin = Y_EN_Pin;
        obj->min_speed_sps = STEPPER_Y_MIN_SPEED_SPS;
        obj->max_speed_limit_sps = STEPPER_Y_MAX_SPEED_SPS;
        obj->default_accel_sps2 = STEPPER_Y_ACCEL_SPS2;
    }

    obj->dir_positive_state = GPIO_PIN_RESET;
    obj->dir_negative_state = GPIO_PIN_SET;
    obj->en_active_state = GPIO_PIN_SET;
    obj->en_inactive_state = GPIO_PIN_RESET;
}

static uint8_t Stepper_IsHwBindingValid(const Stepper_AxisRuntime_t *axis_obj)
{
    if (axis_obj == (const Stepper_AxisRuntime_t *)0)
    {
        return 0U;
    }

    if ((axis_obj->htim == (TIM_HandleTypeDef *)0) ||
        (axis_obj->tim_clk_hz == 0U) ||
        (axis_obj->dir_port == (GPIO_TypeDef *)0) ||
        (axis_obj->dir_pin == 0U) ||
        (axis_obj->en_port == (GPIO_TypeDef *)0) ||
        (axis_obj->en_pin == 0U) ||
        (axis_obj->min_speed_sps <= 0.0f) ||
        (axis_obj->max_speed_limit_sps < axis_obj->min_speed_sps) ||
        (axis_obj->default_accel_sps2 <= 0.0f))
    {
        return 0U;
    }

    if ((axis_obj->tim_channel != TIM_CHANNEL_1) &&
        (axis_obj->tim_channel != TIM_CHANNEL_2) &&
        (axis_obj->tim_channel != TIM_CHANNEL_3) &&
        (axis_obj->tim_channel != TIM_CHANNEL_4))
    {
        return 0U;
    }

    return 1U;
}

static void Stepper_ApplyEnable(uint8_t axis, uint8_t enable)
{
    GPIO_PinState state;

    state = (enable != 0U) ? g_stepper_axis[axis].en_active_state : g_stepper_axis[axis].en_inactive_state;
    HAL_GPIO_WritePin(g_stepper_axis[axis].en_port, g_stepper_axis[axis].en_pin, state);
}

static void Stepper_ApplyDirection(uint8_t axis)
{
    GPIO_PinState state;

    state = (g_stepper_axis[axis].direction_sign >= 0) ?
            g_stepper_axis[axis].dir_positive_state :
            g_stepper_axis[axis].dir_negative_state;
    HAL_GPIO_WritePin(g_stepper_axis[axis].dir_port, g_stepper_axis[axis].dir_pin, state);
}

static uint8_t Stepper_ConfigPulseTimer(uint8_t axis, float pulse_hz, uint8_t force_update_event)
{
    uint32_t total_ticks;
    uint32_t psc;
    uint32_t arr;
    uint32_t ccr;

    if (pulse_hz <= STEPPER_SPEED_EPSILON_SPS)
    {
        return 0U;
    }

    total_ticks = (uint32_t)((((float)g_stepper_axis[axis].tim_clk_hz) / pulse_hz) + 0.5f);
    if (total_ticks < 2U)
    {
        total_ticks = 2U;
    }

    psc = (total_ticks - 1U) / (STEPPER_TIMER_ARR_MAX + 1U);
    if (psc > STEPPER_TIMER_PSC_MAX)
    {
        psc = STEPPER_TIMER_PSC_MAX;
    }

    arr = total_ticks / (psc + 1U);
    if (arr > 0U)
    {
        arr -= 1U;
    }
    if (arr > STEPPER_TIMER_ARR_MAX)
    {
        arr = STEPPER_TIMER_ARR_MAX;
    }
    if (arr < 1U)
    {
        arr = 1U;
    }

    ccr = (arr + 1U) / 2U;
    if (ccr < 1U)
    {
        ccr = 1U;
    }

    __HAL_TIM_SET_PRESCALER(g_stepper_axis[axis].htim, psc);
    __HAL_TIM_SET_AUTORELOAD(g_stepper_axis[axis].htim, arr);
    __HAL_TIM_SET_COMPARE(g_stepper_axis[axis].htim, g_stepper_axis[axis].tim_channel, ccr);

    if (force_update_event != 0U)
    {
        __HAL_TIM_DISABLE_IT(g_stepper_axis[axis].htim, TIM_IT_UPDATE);
        (void)HAL_TIM_GenerateEvent(g_stepper_axis[axis].htim, TIM_EVENTSOURCE_UPDATE);
        __HAL_TIM_CLEAR_FLAG(g_stepper_axis[axis].htim, TIM_FLAG_UPDATE);
    }

    return 1U;
}

static uint8_t Stepper_StartPulse(uint8_t axis)
{
    if (Stepper_ConfigPulseTimer(axis, g_stepper_axis[axis].current_speed_sps, 1U) == 0U)
    {
        return 0U;
    }

    __HAL_TIM_SET_COUNTER(g_stepper_axis[axis].htim, 0U);
    __HAL_TIM_CLEAR_FLAG(g_stepper_axis[axis].htim, TIM_FLAG_UPDATE);
    Stepper_ApplyEnable(axis, 1U);

    if (HAL_TIM_PWM_Start(g_stepper_axis[axis].htim, g_stepper_axis[axis].tim_channel) != HAL_OK)
    {
        Stepper_ApplyEnable(axis, g_stepper_axis[axis].hold_enabled);
        return 0U;
    }

    __HAL_TIM_CLEAR_FLAG(g_stepper_axis[axis].htim, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(g_stepper_axis[axis].htim, TIM_IT_UPDATE);
    g_stepper_axis[axis].busy = 1U;
    g_stepper_axis[axis].state = STEPPER_STATE_MOVING;
    return 1U;
}

static void Stepper_StopPulse(uint8_t axis, BSP_StepperState_t final_state)
{
    __HAL_TIM_DISABLE_IT(g_stepper_axis[axis].htim, TIM_IT_UPDATE);
    (void)HAL_TIM_PWM_Stop(g_stepper_axis[axis].htim, g_stepper_axis[axis].tim_channel);
    __HAL_TIM_CLEAR_FLAG(g_stepper_axis[axis].htim, TIM_FLAG_UPDATE);

    g_stepper_axis[axis].busy = 0U;
    g_stepper_axis[axis].stop_requested = 0U;
    g_stepper_axis[axis].current_speed_sps = 0.0f;
    g_stepper_axis[axis].state = final_state;
    Stepper_ApplyEnable(axis, g_stepper_axis[axis].hold_enabled);
}

static void Stepper_OnPeriodElapsed(uint8_t axis)
{
    if ((g_stepper_axis[axis].state != STEPPER_STATE_MOVING) &&
        (g_stepper_axis[axis].state != STEPPER_STATE_STOPPING))
    {
        return;
    }

    if (g_stepper_axis[axis].remaining_steps > 0U)
    {
        g_stepper_axis[axis].remaining_steps--;
        g_stepper_axis[axis].position_steps += g_stepper_axis[axis].direction_sign;
    }

    if (g_stepper_axis[axis].remaining_steps == 0U)
    {
        Stepper_StopPulse(axis, STEPPER_STATE_DONE);
        return;
    }

    if (g_stepper_axis[axis].stop_requested != 0U)
    {
        if (g_stepper_axis[axis].current_speed_sps <= (g_stepper_axis[axis].min_speed_sps + STEPPER_SPEED_EPSILON_SPS))
        {
            g_stepper_axis[axis].remaining_steps = 0U;
            Stepper_StopPulse(axis, STEPPER_STATE_IDLE);
            return;
        }

        g_stepper_axis[axis].state = STEPPER_STATE_STOPPING;
    }

    Stepper_UpdateSpeedForNextStep(axis);
    if (Stepper_ConfigPulseTimer(axis, g_stepper_axis[axis].current_speed_sps, 0U) == 0U)
    {
        g_stepper_axis[axis].remaining_steps = 0U;
        Stepper_StopPulse(axis, STEPPER_STATE_ERROR);
    }
}

static void Stepper_UpdateSpeedForNextStep(uint8_t axis)
{
    float current;
    float delta_v;
    float stop_steps;

    current = g_stepper_axis[axis].current_speed_sps;
    if (current < g_stepper_axis[axis].min_speed_sps)
    {
        current = g_stepper_axis[axis].min_speed_sps;
    }

    delta_v = g_stepper_axis[axis].accel_sps2 / current;
    if (delta_v < STEPPER_SPEED_EPSILON_SPS)
    {
        delta_v = STEPPER_SPEED_EPSILON_SPS;
    }

    stop_steps = (current * current) / (2.0f * g_stepper_axis[axis].accel_sps2);

    if ((g_stepper_axis[axis].stop_requested != 0U) ||
        (((float)g_stepper_axis[axis].remaining_steps) <= (stop_steps + 1.0f)))
    {
        current -= delta_v;
        if (current < g_stepper_axis[axis].min_speed_sps)
        {
            current = g_stepper_axis[axis].min_speed_sps;
        }
    }
    else
    {
        current += delta_v;
        if (current > g_stepper_axis[axis].max_speed_sps)
        {
            current = g_stepper_axis[axis].max_speed_sps;
        }
    }

    if (Stepper_AbsFloat(current) <= STEPPER_SPEED_EPSILON_SPS)
    {
        current = g_stepper_axis[axis].min_speed_sps;
    }

    g_stepper_axis[axis].current_speed_sps = current;
}
