#include "app_gimbal.h"

#include "bsp_stepper.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>

#define GIMBAL_DEFAULT_SPEED_SPS        (400.0f)
#define GIMBAL_MAX_COMMAND_SPEED_SPS    (4000.0f)
#define GIMBAL_DEFAULT_ACCEL_SPS2       (3000.0f)
#define GIMBAL_MAX_RELATIVE_STEPS       (2000L)

typedef enum
{
    GIMBAL_ERR_NONE = 0,
    GIMBAL_ERR_BUSY,
    GIMBAL_ERR_STEPPER,
    GIMBAL_ERR_CAL,
    GIMBAL_ERR_RANGE
} Gimbal_Error_t;

static Gimbal_State_t g_gimbal_state = GIMBAL_STATE_IDLE;
static Gimbal_Error_t g_gimbal_last_error = GIMBAL_ERR_NONE;
static uint32_t g_gimbal_state_enter_ms = 0U;
static uint8_t g_gimbal_calibrated = 0U;
static uint8_t g_gimbal_hold_enabled = 0U;
static float g_gimbal_cal_a = 0.0f;
static float g_gimbal_cal_b = 0.0f;
static float g_gimbal_cal_c = 0.0f;
static float g_gimbal_cal_d = 0.0f;

static void Gimbal_EnterState(Gimbal_State_t next);
static uint32_t Gimbal_StateElapsedMs(void);
static float Gimbal_AbsFloat(float value);
static int32_t Gimbal_AbsLong(int32_t value);
static int32_t Gimbal_RoundToSteps(float value);
static int32_t Gimbal_ClampSteps(int32_t value);
static float Gimbal_ClampSpeed(float speed_sps);
static const char *Gimbal_GetErrorName(Gimbal_Error_t error);

void Gimbal_Init(void)
{
    BSP_Stepper_Init();
    Gimbal_SetHoldEnabled(0U);
    g_gimbal_calibrated = 0U;
    g_gimbal_last_error = GIMBAL_ERR_NONE;
    Gimbal_EnterState(GIMBAL_STATE_IDLE);
}

void Gimbal_Poll(void)
{
    if ((g_gimbal_state == GIMBAL_STATE_MOVING) ||
        (g_gimbal_state == GIMBAL_STATE_STOPPING))
    {
        if (Gimbal_IsBusy() == 0U)
        {
            Gimbal_EnterState(GIMBAL_STATE_IDLE);
        }
    }
}

uint8_t Gimbal_MoveRelativeSteps(int32_t x_steps, int32_t y_steps, float speed_sps)
{
    int32_t x_abs;
    int32_t y_abs;
    int32_t max_abs;
    float base_speed;
    float x_speed;
    float y_speed;
    uint8_t x_started = 0U;
    uint8_t y_started = 0U;

    if (Gimbal_IsBusy() != 0U)
    {
        g_gimbal_last_error = GIMBAL_ERR_BUSY;
        return 0U;
    }

    x_steps = Gimbal_ClampSteps(x_steps);
    y_steps = Gimbal_ClampSteps(y_steps);
    x_abs = Gimbal_AbsLong(x_steps);
    y_abs = Gimbal_AbsLong(y_steps);
    max_abs = (x_abs > y_abs) ? x_abs : y_abs;

    if (max_abs == 0)
    {
        g_gimbal_last_error = GIMBAL_ERR_NONE;
        Gimbal_EnterState(GIMBAL_STATE_IDLE);
        return 1U;
    }

    base_speed = Gimbal_ClampSpeed(speed_sps);
    x_speed = (x_abs > 0) ? (base_speed * ((float)x_abs / (float)max_abs)) : base_speed;
    y_speed = (y_abs > 0) ? (base_speed * ((float)y_abs / (float)max_abs)) : base_speed;

    if (x_abs > 0)
    {
        x_started = BSP_Stepper_MoveSteps(STEPPER_AXIS_X, x_steps, x_speed, GIMBAL_DEFAULT_ACCEL_SPS2);
    }
    if (y_abs > 0)
    {
        y_started = BSP_Stepper_MoveSteps(STEPPER_AXIS_Y, y_steps, y_speed, GIMBAL_DEFAULT_ACCEL_SPS2);
    }

    if (((x_abs > 0) && (x_started == 0U)) ||
        ((y_abs > 0) && (y_started == 0U)))
    {
        BSP_Stepper_EmergencyStop(STEPPER_AXIS_X);
        BSP_Stepper_EmergencyStop(STEPPER_AXIS_Y);
        g_gimbal_last_error = GIMBAL_ERR_STEPPER;
        Gimbal_EnterState(GIMBAL_STATE_ERROR);
        return 0U;
    }

    g_gimbal_last_error = GIMBAL_ERR_NONE;
    Gimbal_EnterState(GIMBAL_STATE_MOVING);
    return 1U;
}

uint8_t Gimbal_MoveByPixelError(int16_t dx, int16_t dy)
{
    float x_float;
    float y_float;
    int32_t x_steps;
    int32_t y_steps;

    if (g_gimbal_calibrated == 0U)
    {
        g_gimbal_last_error = GIMBAL_ERR_CAL;
        return 0U;
    }

    x_float = (g_gimbal_cal_a * (float)dx) + (g_gimbal_cal_b * (float)dy);
    y_float = (g_gimbal_cal_c * (float)dx) + (g_gimbal_cal_d * (float)dy);
    x_steps = Gimbal_ClampSteps(Gimbal_RoundToSteps(x_float));
    y_steps = Gimbal_ClampSteps(Gimbal_RoundToSteps(y_float));

    return Gimbal_MoveRelativeSteps(x_steps, y_steps, GIMBAL_DEFAULT_SPEED_SPS);
}

void Gimbal_Zero(void)
{
    if (Gimbal_IsBusy() != 0U)
    {
        g_gimbal_last_error = GIMBAL_ERR_BUSY;
        return;
    }

    BSP_Stepper_SetPosition(STEPPER_AXIS_X, 0);
    BSP_Stepper_SetPosition(STEPPER_AXIS_Y, 0);
    g_gimbal_last_error = GIMBAL_ERR_NONE;
}

void Gimbal_Stop(void)
{
    BSP_Stepper_Stop(STEPPER_AXIS_X);
    BSP_Stepper_Stop(STEPPER_AXIS_Y);
    if (Gimbal_IsBusy() != 0U)
    {
        Gimbal_EnterState(GIMBAL_STATE_STOPPING);
    }
}

void Gimbal_EStop(void)
{
    BSP_Stepper_EmergencyStop(STEPPER_AXIS_X);
    BSP_Stepper_EmergencyStop(STEPPER_AXIS_Y);
    Gimbal_EnterState(GIMBAL_STATE_IDLE);
}

void Gimbal_SetHoldEnabled(uint8_t enabled)
{
    g_gimbal_hold_enabled = (enabled != 0U) ? 1U : 0U;
    BSP_Stepper_SetHoldEnabled(STEPPER_AXIS_X, g_gimbal_hold_enabled);
    BSP_Stepper_SetHoldEnabled(STEPPER_AXIS_Y, g_gimbal_hold_enabled);
}

uint8_t Gimbal_IsBusy(void)
{
    return (uint8_t)((BSP_Stepper_IsBusy(STEPPER_AXIS_X) != 0U) ||
                     (BSP_Stepper_IsBusy(STEPPER_AXIS_Y) != 0U));
}

uint8_t Gimbal_IsCalibrated(void)
{
    return g_gimbal_calibrated;
}

void Gimbal_SetCalibration(float a, float b, float c, float d)
{
    g_gimbal_cal_a = a;
    g_gimbal_cal_b = b;
    g_gimbal_cal_c = c;
    g_gimbal_cal_d = d;
    g_gimbal_calibrated = 1U;
    g_gimbal_last_error = GIMBAL_ERR_NONE;
}

void Gimbal_GetCalibration(float *a, float *b, float *c, float *d)
{
    if (a != (float *)0)
    {
        *a = g_gimbal_cal_a;
    }
    if (b != (float *)0)
    {
        *b = g_gimbal_cal_b;
    }
    if (c != (float *)0)
    {
        *c = g_gimbal_cal_c;
    }
    if (d != (float *)0)
    {
        *d = g_gimbal_cal_d;
    }
}

Gimbal_State_t Gimbal_GetState(void)
{
    return g_gimbal_state;
}

const char *Gimbal_GetStateName(void)
{
    switch (g_gimbal_state)
    {
    case GIMBAL_STATE_IDLE:
        return "IDLE";
    case GIMBAL_STATE_MOVING:
        return "MOVING";
    case GIMBAL_STATE_STOPPING:
        return "STOPPING";
    case GIMBAL_STATE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

void Gimbal_FormatStatus(char *buffer, size_t buffer_size, const char *prefix)
{
    BSP_StepperStatus_t x_status;
    BSP_StepperStatus_t y_status;

    if ((buffer == (char *)0) || (buffer_size == 0U))
    {
        return;
    }

    BSP_Stepper_GetStatus(STEPPER_AXIS_X, &x_status);
    BSP_Stepper_GetStatus(STEPPER_AXIS_Y, &y_status);

    (void)snprintf(buffer,
                   buffer_size,
                   "%s GIMBAL state=%s busy=%u hold=%u cal=%u age=%lu err=%s x_state=%s x_pos=%ld x_rem=%ld y_state=%s y_pos=%ld y_rem=%ld",
                   (prefix != (const char *)0) ? prefix : "OK",
                   Gimbal_GetStateName(),
                   (unsigned int)Gimbal_IsBusy(),
                   (unsigned int)g_gimbal_hold_enabled,
                   (unsigned int)g_gimbal_calibrated,
                   (unsigned long)Gimbal_StateElapsedMs(),
                   Gimbal_GetErrorName(g_gimbal_last_error),
                   BSP_Stepper_GetStateName(x_status.state),
                   (long)x_status.position_steps,
                   (long)x_status.remaining_steps,
                   BSP_Stepper_GetStateName(y_status.state),
                   (long)y_status.position_steps,
                   (long)y_status.remaining_steps);
}

static void Gimbal_EnterState(Gimbal_State_t next)
{
    g_gimbal_state = next;
    g_gimbal_state_enter_ms = HAL_GetTick();
}

static uint32_t Gimbal_StateElapsedMs(void)
{
    return (uint32_t)(HAL_GetTick() - g_gimbal_state_enter_ms);
}

static float Gimbal_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static int32_t Gimbal_AbsLong(int32_t value)
{
    if (value >= 0)
    {
        return value;
    }

    if (value == (-2147483647L - 1L))
    {
        return 2147483647L;
    }

    return -value;
}

static int32_t Gimbal_RoundToSteps(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }

    return (int32_t)(value - 0.5f);
}

static int32_t Gimbal_ClampSteps(int32_t value)
{
    if (value > GIMBAL_MAX_RELATIVE_STEPS)
    {
        g_gimbal_last_error = GIMBAL_ERR_RANGE;
        return GIMBAL_MAX_RELATIVE_STEPS;
    }
    if (value < -GIMBAL_MAX_RELATIVE_STEPS)
    {
        g_gimbal_last_error = GIMBAL_ERR_RANGE;
        return -GIMBAL_MAX_RELATIVE_STEPS;
    }

    return value;
}

static float Gimbal_ClampSpeed(float speed_sps)
{
    float speed = speed_sps;

    if (speed <= 0.0f)
    {
        speed = GIMBAL_DEFAULT_SPEED_SPS;
    }
    if (Gimbal_AbsFloat(speed) > GIMBAL_MAX_COMMAND_SPEED_SPS)
    {
        speed = GIMBAL_MAX_COMMAND_SPEED_SPS;
    }

    return speed;
}

static const char *Gimbal_GetErrorName(Gimbal_Error_t error)
{
    switch (error)
    {
    case GIMBAL_ERR_NONE:
        return "NONE";
    case GIMBAL_ERR_BUSY:
        return "BUSY";
    case GIMBAL_ERR_STEPPER:
        return "STEPPER";
    case GIMBAL_ERR_CAL:
        return "CAL";
    case GIMBAL_ERR_RANGE:
        return "RANGE";
    default:
        return "UNKNOWN";
    }
}
