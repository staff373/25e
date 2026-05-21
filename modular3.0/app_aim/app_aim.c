#include "app_aim.h"

#include "app_gimbal.h"
#include "app_vision.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>

#define AIM_DEFAULT_ONCE_TIMEOUT_MS     (2000U)
#define AIM_TRACK_INTERVAL_MS           (50U)
#define AIM_LOCK_TOLERANCE_PX           (4)

typedef enum
{
    AIM_ERR_NONE = 0,
    AIM_ERR_NO_TARGET,
    AIM_ERR_TIMEOUT,
    AIM_ERR_GIMBAL,
    AIM_ERR_CAL
} Aim_Error_t;

static Aim_State_t g_aim_state = AIM_STATE_IDLE;
static Aim_Error_t g_aim_last_error = AIM_ERR_NONE;
static uint32_t g_aim_state_enter_ms = 0U;
static uint32_t g_aim_last_step_ms = 0U;
static uint32_t g_aim_once_timeout_ms = AIM_DEFAULT_ONCE_TIMEOUT_MS;

static void Aim_EnterState(Aim_State_t next);
static uint32_t Aim_StateElapsedMs(void);
static int16_t Aim_AbsInt16(int16_t value);
static uint8_t Aim_TargetWithinTolerance(const Vision_Target_t *target);
static void Aim_RunOnce(void);
static void Aim_RunTrack(void);
static const char *Aim_GetErrorName(Aim_Error_t error);

void Aim_Init(void)
{
    g_aim_last_error = AIM_ERR_NONE;
    g_aim_last_step_ms = 0U;
    g_aim_once_timeout_ms = AIM_DEFAULT_ONCE_TIMEOUT_MS;
    Aim_EnterState(AIM_STATE_IDLE);
}

void Aim_Poll(void)
{
    switch (g_aim_state)
    {
    case AIM_STATE_ONCE:
        Aim_RunOnce();
        break;
    case AIM_STATE_TRACKING:
        Aim_RunTrack();
        break;
    case AIM_STATE_IDLE:
    case AIM_STATE_LOCKED:
    case AIM_STATE_ERROR:
    default:
        break;
    }
}

uint8_t Aim_StartOnce(uint32_t timeout_ms)
{
    if (Gimbal_IsCalibrated() == 0U)
    {
        g_aim_last_error = AIM_ERR_CAL;
        Aim_EnterState(AIM_STATE_ERROR);
        return 0U;
    }

    g_aim_once_timeout_ms = (timeout_ms == 0U) ? AIM_DEFAULT_ONCE_TIMEOUT_MS : timeout_ms;
    g_aim_last_error = AIM_ERR_NONE;
    g_aim_last_step_ms = 0U;
    Aim_EnterState(AIM_STATE_ONCE);
    return 1U;
}

uint8_t Aim_StartTrack(void)
{
    if (Gimbal_IsCalibrated() == 0U)
    {
        g_aim_last_error = AIM_ERR_CAL;
        Aim_EnterState(AIM_STATE_ERROR);
        return 0U;
    }

    g_aim_last_error = AIM_ERR_NONE;
    g_aim_last_step_ms = 0U;
    Aim_EnterState(AIM_STATE_TRACKING);
    return 1U;
}

void Aim_Stop(void)
{
    Gimbal_Stop();
    Aim_EnterState(AIM_STATE_IDLE);
}

Aim_State_t Aim_GetState(void)
{
    return g_aim_state;
}

const char *Aim_GetStateName(void)
{
    switch (g_aim_state)
    {
    case AIM_STATE_IDLE:
        return "IDLE";
    case AIM_STATE_ONCE:
        return "ONCE";
    case AIM_STATE_TRACKING:
        return "TRACKING";
    case AIM_STATE_LOCKED:
        return "LOCKED";
    case AIM_STATE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

void Aim_FormatStatus(char *buffer, size_t buffer_size, const char *prefix)
{
    Vision_Target_t target;
    uint8_t has_target;

    if ((buffer == (char *)0) || (buffer_size == 0U))
    {
        return;
    }

    has_target = Vision_GetTarget(&target);
    if (has_target == 0U)
    {
        target.dx = 0;
        target.dy = 0;
        target.age_ms = 0U;
    }

    (void)snprintf(buffer,
                   buffer_size,
                   "%s AIM state=%s err=%s elapsed=%lu target=%u dx=%d dy=%d age=%lu gimbal=%s cal=%u",
                   (prefix != (const char *)0) ? prefix : "OK",
                   Aim_GetStateName(),
                   Aim_GetErrorName(g_aim_last_error),
                   (unsigned long)Aim_StateElapsedMs(),
                   (unsigned int)has_target,
                   (int)target.dx,
                   (int)target.dy,
                   (unsigned long)target.age_ms,
                   Gimbal_GetStateName(),
                   (unsigned int)Gimbal_IsCalibrated());
}

static void Aim_EnterState(Aim_State_t next)
{
    g_aim_state = next;
    g_aim_state_enter_ms = HAL_GetTick();
}

static uint32_t Aim_StateElapsedMs(void)
{
    return (uint32_t)(HAL_GetTick() - g_aim_state_enter_ms);
}

static int16_t Aim_AbsInt16(int16_t value)
{
    if (value >= 0)
    {
        return value;
    }
    if (value == (int16_t)-32768)
    {
        return 32767;
    }
    return (int16_t)(-value);
}

static uint8_t Aim_TargetWithinTolerance(const Vision_Target_t *target)
{
    if (target == (const Vision_Target_t *)0)
    {
        return 0U;
    }

    return (uint8_t)((Aim_AbsInt16(target->dx) <= AIM_LOCK_TOLERANCE_PX) &&
                     (Aim_AbsInt16(target->dy) <= AIM_LOCK_TOLERANCE_PX));
}

static void Aim_RunOnce(void)
{
    Vision_Target_t target;

    if (Aim_StateElapsedMs() > g_aim_once_timeout_ms)
    {
        g_aim_last_error = AIM_ERR_TIMEOUT;
        Aim_EnterState(AIM_STATE_ERROR);
        return;
    }

    if (Gimbal_IsBusy() != 0U)
    {
        return;
    }

    if (Vision_GetTarget(&target) == 0U)
    {
        g_aim_last_error = AIM_ERR_NO_TARGET;
        return;
    }

    if (Aim_TargetWithinTolerance(&target) != 0U)
    {
        g_aim_last_error = AIM_ERR_NONE;
        Aim_EnterState(AIM_STATE_LOCKED);
        return;
    }

    if (Gimbal_MoveByPixelError(target.dx, target.dy) == 0U)
    {
        g_aim_last_error = (Gimbal_IsCalibrated() == 0U) ? AIM_ERR_CAL : AIM_ERR_GIMBAL;
        Aim_EnterState(AIM_STATE_ERROR);
    }
}

static void Aim_RunTrack(void)
{
    Vision_Target_t target;
    uint32_t now_ms;

    if (Gimbal_IsBusy() != 0U)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - g_aim_last_step_ms) < AIM_TRACK_INTERVAL_MS)
    {
        return;
    }
    g_aim_last_step_ms = now_ms;

    if (Vision_GetTarget(&target) == 0U)
    {
        g_aim_last_error = AIM_ERR_NO_TARGET;
        return;
    }

    if (Aim_TargetWithinTolerance(&target) != 0U)
    {
        g_aim_last_error = AIM_ERR_NONE;
        return;
    }

    if (Gimbal_MoveByPixelError(target.dx, target.dy) == 0U)
    {
        g_aim_last_error = (Gimbal_IsCalibrated() == 0U) ? AIM_ERR_CAL : AIM_ERR_GIMBAL;
        Aim_EnterState(AIM_STATE_ERROR);
    }
}

static const char *Aim_GetErrorName(Aim_Error_t error)
{
    switch (error)
    {
    case AIM_ERR_NONE:
        return "NONE";
    case AIM_ERR_NO_TARGET:
        return "NO_TARGET";
    case AIM_ERR_TIMEOUT:
        return "TIMEOUT";
    case AIM_ERR_GIMBAL:
        return "GIMBAL";
    case AIM_ERR_CAL:
        return "CAL";
    default:
        return "UNKNOWN";
    }
}
