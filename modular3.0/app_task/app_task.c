#include "app_task.h"

#include "app_aim.h"
#include "app_imu.h"
#include "app_motion.h"
#include "app_track.h"
#include "app_turn.h"
#include "main.h"

#include <stdio.h>

#define TASK_Q2_AIM_TIMEOUT_MS (2000U)
#define TASK_Q3_AIM_TIMEOUT_MS (4000U)

static Task_Mode_t g_task_mode = TASK_MODE_NONE;
static Task_State_t g_task_state = TASK_STATE_IDLE;
static Task_StopReason_t g_task_stop_reason = TASK_STOP_REASON_NONE;
static uint8_t g_task_initialized = 0U;
static uint32_t g_task_state_enter_ms = 0U;
static uint32_t g_task_start_ms = 0U;
static uint32_t g_task_stop_ms = 0U;

static void Task_EnterState(Task_State_t next);
static void Task_StopChildren(void);
static uint8_t Task_ModeIsSupported(Task_Mode_t mode);
static uint8_t Task_IsRunning(void);

void Task_Init(void)
{
    g_task_mode = TASK_MODE_Q1_TRACK;
    g_task_state = TASK_STATE_SELECTED;
    g_task_stop_reason = TASK_STOP_REASON_NONE;
    g_task_state_enter_ms = HAL_GetTick();
    g_task_start_ms = 0U;
    g_task_stop_ms = 0U;
    g_task_initialized = 1U;
}

void Task_Poll(void)
{
    Aim_State_t aim_state;
    Track_State_t track_state;
    Track_StopReason_t track_stop_reason;

    if ((g_task_initialized == 0U) || (g_task_state != TASK_STATE_RUNNING))
    {
        return;
    }

    if ((g_task_mode != TASK_MODE_Q1_TRACK) &&
        (g_task_mode != TASK_MODE_Q2_AIM_STATIC) &&
        (g_task_mode != TASK_MODE_Q3_AIM_X_REV_SCAN) &&
        (g_task_mode != TASK_MODE_ADV1_AIM_TRACK) &&
        (g_task_mode != TASK_MODE_ADV_TRACK_TEST))
    {
        g_task_stop_reason = TASK_STOP_REASON_UNSUPPORTED;
        g_task_stop_ms = HAL_GetTick();
        Task_StopChildren();
        Task_EnterState(TASK_STATE_ERROR);
        return;
    }

    track_state = Track_GetState();
    if (g_task_mode == TASK_MODE_ADV1_AIM_TRACK)
    {
        Aim_UpdateTrackHint(track_state,
                            Turn_GetDirection(),
                            Turn_GetProgressDeg(),
                            Imu_GetGyroZDps());
    }

    if ((g_task_mode == TASK_MODE_Q2_AIM_STATIC) ||
        (g_task_mode == TASK_MODE_Q3_AIM_X_REV_SCAN))
    {
        aim_state = Aim_GetState();
        if (aim_state == AIM_STATE_LOCKED)
        {
            g_task_stop_reason = TASK_STOP_REASON_COMPLETE;
            g_task_stop_ms = HAL_GetTick();
            Task_EnterState(TASK_STATE_FINISHED);
            return;
        }

        if (aim_state == AIM_STATE_ERROR)
        {
            g_task_stop_reason = TASK_STOP_REASON_CHILD_STOPPED;
            g_task_stop_ms = HAL_GetTick();
            Task_EnterState(TASK_STATE_ERROR);
            return;
        }

        if (aim_state == AIM_STATE_IDLE)
        {
            g_task_stop_reason = TASK_STOP_REASON_CHILD_STOPPED;
            g_task_stop_ms = HAL_GetTick();
            Task_EnterState(TASK_STATE_ERROR);
        }
        return;
    }

    if (g_task_mode == TASK_MODE_ADV1_AIM_TRACK)
    {
        aim_state = Aim_GetState();
        if ((aim_state == AIM_STATE_ERROR) ||
            (aim_state == AIM_STATE_IDLE) ||
            (aim_state == AIM_STATE_LOCKED))
        {
            g_task_stop_reason = TASK_STOP_REASON_CHILD_STOPPED;
            g_task_stop_ms = HAL_GetTick();
            Task_StopChildren();
            Task_EnterState(TASK_STATE_ERROR);
            return;
        }
    }

    if (track_state == TRACK_STATE_FINISHED)
    {
        if (g_task_mode == TASK_MODE_ADV1_AIM_TRACK)
        {
            Aim_Stop();
        }
        g_task_stop_reason = TASK_STOP_REASON_COMPLETE;
        g_task_stop_ms = HAL_GetTick();
        Task_EnterState(TASK_STATE_FINISHED);
        return;
    }

    if (track_state == TRACK_STATE_STOPPED)
    {
        track_stop_reason = Track_GetStopReason();
        g_task_stop_ms = HAL_GetTick();
        if (g_task_mode == TASK_MODE_ADV1_AIM_TRACK)
        {
            Aim_Stop();
        }
        if (track_stop_reason == TRACK_STOP_REASON_USER)
        {
            g_task_stop_reason = TASK_STOP_REASON_USER;
            Task_EnterState(TASK_STATE_STOPPED);
        }
        else
        {
            g_task_stop_reason = TASK_STOP_REASON_CHILD_STOPPED;
            Task_EnterState(TASK_STATE_ERROR);
        }
        return;
    }

    if (Track_IsRunning() == 0U)
    {
        g_task_stop_reason = TASK_STOP_REASON_CHILD_STOPPED;
        g_task_stop_ms = HAL_GetTick();
        Task_StopChildren();
        Task_EnterState(TASK_STATE_ERROR);
    }
}

uint8_t Task_Select(Task_Mode_t mode)
{
    if (g_task_initialized == 0U)
    {
        return 0U;
    }

    if (Task_IsRunning() != 0U)
    {
        return 0U;
    }

    if (Task_ModeIsSupported(mode) == 0U)
    {
        g_task_stop_reason = TASK_STOP_REASON_UNSUPPORTED;
        Task_EnterState(TASK_STATE_ERROR);
        return 0U;
    }

    g_task_mode = mode;
    g_task_stop_reason = TASK_STOP_REASON_NONE;
    g_task_start_ms = 0U;
    g_task_stop_ms = 0U;
    if (mode == TASK_MODE_Q1_TRACK)
    {
        (void)Track_ApplyPreset(TRACK_PRESET_NORMAL);
    }
    else if ((mode == TASK_MODE_ADV1_AIM_TRACK) ||
             (mode == TASK_MODE_ADV_TRACK_TEST))
    {
        (void)Track_ApplyPreset(TRACK_PRESET_ADV);
        (void)Track_SetTargetLaps(1U);
    }
    Task_EnterState(TASK_STATE_SELECTED);
    return 1U;
}

uint8_t Task_SelectQuestion(uint8_t question_id)
{
    switch (question_id)
    {
    case 1U:
        return Task_Select(TASK_MODE_Q1_TRACK);
    case 2U:
        return Task_Select(TASK_MODE_Q2_AIM_STATIC);
    case 3U:
        return Task_Select(TASK_MODE_Q3_AIM_X_REV_SCAN);
    case 4U:
        return Task_Select(TASK_MODE_ADV1_AIM_TRACK);
    default:
        g_task_stop_reason = TASK_STOP_REASON_UNSUPPORTED;
        if (Task_IsRunning() == 0U)
        {
            Task_EnterState(TASK_STATE_ERROR);
        }
        return 0U;
    }
}

uint8_t Task_Start(void)
{
    if ((g_task_initialized == 0U) || (g_task_state == TASK_STATE_RUNNING))
    {
        return 0U;
    }

    if (Task_ModeIsSupported(g_task_mode) == 0U)
    {
        g_task_stop_reason = TASK_STOP_REASON_UNSUPPORTED;
        g_task_stop_ms = HAL_GetTick();
        Task_EnterState(TASK_STATE_ERROR);
        return 0U;
    }

    g_task_start_ms = HAL_GetTick();
    g_task_stop_ms = 0U;
    g_task_stop_reason = TASK_STOP_REASON_NONE;

    if (g_task_mode == TASK_MODE_Q1_TRACK)
    {
        (void)Track_ApplyPreset(TRACK_PRESET_NORMAL);
        if (Track_Start() != 0U)
        {
            Task_EnterState(TASK_STATE_RUNNING);
            return 1U;
        }
    }
    else if (g_task_mode == TASK_MODE_Q2_AIM_STATIC)
    {
        if (Aim_StartOnce(TASK_Q2_AIM_TIMEOUT_MS) != 0U)
        {
            Task_EnterState(TASK_STATE_RUNNING);
            return 1U;
        }
    }
    else if (g_task_mode == TASK_MODE_Q3_AIM_X_REV_SCAN)
    {
        if (Aim_StartQuestion3(TASK_Q3_AIM_TIMEOUT_MS) != 0U)
        {
            Task_EnterState(TASK_STATE_RUNNING);
            return 1U;
        }
    }
    else if (g_task_mode == TASK_MODE_ADV1_AIM_TRACK)
    {
        (void)Track_ApplyPreset(TRACK_PRESET_ADV);
        (void)Track_SetTargetLaps(1U);
        if ((Track_Start() != 0U) && (Aim_StartTrack() != 0U))
        {
            Task_EnterState(TASK_STATE_RUNNING);
            return 1U;
        }
    }
    else if (g_task_mode == TASK_MODE_ADV_TRACK_TEST)
    {
        Aim_Stop();
        (void)Track_ApplyPreset(TRACK_PRESET_ADV);
        (void)Track_SetTargetLaps(1U);
        if (Track_Start() != 0U)
        {
            Task_EnterState(TASK_STATE_RUNNING);
            return 1U;
        }
    }

    g_task_stop_reason = TASK_STOP_REASON_START_FAIL;
    g_task_stop_ms = HAL_GetTick();
    Task_StopChildren();
    Task_EnterState(TASK_STATE_ERROR);
    return 0U;
}

void Task_Stop(void)
{
    if (g_task_initialized == 0U)
    {
        return;
    }

    Task_StopChildren();
    g_task_stop_reason = TASK_STOP_REASON_USER;
    if (g_task_start_ms != 0U)
    {
        g_task_stop_ms = HAL_GetTick();
    }
    Task_EnterState(TASK_STATE_STOPPED);
}

void Task_Reset(void)
{
    if (g_task_initialized == 0U)
    {
        return;
    }

    Task_StopChildren();
    g_task_stop_reason = TASK_STOP_REASON_NONE;
    g_task_start_ms = 0U;
    g_task_stop_ms = 0U;
    Task_EnterState((g_task_mode == TASK_MODE_NONE) ? TASK_STATE_IDLE : TASK_STATE_SELECTED);
}

Task_Mode_t Task_GetMode(void)
{
    return g_task_mode;
}

const char *Task_GetModeName(void)
{
    switch (g_task_mode)
    {
    case TASK_MODE_NONE:
        return "NONE";
    case TASK_MODE_Q1_TRACK:
        return "Q1_TRACK";
    case TASK_MODE_Q2_AIM_STATIC:
        return "Q2_AIM_STATIC";
    case TASK_MODE_Q3_AIM_X_REV_SCAN:
        return "Q3_AIM_X_REV_SCAN";
    case TASK_MODE_ADV1_AIM_TRACK:
        return "ADV1_AIM_TRACK";
    case TASK_MODE_ADV_TRACK_TEST:
        return "ADV_TRACK_TEST";
    default:
        return "UNKNOWN";
    }
}

Task_State_t Task_GetState(void)
{
    return g_task_state;
}

const char *Task_GetStateName(void)
{
    switch (g_task_state)
    {
    case TASK_STATE_IDLE:
        return "IDLE";
    case TASK_STATE_SELECTED:
        return "SELECTED";
    case TASK_STATE_RUNNING:
        return "RUNNING";
    case TASK_STATE_FINISHED:
        return "FINISHED";
    case TASK_STATE_STOPPED:
        return "STOPPED";
    case TASK_STATE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

Task_StopReason_t Task_GetStopReason(void)
{
    return g_task_stop_reason;
}

const char *Task_GetStopReasonName(void)
{
    switch (g_task_stop_reason)
    {
    case TASK_STOP_REASON_NONE:
        return "NONE";
    case TASK_STOP_REASON_USER:
        return "USER";
    case TASK_STOP_REASON_COMPLETE:
        return "COMPLETE";
    case TASK_STOP_REASON_START_FAIL:
        return "START_FAIL";
    case TASK_STOP_REASON_CHILD_STOPPED:
        return "CHILD_STOPPED";
    case TASK_STOP_REASON_UNSUPPORTED:
        return "UNSUPPORTED";
    default:
        return "UNKNOWN";
    }
}

uint32_t Task_GetElapsedMs(void)
{
    if (g_task_start_ms == 0U)
    {
        return 0U;
    }

    if ((Task_IsRunning() == 0U) && (g_task_stop_ms != 0U))
    {
        return (uint32_t)(g_task_stop_ms - g_task_start_ms);
    }

    return (uint32_t)(HAL_GetTick() - g_task_start_ms);
}

void Task_FormatStatus(char *buffer, size_t buffer_size, const char *prefix)
{
    if ((buffer == (char *)0) || (buffer_size == 0U))
    {
        return;
    }

    if (prefix == (const char *)0)
    {
        prefix = "OK";
    }

    (void)snprintf(buffer,
                   buffer_size,
                   "%s TASK state=%s mode=%s elapsed=%lu state_ms=%lu stop=%s track=%s track_stop=%s laps=%u/%u corners=%u aim=%s",
                   prefix,
                   Task_GetStateName(),
                   Task_GetModeName(),
                   (unsigned long)Task_GetElapsedMs(),
                   (unsigned long)(HAL_GetTick() - g_task_state_enter_ms),
                   Task_GetStopReasonName(),
                   Track_GetStateName(),
                   Track_GetStopReasonName(),
                   (unsigned int)Track_GetLapsDone(),
                   (unsigned int)Track_GetTargetLaps(),
                   (unsigned int)Track_GetCornerCount(),
                   Aim_GetStateName());
}

static void Task_EnterState(Task_State_t next)
{
    g_task_state = next;
    g_task_state_enter_ms = HAL_GetTick();
}

static void Task_StopChildren(void)
{
    Aim_Stop();
    Track_Stop();
    Turn_Stop();
    Motion_Stop();
}

static uint8_t Task_ModeIsSupported(Task_Mode_t mode)
{
    return (uint8_t)((mode == TASK_MODE_Q1_TRACK) ||
                     (mode == TASK_MODE_Q2_AIM_STATIC) ||
                     (mode == TASK_MODE_Q3_AIM_X_REV_SCAN) ||
                     (mode == TASK_MODE_ADV1_AIM_TRACK) ||
                     (mode == TASK_MODE_ADV_TRACK_TEST));
}

static uint8_t Task_IsRunning(void)
{
    return (uint8_t)(g_task_state == TASK_STATE_RUNNING);
}
