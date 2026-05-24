#include "app_track.h"

#include "app_imu.h"
#include "app_motion.h"
#include "app_sensor.h"
#include "app_turn.h"
#include "pid_core.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

#define APP_TRACK_UPDATE_PERIOD_MS   (10U)
#define APP_TRACK_DEFAULT_BASE_DUTY  (33.0f)   /* 循迹基础速度，直线和出弯恢复都用它 */
#define APP_TRACK_DEFAULT_KP         (60.0f)   /* 比例修正强度，越大转向越积极 */
#define APP_TRACK_DEFAULT_KD         (8.0f)    /* 微分阻尼强度，越大越压摆动 */
#define APP_TRACK_DEFAULT_CENTER_BIAS (-0.8f)   /* 居中态固定纠偏，默认 0，现场按机械偏差调 */
#define APP_TRACK_DEFAULT_CORNER_ADVANCE_MS (20U) /* 右角点后继续直走多久再进入转弯 */
#define APP_TRACK_DEFAULT_RECOVER_MS (0U)    /* 转完后先直走多久，再重新回到循迹 */
#define APP_TRACK_DEFAULT_LEFT_TRIM  (0.89f)  /* 机械右偏补偿：左侧轮按实测比例降速 */
#define APP_TRACK_DEFAULT_RIGHT_TRIM (1.00f)
// 进阶题参数
#define APP_TRACK_ADV_BASE_DUTY      (26.0f)
#define APP_TRACK_ADV_KP             (50.0f)
#define APP_TRACK_ADV_KD             (8.0f)
#define APP_TRACK_ADV_CENTER_BIAS    (1.3f)
#define APP_TRACK_ADV_CORNER_ADVANCE_MS (130U)
#define APP_TRACK_ADV_RECOVER_MS     (0U)
#define APP_TRACK_ADV_LEFT_TRIM      (0.95f)
#define APP_TRACK_ADV_RIGHT_TRIM     (1.00f)
#define APP_TRACK_PARAM_GAIN_MAX     (FLT_MAX)
#define APP_TRACK_PARAM_BIAS_LIMIT   (100.0f)
#define APP_TRACK_PARAM_TRIM_MAX     (1.50f)
#define APP_TRACK_PARAM_TIME_MS_MAX  (4294967040.0f)
#define APP_TRACK_DEFAULT_TARGET_LAPS (1U)
#define APP_TRACK_MIN_TARGET_LAPS     (1U)
#define APP_TRACK_MAX_TARGET_LAPS     (5U)
#define APP_TRACK_CORNERS_PER_LAP     (4U)
#define APP_TRACK_DEFAULT_CORNER_DIR  (TURN_DIR_RIGHT)

static PID_Handle_t g_track_pid;
static Track_State_t g_track_state = TRACK_STATE_IDLE;
static Track_StopReason_t g_track_stop_reason = TRACK_STOP_REASON_NONE;
static uint8_t g_track_initialized = 0U;
static uint32_t g_track_last_update_ms = 0U;
static uint32_t g_track_state_enter_ms = 0U;
static uint32_t g_track_start_ms = 0U;
static uint32_t g_track_stop_ms = 0U;
static uint32_t g_track_corner_advance_start_ms = 0U;
static uint32_t g_track_recover_start_ms = 0U;
static int8_t g_track_corner_dir = 0;
static uint8_t g_track_target_laps = APP_TRACK_DEFAULT_TARGET_LAPS;
static uint8_t g_track_corner_count = 0U;
static uint8_t g_track_laps_done = 0U;
static Track_Preset_t g_track_preset = TRACK_PRESET_NORMAL;
static float g_track_base_duty = APP_TRACK_DEFAULT_BASE_DUTY;
static float g_track_kp = APP_TRACK_DEFAULT_KP;
static float g_track_kd = APP_TRACK_DEFAULT_KD;
static float g_track_center_bias = APP_TRACK_DEFAULT_CENTER_BIAS;
static uint32_t g_track_corner_advance_ms = APP_TRACK_DEFAULT_CORNER_ADVANCE_MS;
static uint32_t g_track_recover_ms = APP_TRACK_DEFAULT_RECOVER_MS;
static float g_track_left_trim = APP_TRACK_DEFAULT_LEFT_TRIM;
static float g_track_right_trim = APP_TRACK_DEFAULT_RIGHT_TRIM;
static float g_track_last_correction = 0.0f;
static float g_track_last_line_correction = 0.0f;

static void Track_EnterState(Track_State_t next);
static void Track_ResetProgress(void);
static void Track_StopWithReason(Track_State_t stop_state, Track_StopReason_t reason);
static void Track_RecordCompletedCorner(void);
static uint8_t Track_IsTargetComplete(void);
static void Track_ApplyPidParams(void);
static void Track_ApplyTuning(float base_duty,
                              float kp,
                              float kd,
                              float center_bias,
                              uint32_t corner_advance_ms,
                              uint32_t recover_ms,
                              float left_trim,
                              float right_trim);
static float Track_Clamp(float value, float min_value, float max_value);
static void Track_ApplyFollowControl(void);
static void Track_SetAutoDuty(float left_duty, float right_duty);
static uint8_t Track_StartLatchedTurn(void);
static int8_t Track_FilterCornerDirection(int8_t detected_dir);

void Track_Init(void)
{
    PID_Params_t params;

    (void)memset(&params, 0, sizeof(params));
    params.kp = g_track_kp;
    params.kd = g_track_kd;
    params.integral_min = 0.0f;
    params.integral_max = 0.0f;
    params.output_min = -100.0f;
    params.output_max = 100.0f;
    params.deadband = 0.0f;
    params.deadband_mode = PID_DEADBAND_MODE_CLEAR;

    PID_Core_Init(&g_track_pid, &params);
    g_track_last_update_ms = HAL_GetTick();
    g_track_state_enter_ms = g_track_last_update_ms;
    g_track_start_ms = 0U;
    g_track_stop_ms = 0U;
    g_track_target_laps = APP_TRACK_DEFAULT_TARGET_LAPS;
    g_track_preset = TRACK_PRESET_NORMAL;
    g_track_stop_reason = TRACK_STOP_REASON_NONE;
    Track_ResetProgress();
    g_track_state = TRACK_STATE_IDLE;
    g_track_initialized = 1U;
}

void Track_Poll(void)
{
    uint32_t now_ms;
    int8_t corner_dir;

    if (g_track_initialized == 0U)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - g_track_last_update_ms) < APP_TRACK_UPDATE_PERIOD_MS)
    {
        return;
    }

    g_track_last_update_ms = now_ms;
    AppSensor_ReadNow();

    switch (g_track_state)
    {
    case TRACK_STATE_IDLE:
    case TRACK_STATE_FINISHED:
    case TRACK_STATE_STOPPED:
        /* 停车状态不再持续抢占电机，避免覆盖蓝牙手动 MOTOR / TURN 控制。 */
        return;

    case TRACK_STATE_CORNER_ADVANCE:
        Track_SetAutoDuty(g_track_base_duty, g_track_base_duty);
        if ((uint32_t)(now_ms - g_track_corner_advance_start_ms) >= g_track_corner_advance_ms)
        {
            (void)Track_StartLatchedTurn();
        }
        return;

    case TRACK_STATE_TURNING:
        if (Turn_IsActive() != 0U)
        {
            return;
        }

        if (Turn_WasLastTimeout() != 0U)
        {
            Track_StopWithReason(TRACK_STATE_STOPPED, TRACK_STOP_REASON_TURN_TIMEOUT);
            return;
        }

        Track_RecordCompletedCorner();
        if (Track_IsTargetComplete() != 0U)
        {
            Track_StopWithReason(TRACK_STATE_FINISHED, TRACK_STOP_REASON_COMPLETE);
            return;
        }

        Track_EnterState(TRACK_STATE_RECOVER_LINE);
        g_track_recover_start_ms = now_ms;
        PID_Core_Reset(&g_track_pid);
        Track_SetAutoDuty(g_track_base_duty, g_track_base_duty);
        return;

    case TRACK_STATE_RECOVER_LINE:
        Track_SetAutoDuty(g_track_base_duty, g_track_base_duty);
        if ((uint32_t)(now_ms - g_track_recover_start_ms) >= g_track_recover_ms)
        {
            Track_EnterState(TRACK_STATE_LINE_FOLLOW);
            g_track_corner_dir = 0;
            g_track_corner_advance_start_ms = 0U;
            g_track_last_correction = 0.0f;
            g_track_last_line_correction = 0.0f;
            PID_Core_Reset(&g_track_pid);
        }
        return;

    default:
        break;
    }

    if ((g_track_state == TRACK_STATE_LINE_FOLLOW) &&
        (AppSensor_GetStateType() == SENSOR_STATE_LOST))
    {
        Track_StopWithReason(TRACK_STATE_STOPPED, TRACK_STOP_REASON_LOST_LINE);
        return;
    }

    corner_dir = Track_FilterCornerDirection(AppSensor_GetCornerDirection());
    if (g_track_state == TRACK_STATE_LINE_FOLLOW)
    {
        if (corner_dir != 0)
        {
            g_track_corner_dir = corner_dir;
            g_track_corner_advance_start_ms = now_ms;
            g_track_last_correction = 0.0f;
            g_track_last_line_correction = 0.0f;
            PID_Core_Reset(&g_track_pid);
            Track_EnterState(TRACK_STATE_CORNER_ADVANCE);
            Track_SetAutoDuty(g_track_base_duty, g_track_base_duty);
            if (g_track_corner_advance_ms == 0U)
            {
                (void)Track_StartLatchedTurn();
            }
            return;
        }

        Track_ApplyFollowControl();
        return;
    }
}

uint8_t Track_Start(void)
{
    if ((g_track_initialized == 0U) || (Imu_IsOnline() == 0U))
    {
        return 0U;
    }

    Turn_Stop();
    Motion_Stop();
    PID_Core_Reset(&g_track_pid);
    Track_ResetProgress();
    g_track_start_ms = HAL_GetTick();
    g_track_stop_ms = 0U;
    g_track_stop_reason = TRACK_STOP_REASON_NONE;
    Track_EnterState(TRACK_STATE_LINE_FOLLOW);
    return 1U;
}

void Track_Stop(void)
{
    Track_StopWithReason(TRACK_STATE_STOPPED, TRACK_STOP_REASON_USER);
}

Track_State_t Track_GetState(void)
{
    return g_track_state;
}

const char *Track_GetStateName(void)
{
    switch (g_track_state)
    {
    case TRACK_STATE_IDLE:
        return "IDLE";
    case TRACK_STATE_LINE_FOLLOW:
        return "LINE_FOLLOW";
    case TRACK_STATE_CORNER_ADVANCE:
        return "CORNER_ADVANCE";
    case TRACK_STATE_TURNING:
        return "TURNING";
    case TRACK_STATE_RECOVER_LINE:
        return "RECOVER_LINE";
    case TRACK_STATE_FINISHED:
        return "FINISHED";
    case TRACK_STATE_STOPPED:
        return "STOPPED";
    default:
        return "UNKNOWN";
    }
}

Track_StopReason_t Track_GetStopReason(void)
{
    return g_track_stop_reason;
}

const char *Track_GetStopReasonName(void)
{
    switch (g_track_stop_reason)
    {
    case TRACK_STOP_REASON_NONE:
        return "NONE";
    case TRACK_STOP_REASON_USER:
        return "USER";
    case TRACK_STOP_REASON_COMPLETE:
        return "COMPLETE";
    case TRACK_STOP_REASON_TURN_TIMEOUT:
        return "TURN_TIMEOUT";
    case TRACK_STOP_REASON_TURN_START_FAIL:
        return "TURN_START_FAIL";
    case TRACK_STOP_REASON_LOST_LINE:
        return "LOST_LINE";
    default:
        return "UNKNOWN";
    }
}

uint8_t Track_IsRunning(void)
{
    return (uint8_t)((g_track_state != TRACK_STATE_IDLE) &&
                     (g_track_state != TRACK_STATE_FINISHED) &&
                     (g_track_state != TRACK_STATE_STOPPED));
}

uint8_t Track_SetTargetLaps(uint8_t laps)
{
    if (laps < APP_TRACK_MIN_TARGET_LAPS)
    {
        laps = APP_TRACK_MIN_TARGET_LAPS;
    }
    else if (laps > APP_TRACK_MAX_TARGET_LAPS)
    {
        laps = APP_TRACK_MAX_TARGET_LAPS;
    }

    g_track_target_laps = laps;
    if ((Track_IsRunning() != 0U) && (Track_IsTargetComplete() != 0U))
    {
        Track_StopWithReason(TRACK_STATE_FINISHED, TRACK_STOP_REASON_COMPLETE);
    }

    return 1U;
}

uint8_t Track_GetTargetLaps(void)
{
    return g_track_target_laps;
}

uint8_t Track_GetLapsDone(void)
{
    return g_track_laps_done;
}

uint8_t Track_GetCornerCount(void)
{
    return g_track_corner_count;
}

uint32_t Track_GetElapsedMs(void)
{
    if (g_track_start_ms == 0U)
    {
        return 0U;
    }

    if ((Track_IsRunning() == 0U) && (g_track_stop_ms != 0U))
    {
        return (uint32_t)(g_track_stop_ms - g_track_start_ms);
    }

    return (uint32_t)(HAL_GetTick() - g_track_start_ms);
}

uint8_t Track_ApplyPreset(Track_Preset_t preset)
{
    switch (preset)
    {
    case TRACK_PRESET_NORMAL:
        Track_ApplyTuning(APP_TRACK_DEFAULT_BASE_DUTY,
                          APP_TRACK_DEFAULT_KP,
                          APP_TRACK_DEFAULT_KD,
                          APP_TRACK_DEFAULT_CENTER_BIAS,
                          APP_TRACK_DEFAULT_CORNER_ADVANCE_MS,
                          APP_TRACK_DEFAULT_RECOVER_MS,
                          APP_TRACK_DEFAULT_LEFT_TRIM,
                          APP_TRACK_DEFAULT_RIGHT_TRIM);
        g_track_preset = TRACK_PRESET_NORMAL;
        return 1U;

    case TRACK_PRESET_ADV:
        Track_ApplyTuning(APP_TRACK_ADV_BASE_DUTY,
                          APP_TRACK_ADV_KP,
                          APP_TRACK_ADV_KD,
                          APP_TRACK_ADV_CENTER_BIAS,
                          APP_TRACK_ADV_CORNER_ADVANCE_MS,
                          APP_TRACK_ADV_RECOVER_MS,
                          APP_TRACK_ADV_LEFT_TRIM,
                          APP_TRACK_ADV_RIGHT_TRIM);
        g_track_preset = TRACK_PRESET_ADV;
        return 1U;

    default:
        return 0U;
    }
}

Track_Preset_t Track_GetPreset(void)
{
    return g_track_preset;
}

const char *Track_GetPresetName(void)
{
    switch (g_track_preset)
    {
    case TRACK_PRESET_NORMAL:
        return "NORMAL";
    case TRACK_PRESET_ADV:
        return "ADV";
    case TRACK_PRESET_CUSTOM:
        return "CUSTOM";
    default:
        return "UNKNOWN";
    }
}

uint8_t Track_SetParam(const char *name, float value)
{
    if (name == (const char *)0)
    {
        return 0U;
    }

    if (strcmp(name, "BASE") == 0)
    {
        g_track_base_duty = Track_Clamp(value, 0.0f, 100.0f); /* duty 物理上限就是 100 */
        g_track_preset = TRACK_PRESET_CUSTOM;
        return 1U;
    }

    if (strcmp(name, "KP") == 0)
    {
        g_track_kp = Track_Clamp(value, 0.0f, APP_TRACK_PARAM_GAIN_MAX); /* 越大，偏线时修得越猛 */
        Track_ApplyPidParams();
        g_track_preset = TRACK_PRESET_CUSTOM;
        return 1U;
    }

    if (strcmp(name, "KD") == 0)
    {
        g_track_kd = Track_Clamp(value, 0.0f, APP_TRACK_PARAM_GAIN_MAX); /* 越大，越抑制来回摆动 */
        Track_ApplyPidParams();
        g_track_preset = TRACK_PRESET_CUSTOM;
        return 1U;
    }

    if ((strcmp(name, "CENTER_BIAS") == 0) || (strcmp(name, "BIAS") == 0))
    {
        g_track_center_bias = Track_Clamp(value,
                                          -APP_TRACK_PARAM_BIAS_LIMIT,
                                          APP_TRACK_PARAM_BIAS_LIMIT);
        g_track_preset = TRACK_PRESET_CUSTOM;
        return 1U;
    }

    if (strcmp(name, "CORNER_ADVANCE_MS") == 0)
    {
        g_track_corner_advance_ms = (uint32_t)Track_Clamp(value, 0.0f, APP_TRACK_PARAM_TIME_MS_MAX);
        g_track_preset = TRACK_PRESET_CUSTOM;
        return 1U;
    }

    if (strcmp(name, "RECOVER_MS") == 0)
    {
        g_track_recover_ms = (uint32_t)Track_Clamp(value, 0.0f, APP_TRACK_PARAM_TIME_MS_MAX); /* 大了出弯更稳，小了更快回循迹 */
        g_track_preset = TRACK_PRESET_CUSTOM;
        return 1U;
    }

    if ((strcmp(name, "LAPS") == 0) || (strcmp(name, "TARGET_LAPS") == 0) || (strcmp(name, "N") == 0))
    {
        return Track_SetTargetLaps((uint8_t)Track_Clamp(value, APP_TRACK_MIN_TARGET_LAPS, APP_TRACK_MAX_TARGET_LAPS));
    }

    if ((strcmp(name, "LEFT_TRIM") == 0) || (strcmp(name, "L_TRIM") == 0))
    {
        g_track_left_trim = Track_Clamp(value, 0.0f, APP_TRACK_PARAM_TRIM_MAX);
        g_track_preset = TRACK_PRESET_CUSTOM;
        return 1U;
    }

    if ((strcmp(name, "RIGHT_TRIM") == 0) || (strcmp(name, "R_TRIM") == 0))
    {
        g_track_right_trim = Track_Clamp(value, 0.0f, APP_TRACK_PARAM_TRIM_MAX);
        g_track_preset = TRACK_PRESET_CUSTOM;
        return 1U;
    }

    return 0U;
}

uint8_t Track_GetParam(const char *name, float *value)
{
    if ((name == (const char *)0) || (value == (float *)0))
    {
        return 0U;
    }

    if (strcmp(name, "BASE") == 0)
    {
        *value = g_track_base_duty;
        return 1U;
    }

    if (strcmp(name, "KP") == 0)
    {
        *value = g_track_kp;
        return 1U;
    }

    if (strcmp(name, "KD") == 0)
    {
        *value = g_track_kd;
        return 1U;
    }

    if ((strcmp(name, "CENTER_BIAS") == 0) || (strcmp(name, "BIAS") == 0))
    {
        *value = g_track_center_bias;
        return 1U;
    }

    if (strcmp(name, "CORNER_ADVANCE_MS") == 0)
    {
        *value = (float)g_track_corner_advance_ms;
        return 1U;
    }

    if (strcmp(name, "RECOVER_MS") == 0)
    {
        *value = (float)g_track_recover_ms;
        return 1U;
    }

    if ((strcmp(name, "LAPS") == 0) || (strcmp(name, "TARGET_LAPS") == 0) || (strcmp(name, "N") == 0))
    {
        *value = (float)g_track_target_laps;
        return 1U;
    }

    if ((strcmp(name, "LEFT_TRIM") == 0) || (strcmp(name, "L_TRIM") == 0))
    {
        *value = g_track_left_trim;
        return 1U;
    }

    if ((strcmp(name, "RIGHT_TRIM") == 0) || (strcmp(name, "R_TRIM") == 0))
    {
        *value = g_track_right_trim;
        return 1U;
    }

    return 0U;
}

void Track_FormatStatus(char *buffer, size_t buffer_size, const char *prefix)
{
    uint32_t now_ms;
    uint32_t state_elapsed_ms;
    uint32_t advance_left_ms = 0U;

    if ((buffer == (char *)0) || (buffer_size == 0U))
    {
        return;
    }

    if (prefix == (const char *)0)
    {
        prefix = "OK";
    }

    now_ms = HAL_GetTick();
    state_elapsed_ms = (uint32_t)(now_ms - g_track_state_enter_ms);
    if (g_track_state == TRACK_STATE_CORNER_ADVANCE)
    {
        uint32_t advance_elapsed_ms;

        advance_elapsed_ms = (uint32_t)(now_ms - g_track_corner_advance_start_ms);
        if (advance_elapsed_ms < g_track_corner_advance_ms)
        {
            advance_left_ms = (uint32_t)(g_track_corner_advance_ms - advance_elapsed_ms);
        }
    }

    (void)snprintf(buffer,
                   buffer_size,
                   "%s TRACK state=%s preset=%s laps=%u/%u corners=%u elapsed=%lu state_ms=%lu raw=0x%02X norm=%.3f corner=%d stop=%s base=%.1f kp=%.1f kd=%.1f center_bias=%.2f corner_advance_ms=%lu advance_left_ms=%lu recover_ms=%lu trim=%.3f/%.3f corr=%.1f line=%.1f",
                   prefix,
                   Track_GetStateName(),
                   Track_GetPresetName(),
                   (unsigned int)g_track_laps_done,
                   (unsigned int)g_track_target_laps,
                   (unsigned int)g_track_corner_count,
                   (unsigned long)Track_GetElapsedMs(),
                   (unsigned long)state_elapsed_ms,
                   (unsigned int)AppSensor_GetRawState(),
                   AppSensor_GetNormError(),
                   (int)Track_FilterCornerDirection(AppSensor_GetCornerDirection()),
                   Track_GetStopReasonName(),
                   g_track_base_duty,
                   g_track_kp,
                   g_track_kd,
                   g_track_center_bias,
                   (unsigned long)g_track_corner_advance_ms,
                   (unsigned long)advance_left_ms,
                   (unsigned long)g_track_recover_ms,
                   g_track_left_trim,
                   g_track_right_trim,
                   g_track_last_correction,
                   g_track_last_line_correction);
}

static void Track_EnterState(Track_State_t next)
{
    g_track_state = next;
    g_track_state_enter_ms = HAL_GetTick();
    (void)next;
}

static void Track_ResetProgress(void)
{
    g_track_corner_count = 0U;
    g_track_laps_done = 0U;
    g_track_corner_dir = 0;
    g_track_corner_advance_start_ms = 0U;
    g_track_recover_start_ms = 0U;
    g_track_last_correction = 0.0f;
    g_track_last_line_correction = 0.0f;
}

static void Track_StopWithReason(Track_State_t stop_state, Track_StopReason_t reason)
{
    Turn_Stop();
    Motion_Stop();
    PID_Core_Reset(&g_track_pid);
    g_track_corner_dir = 0;
    g_track_corner_advance_start_ms = 0U;
    g_track_recover_start_ms = 0U;
    g_track_last_correction = 0.0f;
    g_track_last_line_correction = 0.0f;
    g_track_stop_reason = reason;
    if (g_track_start_ms != 0U)
    {
        g_track_stop_ms = HAL_GetTick();
    }
    Track_EnterState(stop_state);
}

static void Track_RecordCompletedCorner(void)
{
    if (g_track_corner_count < 255U)
    {
        g_track_corner_count++;
    }

    g_track_laps_done = (uint8_t)(g_track_corner_count / APP_TRACK_CORNERS_PER_LAP);
}

static uint8_t Track_IsTargetComplete(void)
{
    return (uint8_t)(g_track_corner_count >=
                     (uint8_t)(g_track_target_laps * APP_TRACK_CORNERS_PER_LAP));
}

static void Track_ApplyPidParams(void)
{
    g_track_pid.params.kp = g_track_kp;
    g_track_pid.params.kd = g_track_kd;
}

static void Track_ApplyTuning(float base_duty,
                              float kp,
                              float kd,
                              float center_bias,
                              uint32_t corner_advance_ms,
                              uint32_t recover_ms,
                              float left_trim,
                              float right_trim)
{
    g_track_base_duty = Track_Clamp(base_duty, 0.0f, 100.0f);
    g_track_kp = Track_Clamp(kp, 0.0f, APP_TRACK_PARAM_GAIN_MAX);
    g_track_kd = Track_Clamp(kd, 0.0f, APP_TRACK_PARAM_GAIN_MAX);
    g_track_center_bias = Track_Clamp(center_bias,
                                      -APP_TRACK_PARAM_BIAS_LIMIT,
                                      APP_TRACK_PARAM_BIAS_LIMIT);
    g_track_corner_advance_ms = (uint32_t)Track_Clamp((float)corner_advance_ms,
                                                      0.0f,
                                                      APP_TRACK_PARAM_TIME_MS_MAX);
    g_track_recover_ms = (uint32_t)Track_Clamp((float)recover_ms,
                                               0.0f,
                                               APP_TRACK_PARAM_TIME_MS_MAX);
    g_track_left_trim = Track_Clamp(left_trim, 0.0f, APP_TRACK_PARAM_TRIM_MAX);
    g_track_right_trim = Track_Clamp(right_trim, 0.0f, APP_TRACK_PARAM_TRIM_MAX);
    if (g_track_initialized != 0U)
    {
        Track_ApplyPidParams();
    }
}

static float Track_Clamp(float value, float min_value, float max_value)
{
    if (value != value)
    {
        return min_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static void Track_ApplyFollowControl(void)
{
    float line_correction;
    float correction;
    float left_duty;
    float right_duty;

    if (AppSensor_IsAllWhite() != 0U)
    {
        line_correction = g_track_last_line_correction;
    }
    else
    {
        line_correction = PID_Core_Calculate(&g_track_pid, 0.0f, AppSensor_GetNormError(), 0.0f);
        g_track_last_line_correction = line_correction;
    }

    correction = Track_Clamp(line_correction + g_track_center_bias, -100.0f, 100.0f);
    g_track_last_correction = correction;

    left_duty = Track_Clamp(g_track_base_duty - correction, -100.0f, 100.0f);
    right_duty = Track_Clamp(g_track_base_duty + correction, -100.0f, 100.0f);
    Track_SetAutoDuty(left_duty, right_duty);
}

static void Track_SetAutoDuty(float left_duty, float right_duty)
{
    float trimmed_left;
    float trimmed_right;

    trimmed_left = Track_Clamp(left_duty * g_track_left_trim, -100.0f, 100.0f);
    trimmed_right = Track_Clamp(right_duty * g_track_right_trim, -100.0f, 100.0f);
    Motion_SetDuty4(trimmed_left, trimmed_right, trimmed_left, trimmed_right);
}

static uint8_t Track_StartLatchedTurn(void)
{
    if (Turn_Start(g_track_corner_dir) != 0U)
    {
        g_track_corner_advance_start_ms = 0U;
        Track_EnterState(TRACK_STATE_TURNING);
        return 1U;
    }

    Track_StopWithReason(TRACK_STATE_STOPPED, TRACK_STOP_REASON_TURN_START_FAIL);
    return 0U;
}

static int8_t Track_FilterCornerDirection(int8_t detected_dir)
{
    if (detected_dir == APP_TRACK_DEFAULT_CORNER_DIR)
    {
        return APP_TRACK_DEFAULT_CORNER_DIR;
    }

    return 0;
}
