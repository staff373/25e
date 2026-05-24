#include "app_aim.h"

#include "app_gimbal.h"
#include "app_vision.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <string.h>

#define AIM_DEFAULT_ONCE_TIMEOUT_MS          (2000U)
#define AIM_TRACK_INTERVAL_MS                (50U)
#define AIM_FILTER_WINDOW                    (3U)
#define AIM_TARGET_MAX_AGE_MS                (120U)
#define AIM_LOCK_TOLERANCE_PX                (2)
#define AIM_LOCK_CONFIRM_FRAMES              (3U)
#define AIM_LOCK_EXIT_TOLERANCE_PX           (4)
#define AIM_LOCK_EXIT_CONFIRM_FRAMES         (2U)
#define AIM_LOCK_VERIFY_SETTLE_MS            (80U)
#define AIM_TRACK_LOST_ERROR_MS              (800U)
#define AIM_Q3_DEFAULT_TIMEOUT_MS            (4000U)
#define AIM_Q3_SCAN_REV_STEPS                (2400L)
#define AIM_Q3_SCAN_SPEED_SPS                (1000.0f)
#define AIM_Q3_TARGET_CONFIRM_FRAMES         (3U)
#define AIM_Q3_SCAN_SETTLE_MS                (100U)
#define AIM_Q3_STABLE_CONFIRM_FRAMES         (3U)
#define AIM_Q3_STABLE_CONFIRM_TIMEOUT_MS     (300U)
#define AIM_Q3_CONFIRM_MISS_LIMIT            (3U)
/* ADV 直线弱预测：只在收到 TRACK_STATE_LINE_FOLLOW hint 时启用，不影响 TASK 2/3 静态瞄准。 */
#define AIM_ADV_LINE_PRED_X_DEFAULT          (1.0f) /* X 方向按最近 dx 变化提前追一点；直线来回晃时先降到 0.15~0.25 */
#define AIM_ADV_LINE_PRED_Y_DEFAULT          (0.15f) /* Y 方向先弱一些；若上下明显滞后再小步加，静态晃动则减小 */

/* ADV 右转前馈使用“步数”而不是像素，目标是车身右转时云台真实补回一段 yaw。 */
#define AIM_ADV_TURN_PREFEED_X_DEFAULT       (320.0f) /* CORNER_ADVANCE 阶段先打一段 X 步数；目标刚进弯就丢时优先调它 */
#define AIM_ADV_TURN_PREFEED_Y_DEFAULT       (0.0f)   /* 暂不默认补 Y；若转弯时目标主要上下甩，再按 AIM? dy 方向小步调 */
#define AIM_ADV_TURN_FF_X_PER_DEG_DEFAULT    (7.5f)   /* TURNING 中每转 1 度补多少 X 步；2400 步/圈时 90 度约 600 步 */
#define AIM_ADV_TURN_FF_Y_PER_DEG_DEFAULT    (0.0f)   /* TURNING 中每转 1 度补多少 Y 步，默认关闭，确认 Y 方向后再开 */
#define AIM_ADV_TURN_FF_GYRO_X_DEFAULT       (0.40f)  /* 角速度补偿：gyro_z * dt * 该系数 = 额外 X 步 */
#define AIM_ADV_TURN_FF_GYRO_Y_DEFAULT       (0.0f)   /* 角速度型 Y 步数前馈，默认关闭 */
#define AIM_ADV_TURN_FF_MAX_STEP_DEFAULT     (300.0f) /* 单次前馈步数限幅；方向未确认前不要放太大 */
#define AIM_ADV_TURN_FF_SPEED_SPS_DEFAULT    (4000.0f) /* 转弯前馈步进速度；太低来不及补，太高可能丢步或抖 */
#define AIM_PARAM_PRED_LIMIT                 (3.0f)
#define AIM_PARAM_FF_VALUE_LIMIT             (1000.0f)
#define AIM_PARAM_FF_GAIN_LIMIT              (20.0f)
#define AIM_PARAM_FF_SPEED_MAX               (4000.0f)

typedef enum
{
    AIM_ERR_NONE = 0,
    AIM_ERR_NO_TARGET,
    AIM_ERR_TIMEOUT,
    AIM_ERR_GIMBAL,
    AIM_ERR_CAL
} Aim_Error_t;

typedef enum
{
    AIM_LOCK_RESULT_WAIT = 0,
    AIM_LOCK_RESULT_CONFIRMED,
    AIM_LOCK_RESULT_RETRY
} Aim_LockResult_t;

typedef enum
{
    AIM_TRACK_PHASE_IDLE = 0,
    AIM_TRACK_PHASE_ACQUIRE,
    AIM_TRACK_PHASE_LINE_TRACK,
    AIM_TRACK_PHASE_LINE_PREDICT,
    AIM_TRACK_PHASE_TURN_PREFEED,
    AIM_TRACK_PHASE_TURN_FEED,
    AIM_TRACK_PHASE_RECOVER_AIM,
    AIM_TRACK_PHASE_HOLD,
    AIM_TRACK_PHASE_CORRECT,
    AIM_TRACK_PHASE_MOVING,
    AIM_TRACK_PHASE_MISS_HOLD,
    AIM_TRACK_PHASE_ERROR
} Aim_TrackPhase_t;

typedef struct
{
    uint16_t seq;
    int16_t dx;
    int16_t dy;
    uint32_t age_ms;
} Aim_TargetSample_t;

typedef struct
{
    uint16_t seq;
    int16_t dx;
    int16_t dy;
    uint32_t age_ms;
} Aim_FilteredTarget_t;

static Aim_State_t g_aim_state = AIM_STATE_IDLE;
static Aim_Error_t g_aim_last_error = AIM_ERR_NONE;
static uint32_t g_aim_state_enter_ms = 0U;
static uint32_t g_aim_last_step_ms = 0U;
static uint32_t g_aim_once_timeout_ms = AIM_DEFAULT_ONCE_TIMEOUT_MS;
static uint32_t g_aim_once_start_ms = 0U;
static uint8_t g_aim_has_last_seq = 0U;
static uint16_t g_aim_last_seq = 0U;
static Aim_TargetSample_t g_aim_filter_samples[AIM_FILTER_WINDOW];
static uint8_t g_aim_filter_count = 0U;
static uint8_t g_aim_filter_next = 0U;
static uint8_t g_aim_filter_valid = 0U;
static Aim_FilteredTarget_t g_aim_filtered_target;
static uint8_t g_aim_lock_frames = 0U;
static uint8_t g_aim_lock_exit_frames = 0U;
static float g_aim_last_gain = 0.0f;
static uint32_t g_aim_q3_timeout_ms = AIM_Q3_DEFAULT_TIMEOUT_MS;
static uint32_t g_aim_q3_start_ms = 0U;
static uint32_t g_aim_q3_settle_start_ms = 0U;
static uint8_t g_aim_q3_scan_started = 0U;
static uint8_t g_aim_q3_seen_frames = 0U;
static uint8_t g_aim_q3_stable_frames = 0U;
static uint8_t g_aim_q3_miss_frames = 0U;
static Aim_TrackPhase_t g_aim_track_phase = AIM_TRACK_PHASE_IDLE;
static uint32_t g_aim_track_miss_start_ms = 0U;
static uint32_t g_aim_track_move_count = 0U;
static int16_t g_aim_last_cmd_dx = 0;
static int16_t g_aim_last_cmd_dy = 0;
static Track_State_t g_aim_hint_track_state = TRACK_STATE_IDLE;
static int8_t g_aim_hint_turn_dir = 0;
static float g_aim_hint_turn_progress_deg = 0.0f;
static float g_aim_hint_turn_gyro_z_dps = 0.0f;
static uint8_t g_aim_turn_prefeed_done = 0U;
static uint8_t g_aim_recover_filter_cleared = 0U;
static float g_aim_turn_last_feed_progress_deg = 0.0f;
static uint32_t g_aim_turn_last_feed_ms = 0U;
static uint8_t g_aim_has_last_raw_target = 0U;
static int16_t g_aim_last_raw_dx = 0;
static int16_t g_aim_last_raw_dy = 0;
static int16_t g_aim_velocity_dx = 0;
static int16_t g_aim_velocity_dy = 0;
static int16_t g_aim_last_ff_dx = 0;
static int16_t g_aim_last_ff_dy = 0;
static float g_aim_line_pred_x = AIM_ADV_LINE_PRED_X_DEFAULT;
static float g_aim_line_pred_y = AIM_ADV_LINE_PRED_Y_DEFAULT;
static float g_aim_turn_prefeed_x = AIM_ADV_TURN_PREFEED_X_DEFAULT;
static float g_aim_turn_prefeed_y = AIM_ADV_TURN_PREFEED_Y_DEFAULT;
static float g_aim_turn_ff_x_per_deg = AIM_ADV_TURN_FF_X_PER_DEG_DEFAULT;
static float g_aim_turn_ff_y_per_deg = AIM_ADV_TURN_FF_Y_PER_DEG_DEFAULT;
static float g_aim_turn_ff_gyro_x = AIM_ADV_TURN_FF_GYRO_X_DEFAULT;
static float g_aim_turn_ff_gyro_y = AIM_ADV_TURN_FF_GYRO_Y_DEFAULT;
static float g_aim_turn_ff_max_step = AIM_ADV_TURN_FF_MAX_STEP_DEFAULT;
static float g_aim_turn_ff_speed_sps = AIM_ADV_TURN_FF_SPEED_SPS_DEFAULT;

static void Aim_EnterState(Aim_State_t next);
static uint32_t Aim_StateElapsedMs(void);
static int16_t Aim_AbsInt16(int16_t value);
static float Aim_AbsFloat(float value);
static float Aim_ClampFloat(float value, float min_value, float max_value);
static int16_t Aim_RoundClampInt16(float value);
static int16_t Aim_ClampCommandFloat(float value);
static int16_t Aim_Median3Int16(int16_t a, int16_t b, int16_t c);
static int16_t Aim_ScaleInt16(int16_t value, float gain);
static uint8_t Aim_ErrorWithinTolerance(int16_t dx, int16_t dy, int16_t tolerance);
static uint8_t Aim_FilterReady(void);
static void Aim_FilterClear(void);
static void Aim_FilterResetSeq(void);
static void Aim_FilterPush(const Vision_Target_t *target);
static void Aim_FilterRecompute(void);
static uint8_t Aim_ReadVisionFrame(Vision_Target_t *target, uint8_t *has_target, uint8_t *new_frame);
static void Aim_UpdateVisionVelocity(const Vision_Target_t *target, uint8_t new_frame);
static float Aim_SelectGain(int16_t dx, int16_t dy);
static void Aim_BuildScaledCommand(const Aim_FilteredTarget_t *target, int16_t *cmd_dx, int16_t *cmd_dy);
static uint8_t Aim_MoveByPixelCommand(int16_t cmd_dx, int16_t cmd_dy);
static uint8_t Aim_MoveByStepCommand(int16_t step_x, int16_t step_y);
static uint8_t Aim_MoveByFilteredError(const Aim_FilteredTarget_t *target);
static void Aim_BuildLinePredictedTarget(const Aim_FilteredTarget_t *source, Aim_FilteredTarget_t *target);
static uint8_t Aim_HandleTurnPrefeed(uint32_t now_ms);
static uint8_t Aim_HandleTurnFeed(uint32_t now_ms, uint8_t has_target, uint8_t new_frame);
static void Aim_BuildTurnFeedCommand(uint32_t now_ms, int16_t *ff_dx, int16_t *ff_dy);
static void Aim_ResetDynamicTrack(void);
static uint8_t Aim_TrackHintIsTurnPhase(void);
static uint8_t Aim_FilteredWithinTolerance(const Aim_FilteredTarget_t *target, int16_t tolerance);
static uint8_t Aim_RawWithinTolerance(const Vision_Target_t *target, int16_t tolerance);
static void Aim_ResetLockCounters(void);
static Aim_LockResult_t Aim_UpdateLockConfirm(const Vision_Target_t *raw_target,
                                              const Aim_FilteredTarget_t *filtered_target,
                                              uint8_t new_frame);
static void Aim_BeginLockVerify(Aim_State_t next_state);
static uint8_t Aim_OnceTimedOut(void);
static uint8_t Aim_RunClosedLoop(uint8_t use_timeout, uint32_t timeout_ms, uint8_t use_track_interval);
static uint8_t Aim_Q3TimedOut(void);
static void Aim_Q3Reset(void);
static void Aim_Q3RestartScan(void);
static void Aim_RunOnce(void);
static void Aim_RunTrack(void);
static void Aim_RunLockVerify(void);
static void Aim_RunQuestion3ScanRev(void);
static void Aim_RunQuestion3ScanSettle(void);
static void Aim_RunQuestion3StableConfirm(void);
static void Aim_RunQuestion3Aim(void);
static void Aim_RunQuestion3LockVerify(void);
static const char *Aim_GetErrorName(Aim_Error_t error);
static void Aim_TrackEnterPhase(Aim_TrackPhase_t phase);
static const char *Aim_GetTrackPhaseName(Aim_TrackPhase_t phase);

void Aim_Init(void)
{
    g_aim_last_error = AIM_ERR_NONE;
    g_aim_last_step_ms = 0U;
    g_aim_once_timeout_ms = AIM_DEFAULT_ONCE_TIMEOUT_MS;
    g_aim_once_start_ms = 0U;
    g_aim_q3_timeout_ms = AIM_Q3_DEFAULT_TIMEOUT_MS;
    Aim_FilterClear();
    Aim_FilterResetSeq();
    Aim_ResetLockCounters();
    Aim_Q3Reset();
    Aim_ResetDynamicTrack();
    Aim_TrackEnterPhase(AIM_TRACK_PHASE_IDLE);
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
    case AIM_STATE_LOCK_VERIFY:
        Aim_RunLockVerify();
        break;
    case AIM_STATE_Q3_SCAN_REV:
        Aim_RunQuestion3ScanRev();
        break;
    case AIM_STATE_Q3_SCAN_SETTLE:
        Aim_RunQuestion3ScanSettle();
        break;
    case AIM_STATE_Q3_STABLE_CONFIRM:
        Aim_RunQuestion3StableConfirm();
        break;
    case AIM_STATE_Q3_AIM:
        Aim_RunQuestion3Aim();
        break;
    case AIM_STATE_Q3_LOCK_VERIFY:
        Aim_RunQuestion3LockVerify();
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
    g_aim_once_start_ms = HAL_GetTick();
    g_aim_last_error = AIM_ERR_NONE;
    g_aim_last_step_ms = 0U;
    Aim_FilterClear();
    Aim_FilterResetSeq();
    Aim_ResetLockCounters();
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
    g_aim_last_cmd_dx = 0;
    g_aim_last_cmd_dy = 0;
    Aim_FilterClear();
    Aim_FilterResetSeq();
    Aim_ResetLockCounters();
    Aim_ResetDynamicTrack();
    g_aim_track_miss_start_ms = 0U;
    g_aim_track_move_count = 0U;
    Aim_TrackEnterPhase(AIM_TRACK_PHASE_ACQUIRE);
    Aim_EnterState(AIM_STATE_TRACKING);
    return 1U;
}

uint8_t Aim_StartQuestion3(uint32_t timeout_ms)
{
    if (Gimbal_IsCalibrated() == 0U)
    {
        g_aim_last_error = AIM_ERR_CAL;
        Aim_EnterState(AIM_STATE_ERROR);
        return 0U;
    }

    g_aim_q3_timeout_ms = (timeout_ms == 0U) ? AIM_Q3_DEFAULT_TIMEOUT_MS : timeout_ms;
    g_aim_last_error = AIM_ERR_NONE;
    g_aim_last_step_ms = 0U;
    Aim_FilterClear();
    Aim_FilterResetSeq();
    Aim_ResetLockCounters();
    Aim_Q3Reset();
    g_aim_q3_start_ms = HAL_GetTick();
    Aim_EnterState(AIM_STATE_Q3_SCAN_REV);
    return 1U;
}

void Aim_Stop(void)
{
    Gimbal_Stop();
    Aim_FilterClear();
    Aim_ResetLockCounters();
    Aim_ResetDynamicTrack();
    Aim_TrackEnterPhase(AIM_TRACK_PHASE_IDLE);
    Aim_EnterState(AIM_STATE_IDLE);
}

void Aim_UpdateTrackHint(Track_State_t track_state,
                         int8_t turn_dir,
                         float turn_progress_deg,
                         float turn_gyro_z_dps)
{
    Track_State_t previous_state = g_aim_hint_track_state;

    g_aim_hint_track_state = track_state;
    g_aim_hint_turn_dir = turn_dir;
    g_aim_hint_turn_progress_deg = turn_progress_deg;
    g_aim_hint_turn_gyro_z_dps = turn_gyro_z_dps;

    if (previous_state == track_state)
    {
        return;
    }

    if (track_state == TRACK_STATE_CORNER_ADVANCE)
    {
        g_aim_turn_prefeed_done = 0U;
        g_aim_recover_filter_cleared = 0U;
    }
    else if (track_state == TRACK_STATE_TURNING)
    {
        g_aim_turn_last_feed_progress_deg = turn_progress_deg;
        g_aim_turn_last_feed_ms = HAL_GetTick();
        g_aim_recover_filter_cleared = 0U;
    }
    else if (track_state == TRACK_STATE_RECOVER_LINE)
    {
        g_aim_last_ff_dx = 0;
        g_aim_last_ff_dy = 0;
        g_aim_turn_prefeed_done = 0U;
    }
    else if (track_state == TRACK_STATE_LINE_FOLLOW)
    {
        g_aim_turn_prefeed_done = 0U;
        g_aim_recover_filter_cleared = 0U;
        g_aim_turn_last_feed_progress_deg = 0.0f;
        g_aim_turn_last_feed_ms = 0U;
        g_aim_last_ff_dx = 0;
        g_aim_last_ff_dy = 0;
    }
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
    case AIM_STATE_LOCK_VERIFY:
        return "LOCK_VERIFY";
    case AIM_STATE_Q3_SCAN_REV:
        return "Q3_SCAN_REV";
    case AIM_STATE_Q3_SCAN_SETTLE:
        return "Q3_SCAN_SETTLE";
    case AIM_STATE_Q3_STABLE_CONFIRM:
        return "Q3_STABLE_CONFIRM";
    case AIM_STATE_Q3_AIM:
        return "Q3_AIM";
    case AIM_STATE_Q3_LOCK_VERIFY:
        return "Q3_LOCK_VERIFY";
    case AIM_STATE_LOCKED:
        return "LOCKED";
    case AIM_STATE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

uint8_t Aim_SetParam(const char *name, float value)
{
    if (name == (const char *)0)
    {
        return 0U;
    }

    if (strcmp(name, "AIM_LINE_PRED_X") == 0)
    {
        g_aim_line_pred_x = Aim_ClampFloat(value, -AIM_PARAM_PRED_LIMIT, AIM_PARAM_PRED_LIMIT);
        return 1U;
    }

    if (strcmp(name, "AIM_LINE_PRED_Y") == 0)
    {
        g_aim_line_pred_y = Aim_ClampFloat(value, -AIM_PARAM_PRED_LIMIT, AIM_PARAM_PRED_LIMIT);
        return 1U;
    }

    if (strcmp(name, "AIM_TURN_PREFEED_X") == 0)
    {
        g_aim_turn_prefeed_x = Aim_ClampFloat(value, -AIM_PARAM_FF_VALUE_LIMIT, AIM_PARAM_FF_VALUE_LIMIT);
        return 1U;
    }

    if (strcmp(name, "AIM_TURN_PREFEED_Y") == 0)
    {
        g_aim_turn_prefeed_y = Aim_ClampFloat(value, -AIM_PARAM_FF_VALUE_LIMIT, AIM_PARAM_FF_VALUE_LIMIT);
        return 1U;
    }

    if (strcmp(name, "AIM_TURN_FF_X_PER_DEG") == 0)
    {
        g_aim_turn_ff_x_per_deg = Aim_ClampFloat(value, -AIM_PARAM_FF_GAIN_LIMIT, AIM_PARAM_FF_GAIN_LIMIT);
        return 1U;
    }

    if (strcmp(name, "AIM_TURN_FF_Y_PER_DEG") == 0)
    {
        g_aim_turn_ff_y_per_deg = Aim_ClampFloat(value, -AIM_PARAM_FF_GAIN_LIMIT, AIM_PARAM_FF_GAIN_LIMIT);
        return 1U;
    }

    if (strcmp(name, "AIM_TURN_FF_GYRO_X") == 0)
    {
        g_aim_turn_ff_gyro_x = Aim_ClampFloat(value, -AIM_PARAM_FF_GAIN_LIMIT, AIM_PARAM_FF_GAIN_LIMIT);
        return 1U;
    }

    if (strcmp(name, "AIM_TURN_FF_GYRO_Y") == 0)
    {
        g_aim_turn_ff_gyro_y = Aim_ClampFloat(value, -AIM_PARAM_FF_GAIN_LIMIT, AIM_PARAM_FF_GAIN_LIMIT);
        return 1U;
    }

    if (strcmp(name, "AIM_TURN_FF_MAX_STEP") == 0)
    {
        g_aim_turn_ff_max_step = Aim_ClampFloat(value, 0.0f, AIM_PARAM_FF_VALUE_LIMIT);
        return 1U;
    }

    if (strcmp(name, "AIM_TURN_FF_SPEED_SPS") == 0)
    {
        g_aim_turn_ff_speed_sps = Aim_ClampFloat(value, 1.0f, AIM_PARAM_FF_SPEED_MAX);
        return 1U;
    }

    return 0U;
}

uint8_t Aim_GetParam(const char *name, float *value)
{
    if ((name == (const char *)0) || (value == (float *)0))
    {
        return 0U;
    }

    if (strcmp(name, "AIM_LINE_PRED_X") == 0)
    {
        *value = g_aim_line_pred_x;
        return 1U;
    }

    if (strcmp(name, "AIM_LINE_PRED_Y") == 0)
    {
        *value = g_aim_line_pred_y;
        return 1U;
    }

    if (strcmp(name, "AIM_TURN_PREFEED_X") == 0)
    {
        *value = g_aim_turn_prefeed_x;
        return 1U;
    }

    if (strcmp(name, "AIM_TURN_PREFEED_Y") == 0)
    {
        *value = g_aim_turn_prefeed_y;
        return 1U;
    }

    if (strcmp(name, "AIM_TURN_FF_X_PER_DEG") == 0)
    {
        *value = g_aim_turn_ff_x_per_deg;
        return 1U;
    }

    if (strcmp(name, "AIM_TURN_FF_Y_PER_DEG") == 0)
    {
        *value = g_aim_turn_ff_y_per_deg;
        return 1U;
    }

    if (strcmp(name, "AIM_TURN_FF_GYRO_X") == 0)
    {
        *value = g_aim_turn_ff_gyro_x;
        return 1U;
    }

    if (strcmp(name, "AIM_TURN_FF_GYRO_Y") == 0)
    {
        *value = g_aim_turn_ff_gyro_y;
        return 1U;
    }

    if (strcmp(name, "AIM_TURN_FF_MAX_STEP") == 0)
    {
        *value = g_aim_turn_ff_max_step;
        return 1U;
    }

    if (strcmp(name, "AIM_TURN_FF_SPEED_SPS") == 0)
    {
        *value = g_aim_turn_ff_speed_sps;
        return 1U;
    }

    return 0U;
}

void Aim_FormatStatus(char *buffer, size_t buffer_size, const char *prefix)
{
    Vision_Target_t target;
    uint8_t has_target;
    uint8_t filter_ready;
    uint8_t raw_ok;
    uint8_t filter_ok;
    uint8_t lock_ok;
    uint32_t verify_ms;
    uint32_t miss_ms;

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

    filter_ready = Aim_FilterReady();
    raw_ok = (uint8_t)((has_target != 0U) &&
                       (Aim_ErrorWithinTolerance(target.dx, target.dy, AIM_LOCK_TOLERANCE_PX) != 0U));
    filter_ok = (uint8_t)((filter_ready != 0U) &&
                          (Aim_ErrorWithinTolerance(g_aim_filtered_target.dx,
                                                    g_aim_filtered_target.dy,
                                                    AIM_LOCK_TOLERANCE_PX) != 0U));
    lock_ok = (uint8_t)((raw_ok != 0U) && (filter_ok != 0U));
    verify_ms = ((g_aim_state == AIM_STATE_LOCK_VERIFY) ||
                 (g_aim_state == AIM_STATE_Q3_LOCK_VERIFY))
                    ? Aim_StateElapsedMs()
                    : 0U;
    miss_ms = (g_aim_track_miss_start_ms == 0U)
                  ? 0U
                  : (uint32_t)(HAL_GetTick() - g_aim_track_miss_start_ms);

    (void)snprintf(buffer,
                   buffer_size,
                   "%s AIM state=%s trk=%s err=%s elapsed=%lu target=%u dx=%d dy=%d f_valid=%u f_dx=%d f_dy=%d raw_ok=%u f_ok=%u lock_ok=%u fail=%u verify_ms=%lu miss_ms=%lu moves=%lu cmd_dx=%d cmd_dy=%d ff_step=%d,%d ff_sps=%.0f v=%d,%d hint=%d dir=%d prog=%.1f gyro=%.1f pred=%.2f,%.2f tol=%d exit_tol=%d gain=%.2f age=%lu q3_seen=%u q3_stable=%u q3_lock=%u q3_rev=%ld gimbal=%s cal=%u",
                   (prefix != (const char *)0) ? prefix : "OK",
                   Aim_GetStateName(),
                   Aim_GetTrackPhaseName(g_aim_track_phase),
                   Aim_GetErrorName(g_aim_last_error),
                   (unsigned long)Aim_StateElapsedMs(),
                   (unsigned int)has_target,
                   (int)target.dx,
                   (int)target.dy,
                   (unsigned int)filter_ready,
                   filter_ready != 0U ? (int)g_aim_filtered_target.dx : 0,
                   filter_ready != 0U ? (int)g_aim_filtered_target.dy : 0,
                   (unsigned int)raw_ok,
                   (unsigned int)filter_ok,
                   (unsigned int)lock_ok,
                   (unsigned int)g_aim_lock_exit_frames,
                   (unsigned long)verify_ms,
                   (unsigned long)miss_ms,
                   (unsigned long)g_aim_track_move_count,
                   (int)g_aim_last_cmd_dx,
                   (int)g_aim_last_cmd_dy,
                   (int)g_aim_last_ff_dx,
                   (int)g_aim_last_ff_dy,
                   (double)g_aim_turn_ff_speed_sps,
                   (int)g_aim_velocity_dx,
                   (int)g_aim_velocity_dy,
                   (int)g_aim_hint_track_state,
                   (int)g_aim_hint_turn_dir,
                   (double)g_aim_hint_turn_progress_deg,
                   (double)g_aim_hint_turn_gyro_z_dps,
                   (double)g_aim_line_pred_x,
                   (double)g_aim_line_pred_y,
                   (int)AIM_LOCK_TOLERANCE_PX,
                   (int)AIM_LOCK_EXIT_TOLERANCE_PX,
                   (double)g_aim_last_gain,
                   (unsigned long)target.age_ms,
                   (unsigned int)g_aim_q3_seen_frames,
                   (unsigned int)g_aim_q3_stable_frames,
                   (unsigned int)g_aim_lock_frames,
                   (long)AIM_Q3_SCAN_REV_STEPS,
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

static float Aim_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float Aim_ClampFloat(float value, float min_value, float max_value)
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

static int16_t Aim_RoundClampInt16(float value)
{
    int32_t rounded;

    if (value >= 0.0f)
    {
        rounded = (int32_t)(value + 0.5f);
    }
    else
    {
        rounded = (int32_t)(value - 0.5f);
    }

    if (rounded > 32767L)
    {
        return 32767;
    }

    if (rounded < -32768L)
    {
        return (int16_t)-32768;
    }

    return (int16_t)rounded;
}

static int16_t Aim_ClampCommandFloat(float value)
{
    float limit = g_aim_turn_ff_max_step;

    if (limit < 0.0f)
    {
        limit = -limit;
    }

    value = Aim_ClampFloat(value, -limit, limit);
    return Aim_RoundClampInt16(value);
}

static int16_t Aim_Median3Int16(int16_t a, int16_t b, int16_t c)
{
    if (((a <= b) && (b <= c)) || ((c <= b) && (b <= a)))
    {
        return b;
    }
    if (((b <= a) && (a <= c)) || ((c <= a) && (a <= b)))
    {
        return a;
    }
    return c;
}

static int16_t Aim_ScaleInt16(int16_t value, float gain)
{
    float scaled = (float)value * gain;
    int32_t rounded;

    if (scaled >= 0.0f)
    {
        rounded = (int32_t)(scaled + 0.5f);
    }
    else
    {
        rounded = (int32_t)(scaled - 0.5f);
    }

    if (rounded > 32767L)
    {
        return 32767;
    }
    if (rounded < -32768L)
    {
        return (int16_t)-32768;
    }
    return (int16_t)rounded;
}

static uint8_t Aim_ErrorWithinTolerance(int16_t dx, int16_t dy, int16_t tolerance)
{
    return (uint8_t)((Aim_AbsInt16(dx) <= tolerance) && (Aim_AbsInt16(dy) <= tolerance));
}

static uint8_t Aim_FilterReady(void)
{
    return (uint8_t)((g_aim_filter_valid != 0U) && (g_aim_filter_count >= AIM_FILTER_WINDOW));
}

static void Aim_FilterClear(void)
{
    g_aim_filter_count = 0U;
    g_aim_filter_next = 0U;
    g_aim_filter_valid = 0U;
    g_aim_filtered_target.seq = 0U;
    g_aim_filtered_target.dx = 0;
    g_aim_filtered_target.dy = 0;
    g_aim_filtered_target.age_ms = 0U;
}

static void Aim_FilterResetSeq(void)
{
    g_aim_has_last_seq = 0U;
    g_aim_last_seq = 0U;
}

static void Aim_FilterPush(const Vision_Target_t *target)
{
    if (target == (const Vision_Target_t *)0)
    {
        return;
    }

    g_aim_filter_samples[g_aim_filter_next].seq = target->seq;
    g_aim_filter_samples[g_aim_filter_next].dx = target->dx;
    g_aim_filter_samples[g_aim_filter_next].dy = target->dy;
    g_aim_filter_samples[g_aim_filter_next].age_ms = target->age_ms;

    g_aim_filter_next++;
    if (g_aim_filter_next >= AIM_FILTER_WINDOW)
    {
        g_aim_filter_next = 0U;
    }
    if (g_aim_filter_count < AIM_FILTER_WINDOW)
    {
        g_aim_filter_count++;
    }
    Aim_FilterRecompute();
}

static void Aim_FilterRecompute(void)
{
    int32_t sum;

    if (g_aim_filter_count == 0U)
    {
        g_aim_filter_valid = 0U;
        return;
    }

    if (g_aim_filter_count == 1U)
    {
        g_aim_filtered_target.dx = g_aim_filter_samples[0].dx;
        g_aim_filtered_target.dy = g_aim_filter_samples[0].dy;
        g_aim_filtered_target.seq = g_aim_filter_samples[0].seq;
        g_aim_filtered_target.age_ms = g_aim_filter_samples[0].age_ms;
    }
    else if (g_aim_filter_count == 2U)
    {
        sum = (int32_t)g_aim_filter_samples[0].dx + (int32_t)g_aim_filter_samples[1].dx;
        g_aim_filtered_target.dx = (int16_t)(sum / 2L);
        sum = (int32_t)g_aim_filter_samples[0].dy + (int32_t)g_aim_filter_samples[1].dy;
        g_aim_filtered_target.dy = (int16_t)(sum / 2L);
        g_aim_filtered_target.seq = g_aim_filter_samples[1].seq;
        g_aim_filtered_target.age_ms = g_aim_filter_samples[1].age_ms;
    }
    else
    {
        g_aim_filtered_target.dx = Aim_Median3Int16(g_aim_filter_samples[0].dx,
                                                    g_aim_filter_samples[1].dx,
                                                    g_aim_filter_samples[2].dx);
        g_aim_filtered_target.dy = Aim_Median3Int16(g_aim_filter_samples[0].dy,
                                                    g_aim_filter_samples[1].dy,
                                                    g_aim_filter_samples[2].dy);
        g_aim_filtered_target.seq = g_aim_filter_samples[(uint8_t)((g_aim_filter_next + AIM_FILTER_WINDOW - 1U) % AIM_FILTER_WINDOW)].seq;
        g_aim_filtered_target.age_ms = g_aim_filter_samples[(uint8_t)((g_aim_filter_next + AIM_FILTER_WINDOW - 1U) % AIM_FILTER_WINDOW)].age_ms;
    }

    g_aim_filter_valid = 1U;
}

static uint8_t Aim_ReadVisionFrame(Vision_Target_t *target, uint8_t *has_target, uint8_t *new_frame)
{
    uint8_t target_valid;

    if ((target == (Vision_Target_t *)0) ||
        (has_target == (uint8_t *)0) ||
        (new_frame == (uint8_t *)0))
    {
        return 0U;
    }

    target_valid = Vision_GetTarget(target);
    *has_target = target_valid;
    *new_frame = 0U;

    if ((Vision_IsOnline() == 0U) || (target->age_ms > AIM_TARGET_MAX_AGE_MS))
    {
        return 0U;
    }

    if ((g_aim_has_last_seq == 0U) || (target->seq != g_aim_last_seq))
    {
        *new_frame = 1U;
        g_aim_last_seq = target->seq;
        g_aim_has_last_seq = 1U;

        if (target_valid != 0U)
        {
            Aim_FilterPush(target);
        }
        else
        {
            Aim_FilterClear();
        }
    }

    return 1U;
}

static void Aim_UpdateVisionVelocity(const Vision_Target_t *target, uint8_t new_frame)
{
    int32_t dx_delta;
    int32_t dy_delta;

    if ((target == (const Vision_Target_t *)0) || (new_frame == 0U) || (target->valid == 0U))
    {
        return;
    }

    if (g_aim_has_last_raw_target == 0U)
    {
        g_aim_velocity_dx = 0;
        g_aim_velocity_dy = 0;
        g_aim_has_last_raw_target = 1U;
    }
    else
    {
        dx_delta = (int32_t)target->dx - (int32_t)g_aim_last_raw_dx;
        dy_delta = (int32_t)target->dy - (int32_t)g_aim_last_raw_dy;
        g_aim_velocity_dx = Aim_RoundClampInt16((float)dx_delta);
        g_aim_velocity_dy = Aim_RoundClampInt16((float)dy_delta);
    }

    g_aim_last_raw_dx = target->dx;
    g_aim_last_raw_dy = target->dy;
}

static float Aim_SelectGain(int16_t dx, int16_t dy)
{
    int16_t abs_dx = Aim_AbsInt16(dx);
    int16_t abs_dy = Aim_AbsInt16(dy);
    int16_t max_abs = (abs_dx > abs_dy) ? abs_dx : abs_dy;

    if (max_abs <= AIM_LOCK_TOLERANCE_PX)
    {
        return 0.0f;
    }
    if (max_abs > 40)
    {
        return 0.70f;
    }
    if (max_abs >= 12)
    {
        return 0.50f;
    }
    return 0.30f;
}

static void Aim_BuildScaledCommand(const Aim_FilteredTarget_t *target, int16_t *cmd_dx, int16_t *cmd_dy)
{
    int16_t scaled_dx;
    int16_t scaled_dy;

    if ((target == (const Aim_FilteredTarget_t *)0) ||
        (cmd_dx == (int16_t *)0) ||
        (cmd_dy == (int16_t *)0))
    {
        return;
    }

    g_aim_last_gain = Aim_SelectGain(target->dx, target->dy);
    if (g_aim_last_gain <= 0.0f)
    {
        *cmd_dx = 0;
        *cmd_dy = 0;
        return;
    }

    scaled_dx = Aim_ScaleInt16(target->dx, g_aim_last_gain);
    scaled_dy = Aim_ScaleInt16(target->dy, g_aim_last_gain);
    if ((scaled_dx == 0) && (target->dx != 0))
    {
        scaled_dx = (target->dx > 0) ? 1 : -1;
    }
    if ((scaled_dy == 0) && (target->dy != 0))
    {
        scaled_dy = (target->dy > 0) ? 1 : -1;
    }

    *cmd_dx = scaled_dx;
    *cmd_dy = scaled_dy;
}

static uint8_t Aim_MoveByPixelCommand(int16_t cmd_dx, int16_t cmd_dy)
{
    g_aim_last_cmd_dx = cmd_dx;
    g_aim_last_cmd_dy = cmd_dy;

    if ((cmd_dx == 0) && (cmd_dy == 0))
    {
        return 1U;
    }

    return Gimbal_MoveByPixelError(cmd_dx, cmd_dy);
}

static uint8_t Aim_MoveByStepCommand(int16_t step_x, int16_t step_y)
{
    g_aim_last_cmd_dx = step_x;
    g_aim_last_cmd_dy = step_y;

    if ((step_x == 0) && (step_y == 0))
    {
        return 1U;
    }

    return Gimbal_MoveRelativeSteps((int32_t)step_x,
                                    (int32_t)step_y,
                                    g_aim_turn_ff_speed_sps);
}

static uint8_t Aim_MoveByFilteredError(const Aim_FilteredTarget_t *target)
{
    int16_t cmd_dx = 0;
    int16_t cmd_dy = 0;

    Aim_BuildScaledCommand(target, &cmd_dx, &cmd_dy);
    return Aim_MoveByPixelCommand(cmd_dx, cmd_dy);
}

static void Aim_BuildLinePredictedTarget(const Aim_FilteredTarget_t *source, Aim_FilteredTarget_t *target)
{
    float predicted_dx;
    float predicted_dy;

    if ((source == (const Aim_FilteredTarget_t *)0) || (target == (Aim_FilteredTarget_t *)0))
    {
        return;
    }

    *target = *source;
    predicted_dx = (float)source->dx + ((float)g_aim_velocity_dx * g_aim_line_pred_x);
    predicted_dy = (float)source->dy + ((float)g_aim_velocity_dy * g_aim_line_pred_y);
    target->dx = Aim_RoundClampInt16(predicted_dx);
    target->dy = Aim_RoundClampInt16(predicted_dy);
}

static uint8_t Aim_HandleTurnPrefeed(uint32_t now_ms)
{
    int8_t dir = (g_aim_hint_turn_dir == 0) ? 1 : g_aim_hint_turn_dir;
    int16_t cmd_dx;
    int16_t cmd_dy;

    if (g_aim_turn_prefeed_done != 0U)
    {
        Aim_TrackEnterPhase(AIM_TRACK_PHASE_TURN_PREFEED);
        return 1U;
    }

    cmd_dx = Aim_ClampCommandFloat((float)dir * g_aim_turn_prefeed_x);
    cmd_dy = Aim_ClampCommandFloat((float)dir * g_aim_turn_prefeed_y);
    g_aim_last_ff_dx = cmd_dx;
    g_aim_last_ff_dy = cmd_dy;

    if (Aim_MoveByStepCommand(cmd_dx, cmd_dy) == 0U)
    {
        g_aim_last_error = AIM_ERR_GIMBAL;
        Aim_TrackEnterPhase(AIM_TRACK_PHASE_ERROR);
        Aim_EnterState(AIM_STATE_ERROR);
        return 0U;
    }

    if ((cmd_dx != 0) || (cmd_dy != 0))
    {
        g_aim_track_move_count++;
        g_aim_last_step_ms = now_ms;
    }

    g_aim_turn_prefeed_done = 1U;
    Aim_FilterClear();
    Aim_ResetLockCounters();
    Aim_TrackEnterPhase(AIM_TRACK_PHASE_TURN_PREFEED);
    return 1U;
}

static uint8_t Aim_HandleTurnFeed(uint32_t now_ms, uint8_t has_target, uint8_t new_frame)
{
    int16_t ff_dx = 0;
    int16_t ff_dy = 0;

    Aim_TrackEnterPhase(AIM_TRACK_PHASE_TURN_FEED);
    if ((uint32_t)(now_ms - g_aim_last_step_ms) < AIM_TRACK_INTERVAL_MS)
    {
        return 1U;
    }

    Aim_BuildTurnFeedCommand(now_ms, &ff_dx, &ff_dy);
    g_aim_last_ff_dx = ff_dx;
    g_aim_last_ff_dy = ff_dy;

    if ((ff_dx == 0) && (ff_dy == 0))
    {
        return 1U;
    }

    (void)has_target;
    (void)new_frame;
    if (Aim_MoveByStepCommand(ff_dx, ff_dy) == 0U)
    {
        g_aim_last_error = AIM_ERR_GIMBAL;
        Aim_TrackEnterPhase(AIM_TRACK_PHASE_ERROR);
        Aim_EnterState(AIM_STATE_ERROR);
        return 0U;
    }

    if ((ff_dx != 0) || (ff_dy != 0))
    {
        g_aim_track_move_count++;
        g_aim_last_step_ms = now_ms;
    }
    g_aim_turn_last_feed_ms = now_ms;
    g_aim_turn_last_feed_progress_deg = g_aim_hint_turn_progress_deg;
    Aim_ResetLockCounters();
    return 1U;
}

static void Aim_BuildTurnFeedCommand(uint32_t now_ms, int16_t *ff_dx, int16_t *ff_dy)
{
    int8_t dir = (g_aim_hint_turn_dir == 0) ? 1 : g_aim_hint_turn_dir;
    float progress_delta;
    float dt_s;
    float gyro_abs;
    float cmd_x;
    float cmd_y;

    if ((ff_dx == (int16_t *)0) || (ff_dy == (int16_t *)0))
    {
        return;
    }

    progress_delta = g_aim_hint_turn_progress_deg - g_aim_turn_last_feed_progress_deg;
    if (progress_delta < 0.0f)
    {
        progress_delta = 0.0f;
    }
    progress_delta = Aim_ClampFloat(progress_delta, 0.0f, 45.0f);

    if (g_aim_turn_last_feed_ms == 0U)
    {
        dt_s = (float)AIM_TRACK_INTERVAL_MS / 1000.0f;
    }
    else
    {
        dt_s = (float)((uint32_t)(now_ms - g_aim_turn_last_feed_ms)) / 1000.0f;
        dt_s = Aim_ClampFloat(dt_s, 0.0f, 0.25f);
    }

    gyro_abs = Aim_AbsFloat(g_aim_hint_turn_gyro_z_dps);
    cmd_x = (float)dir * ((progress_delta * g_aim_turn_ff_x_per_deg) +
                          (gyro_abs * dt_s * g_aim_turn_ff_gyro_x));
    cmd_y = (float)dir * ((progress_delta * g_aim_turn_ff_y_per_deg) +
                          (gyro_abs * dt_s * g_aim_turn_ff_gyro_y));
    *ff_dx = Aim_ClampCommandFloat(cmd_x);
    *ff_dy = Aim_ClampCommandFloat(cmd_y);
}

static void Aim_ResetDynamicTrack(void)
{
    g_aim_hint_track_state = TRACK_STATE_IDLE;
    g_aim_hint_turn_dir = 0;
    g_aim_hint_turn_progress_deg = 0.0f;
    g_aim_hint_turn_gyro_z_dps = 0.0f;
    g_aim_turn_prefeed_done = 0U;
    g_aim_recover_filter_cleared = 0U;
    g_aim_turn_last_feed_progress_deg = 0.0f;
    g_aim_turn_last_feed_ms = 0U;
    g_aim_has_last_raw_target = 0U;
    g_aim_last_raw_dx = 0;
    g_aim_last_raw_dy = 0;
    g_aim_velocity_dx = 0;
    g_aim_velocity_dy = 0;
    g_aim_last_ff_dx = 0;
    g_aim_last_ff_dy = 0;
}

static uint8_t Aim_TrackHintIsTurnPhase(void)
{
    return (uint8_t)((g_aim_hint_track_state == TRACK_STATE_CORNER_ADVANCE) ||
                     (g_aim_hint_track_state == TRACK_STATE_TURNING) ||
                     (g_aim_hint_track_state == TRACK_STATE_RECOVER_LINE));
}

static void Aim_ResetLockCounters(void)
{
    g_aim_lock_frames = 0U;
    g_aim_lock_exit_frames = 0U;
}

static uint8_t Aim_FilteredWithinTolerance(const Aim_FilteredTarget_t *target, int16_t tolerance)
{
    if (target == (const Aim_FilteredTarget_t *)0)
    {
        return 0U;
    }

    return Aim_ErrorWithinTolerance(target->dx, target->dy, tolerance);
}

static uint8_t Aim_RawWithinTolerance(const Vision_Target_t *target, int16_t tolerance)
{
    if (target == (const Vision_Target_t *)0)
    {
        return 0U;
    }

    return Aim_ErrorWithinTolerance(target->dx, target->dy, tolerance);
}

static Aim_LockResult_t Aim_UpdateLockConfirm(const Vision_Target_t *raw_target,
                                              const Aim_FilteredTarget_t *filtered_target,
                                              uint8_t new_frame)
{
    uint8_t raw_ok;
    uint8_t filtered_ok;

    if ((raw_target == (const Vision_Target_t *)0) ||
        (filtered_target == (const Aim_FilteredTarget_t *)0) ||
        (new_frame == 0U))
    {
        return AIM_LOCK_RESULT_WAIT;
    }

    raw_ok = Aim_RawWithinTolerance(raw_target, AIM_LOCK_TOLERANCE_PX);
    filtered_ok = Aim_FilteredWithinTolerance(filtered_target, AIM_LOCK_TOLERANCE_PX);
    if ((raw_ok != 0U) && (filtered_ok != 0U))
    {
        g_aim_lock_exit_frames = 0U;
        if (g_aim_lock_frames < AIM_LOCK_CONFIRM_FRAMES)
        {
            g_aim_lock_frames++;
        }
        return (g_aim_lock_frames >= AIM_LOCK_CONFIRM_FRAMES) ? AIM_LOCK_RESULT_CONFIRMED
                                                              : AIM_LOCK_RESULT_WAIT;
    }

    g_aim_lock_frames = 0U;
    if ((Aim_RawWithinTolerance(raw_target, AIM_LOCK_EXIT_TOLERANCE_PX) == 0U) ||
        (Aim_FilteredWithinTolerance(filtered_target, AIM_LOCK_EXIT_TOLERANCE_PX) == 0U))
    {
        g_aim_lock_exit_frames = AIM_LOCK_EXIT_CONFIRM_FRAMES;
    }
    else if (g_aim_lock_exit_frames < AIM_LOCK_EXIT_CONFIRM_FRAMES)
    {
        g_aim_lock_exit_frames++;
    }

    return (g_aim_lock_exit_frames >= AIM_LOCK_EXIT_CONFIRM_FRAMES) ? AIM_LOCK_RESULT_RETRY
                                                                    : AIM_LOCK_RESULT_WAIT;
}

static void Aim_BeginLockVerify(Aim_State_t next_state)
{
    Aim_FilterClear();
    Aim_ResetLockCounters();
    g_aim_last_error = AIM_ERR_NONE;
    Aim_EnterState(next_state);
}

static uint8_t Aim_OnceTimedOut(void)
{
    if ((uint32_t)(HAL_GetTick() - g_aim_once_start_ms) <= g_aim_once_timeout_ms)
    {
        return 0U;
    }

    g_aim_last_error = AIM_ERR_TIMEOUT;
    Aim_EnterState(AIM_STATE_ERROR);
    return 1U;
}

static uint8_t Aim_RunClosedLoop(uint8_t use_timeout, uint32_t timeout_ms, uint8_t use_track_interval)
{
    Vision_Target_t raw_target;
    uint8_t has_target = 0U;
    uint8_t new_frame = 0U;
    uint8_t fresh_frame;
    uint32_t now_ms;

    (void)timeout_ms;

    if ((use_timeout != 0U) && (Aim_OnceTimedOut() != 0U))
    {
        return 0U;
    }

    if (Gimbal_IsBusy() != 0U)
    {
        return 1U;
    }

    fresh_frame = Aim_ReadVisionFrame(&raw_target, &has_target, &new_frame);
    if ((fresh_frame == 0U) || (has_target == 0U))
    {
        g_aim_last_error = AIM_ERR_NO_TARGET;
        Aim_ResetLockCounters();
        return 1U;
    }

    if (Aim_FilterReady() == 0U)
    {
        return 1U;
    }

    if (Aim_FilteredWithinTolerance(&g_aim_filtered_target, AIM_LOCK_TOLERANCE_PX) != 0U)
    {
        g_aim_last_error = AIM_ERR_NONE;
        if ((use_timeout != 0U) && (use_track_interval == 0U))
        {
            Aim_BeginLockVerify(AIM_STATE_LOCK_VERIFY);
            return 1U;
        }
        if (Aim_UpdateLockConfirm(&raw_target, &g_aim_filtered_target, new_frame) == AIM_LOCK_RESULT_CONFIRMED)
        {
            Aim_EnterState(AIM_STATE_LOCKED);
        }
        return 1U;
    }

    Aim_ResetLockCounters();
    if ((use_track_interval != 0U) &&
        ((uint32_t)(HAL_GetTick() - g_aim_last_step_ms) < AIM_TRACK_INTERVAL_MS))
    {
        return 1U;
    }
    if (new_frame == 0U)
    {
        return 1U;
    }

    now_ms = HAL_GetTick();
    if (Aim_MoveByFilteredError(&g_aim_filtered_target) == 0U)
    {
        g_aim_last_error = (Gimbal_IsCalibrated() == 0U) ? AIM_ERR_CAL : AIM_ERR_GIMBAL;
        Aim_EnterState(AIM_STATE_ERROR);
        return 0U;
    }

    g_aim_last_error = AIM_ERR_NONE;
    g_aim_last_step_ms = now_ms;
    Aim_FilterClear();
    return 1U;
}

static uint8_t Aim_Q3TimedOut(void)
{
    uint32_t elapsed_ms = (uint32_t)(HAL_GetTick() - g_aim_q3_start_ms);

    if (elapsed_ms <= g_aim_q3_timeout_ms)
    {
        return 0U;
    }

    g_aim_last_error = AIM_ERR_TIMEOUT;
    Gimbal_Stop();
    Aim_EnterState(AIM_STATE_ERROR);
    return 1U;
}

static void Aim_Q3Reset(void)
{
    g_aim_q3_scan_started = 0U;
    g_aim_q3_seen_frames = 0U;
    g_aim_q3_stable_frames = 0U;
    g_aim_q3_miss_frames = 0U;
    g_aim_q3_settle_start_ms = 0U;
    g_aim_q3_start_ms = 0U;
}

static void Aim_Q3RestartScan(void)
{
    g_aim_q3_scan_started = 0U;
    g_aim_q3_seen_frames = 0U;
    g_aim_q3_stable_frames = 0U;
    g_aim_q3_miss_frames = 0U;
    g_aim_q3_settle_start_ms = 0U;
    Aim_FilterClear();
    Aim_ResetLockCounters();
    Aim_EnterState(AIM_STATE_Q3_SCAN_REV);
}

static void Aim_RunOnce(void)
{
    (void)Aim_RunClosedLoop(1U, g_aim_once_timeout_ms, 0U);
}

static void Aim_RunTrack(void)
{
    Vision_Target_t raw_target;
    uint8_t has_target = 0U;
    uint8_t new_frame = 0U;
    uint8_t fresh_frame;
    uint8_t raw_inner;
    uint8_t filtered_inner;
    uint8_t predicted_inner;
    uint8_t raw_exit;
    uint8_t filtered_exit;
    uint8_t predicted_exit;
    uint32_t now_ms;
    uint32_t miss_ms;
    Aim_FilteredTarget_t move_target;

    if (Gimbal_IsBusy() != 0U)
    {
        if (Aim_TrackHintIsTurnPhase() == 0U)
        {
            Aim_TrackEnterPhase(AIM_TRACK_PHASE_MOVING);
        }
        return;
    }

    if ((g_aim_track_phase == AIM_TRACK_PHASE_MOVING) &&
        (Aim_TrackHintIsTurnPhase() == 0U))
    {
        Aim_FilterClear();
        Aim_ResetLockCounters();
        Aim_TrackEnterPhase(AIM_TRACK_PHASE_ACQUIRE);
    }

    now_ms = HAL_GetTick();
    fresh_frame = Aim_ReadVisionFrame(&raw_target, &has_target, &new_frame);
    if ((fresh_frame != 0U) && (has_target != 0U))
    {
        Aim_UpdateVisionVelocity(&raw_target, new_frame);
    }

    if ((fresh_frame == 0U) || (has_target == 0U))
    {
        if (g_aim_track_miss_start_ms == 0U)
        {
            g_aim_track_miss_start_ms = now_ms;
        }

        miss_ms = (uint32_t)(now_ms - g_aim_track_miss_start_ms);
        g_aim_last_error = AIM_ERR_NO_TARGET;
        Aim_FilterClear();
        Aim_ResetLockCounters();

        if (miss_ms > AIM_TRACK_LOST_ERROR_MS)
        {
            Aim_TrackEnterPhase(AIM_TRACK_PHASE_ERROR);
            Aim_EnterState(AIM_STATE_ERROR);
            return;
        }

        if (g_aim_hint_track_state == TRACK_STATE_TURNING)
        {
            (void)Aim_HandleTurnFeed(now_ms, 0U, 0U);
        }
        else
        {
            Aim_TrackEnterPhase(AIM_TRACK_PHASE_MISS_HOLD);
        }
        return;
    }

    g_aim_track_miss_start_ms = 0U;
    g_aim_last_error = AIM_ERR_NONE;

    if (g_aim_hint_track_state == TRACK_STATE_CORNER_ADVANCE)
    {
        (void)Aim_HandleTurnPrefeed(now_ms);
        return;
    }

    if (g_aim_hint_track_state == TRACK_STATE_TURNING)
    {
        (void)Aim_HandleTurnFeed(now_ms, has_target, new_frame);
        return;
    }

    if (g_aim_hint_track_state == TRACK_STATE_RECOVER_LINE)
    {
        if (g_aim_recover_filter_cleared == 0U)
        {
            Aim_FilterClear();
            Aim_ResetLockCounters();
            g_aim_has_last_raw_target = 0U;
            g_aim_velocity_dx = 0;
            g_aim_velocity_dy = 0;
            g_aim_recover_filter_cleared = 1U;
        }
        Aim_TrackEnterPhase(AIM_TRACK_PHASE_RECOVER_AIM);
        return;
    }

    if (Aim_FilterReady() == 0U)
    {
        Aim_TrackEnterPhase(AIM_TRACK_PHASE_ACQUIRE);
        return;
    }

    if (g_aim_hint_track_state == TRACK_STATE_LINE_FOLLOW)
    {
        Aim_BuildLinePredictedTarget(&g_aim_filtered_target, &move_target);
    }
    else
    {
        move_target = g_aim_filtered_target;
    }
    raw_inner = Aim_RawWithinTolerance(&raw_target, AIM_LOCK_TOLERANCE_PX);
    filtered_inner = Aim_FilteredWithinTolerance(&g_aim_filtered_target, AIM_LOCK_TOLERANCE_PX);
    predicted_inner = Aim_FilteredWithinTolerance(&move_target, AIM_LOCK_TOLERANCE_PX);
    if ((raw_inner != 0U) && (filtered_inner != 0U) && (predicted_inner != 0U))
    {
        Aim_ResetLockCounters();
        Aim_TrackEnterPhase(AIM_TRACK_PHASE_HOLD);
        return;
    }

    raw_exit = (uint8_t)(Aim_RawWithinTolerance(&raw_target, AIM_LOCK_EXIT_TOLERANCE_PX) == 0U);
    filtered_exit = (uint8_t)(Aim_FilteredWithinTolerance(&g_aim_filtered_target, AIM_LOCK_EXIT_TOLERANCE_PX) == 0U);
    predicted_exit = (uint8_t)(Aim_FilteredWithinTolerance(&move_target, AIM_LOCK_EXIT_TOLERANCE_PX) == 0U);

    if ((raw_exit == 0U) && (filtered_exit == 0U) && (predicted_exit == 0U))
    {
        Aim_ResetLockCounters();
        Aim_TrackEnterPhase(AIM_TRACK_PHASE_HOLD);
        return;
    }

    if ((raw_exit != filtered_exit) && (predicted_exit == 0U))
    {
        Aim_FilterClear();
        Aim_FilterPush(&raw_target);
        Aim_ResetLockCounters();
        Aim_TrackEnterPhase(AIM_TRACK_PHASE_ACQUIRE);
        return;
    }

    if (new_frame == 0U)
    {
        return;
    }

    if ((uint32_t)(now_ms - g_aim_last_step_ms) < AIM_TRACK_INTERVAL_MS)
    {
        return;
    }

    if ((move_target.dx != g_aim_filtered_target.dx) || (move_target.dy != g_aim_filtered_target.dy))
    {
        Aim_TrackEnterPhase(AIM_TRACK_PHASE_LINE_PREDICT);
    }
    else
    {
        Aim_TrackEnterPhase(AIM_TRACK_PHASE_LINE_TRACK);
    }

    if (Aim_MoveByFilteredError(&move_target) == 0U)
    {
        g_aim_last_error = (Gimbal_IsCalibrated() == 0U) ? AIM_ERR_CAL : AIM_ERR_GIMBAL;
        Aim_TrackEnterPhase(AIM_TRACK_PHASE_ERROR);
        Aim_EnterState(AIM_STATE_ERROR);
        return;
    }

    if ((g_aim_last_cmd_dx != 0) || (g_aim_last_cmd_dy != 0))
    {
        g_aim_track_move_count++;
    }
    g_aim_last_step_ms = now_ms;
    Aim_FilterClear();
    Aim_ResetLockCounters();
    Aim_TrackEnterPhase(AIM_TRACK_PHASE_MOVING);
}

static void Aim_RunLockVerify(void)
{
    Vision_Target_t raw_target;
    Aim_LockResult_t lock_result;
    uint8_t has_target = 0U;
    uint8_t new_frame = 0U;
    uint8_t fresh_frame;

    if (Aim_OnceTimedOut() != 0U)
    {
        return;
    }

    if (Gimbal_IsBusy() != 0U)
    {
        return;
    }

    if (Aim_StateElapsedMs() < AIM_LOCK_VERIFY_SETTLE_MS)
    {
        return;
    }

    fresh_frame = Aim_ReadVisionFrame(&raw_target, &has_target, &new_frame);
    if ((fresh_frame == 0U) || (has_target == 0U))
    {
        g_aim_last_error = AIM_ERR_NO_TARGET;
        Aim_ResetLockCounters();
        return;
    }

    if (Aim_FilterReady() == 0U)
    {
        return;
    }

    lock_result = Aim_UpdateLockConfirm(&raw_target, &g_aim_filtered_target, new_frame);
    if (lock_result == AIM_LOCK_RESULT_CONFIRMED)
    {
        g_aim_last_error = AIM_ERR_NONE;
        Aim_EnterState(AIM_STATE_LOCKED);
        return;
    }
    if (lock_result == AIM_LOCK_RESULT_RETRY)
    {
        Aim_FilterClear();
        Aim_ResetLockCounters();
        g_aim_last_error = AIM_ERR_NONE;
        Aim_EnterState(AIM_STATE_ONCE);
    }
}

static void Aim_RunQuestion3ScanRev(void)
{
    Vision_Target_t target;
    uint8_t has_target = 0U;
    uint8_t new_frame = 0U;
    uint8_t fresh_frame;

    if (Aim_Q3TimedOut() != 0U)
    {
        return;
    }

    fresh_frame = Aim_ReadVisionFrame(&target, &has_target, &new_frame);
    if ((fresh_frame != 0U) && (new_frame != 0U))
    {
        if (has_target != 0U)
        {
            g_aim_q3_miss_frames = 0U;
            if (g_aim_q3_seen_frames < AIM_Q3_TARGET_CONFIRM_FRAMES)
            {
                g_aim_q3_seen_frames++;
            }
            if (g_aim_q3_seen_frames >= AIM_Q3_TARGET_CONFIRM_FRAMES)
            {
                Gimbal_Stop();
                Aim_FilterClear();
                Aim_ResetLockCounters();
                g_aim_q3_stable_frames = 0U;
                g_aim_q3_settle_start_ms = 0U;
                Aim_EnterState(AIM_STATE_Q3_SCAN_SETTLE);
                return;
            }
        }
        else
        {
            if (g_aim_q3_miss_frames < AIM_Q3_CONFIRM_MISS_LIMIT)
            {
                g_aim_q3_miss_frames++;
            }
            if (g_aim_q3_miss_frames >= AIM_Q3_CONFIRM_MISS_LIMIT)
            {
                g_aim_q3_seen_frames = 0U;
            }
        }
    }

    if ((g_aim_q3_scan_started == 0U) &&
        (g_aim_q3_seen_frames > 0U) &&
        (has_target != 0U))
    {
        return;
    }

    if (Gimbal_IsBusy() != 0U)
    {
        return;
    }

    if (g_aim_q3_scan_started == 0U)
    {
        g_aim_q3_scan_started = 1U;
        if (Gimbal_MoveRelativeSteps(AIM_Q3_SCAN_REV_STEPS, 0, AIM_Q3_SCAN_SPEED_SPS) == 0U)
        {
            g_aim_last_error = AIM_ERR_GIMBAL;
            Aim_EnterState(AIM_STATE_ERROR);
        }
        return;
    }

    g_aim_last_error = AIM_ERR_NO_TARGET;
    Aim_EnterState(AIM_STATE_ERROR);
}

static void Aim_RunQuestion3ScanSettle(void)
{
    if (Aim_Q3TimedOut() != 0U)
    {
        return;
    }

    if (Gimbal_IsBusy() != 0U)
    {
        return;
    }

    if (g_aim_q3_settle_start_ms == 0U)
    {
        g_aim_q3_settle_start_ms = HAL_GetTick();
        Aim_FilterClear();
        return;
    }

    if ((uint32_t)(HAL_GetTick() - g_aim_q3_settle_start_ms) < AIM_Q3_SCAN_SETTLE_MS)
    {
        return;
    }

    g_aim_q3_stable_frames = 0U;
    g_aim_q3_miss_frames = 0U;
    Aim_FilterClear();
    Aim_ResetLockCounters();
    Aim_EnterState(AIM_STATE_Q3_STABLE_CONFIRM);
}

static void Aim_RunQuestion3StableConfirm(void)
{
    Vision_Target_t target;
    uint8_t has_target = 0U;
    uint8_t new_frame = 0U;
    uint8_t fresh_frame;

    if (Aim_Q3TimedOut() != 0U)
    {
        return;
    }

    fresh_frame = Aim_ReadVisionFrame(&target, &has_target, &new_frame);
    if ((fresh_frame != 0U) && (new_frame != 0U))
    {
        if (has_target != 0U)
        {
            g_aim_q3_miss_frames = 0U;
            if (g_aim_q3_stable_frames < AIM_Q3_STABLE_CONFIRM_FRAMES)
            {
                g_aim_q3_stable_frames++;
            }
            if ((g_aim_q3_stable_frames >= AIM_Q3_STABLE_CONFIRM_FRAMES) &&
                (Aim_FilterReady() != 0U))
            {
                Aim_ResetLockCounters();
                Aim_EnterState(AIM_STATE_Q3_AIM);
                return;
            }
        }
        else
        {
            if (g_aim_q3_miss_frames < AIM_Q3_CONFIRM_MISS_LIMIT)
            {
                g_aim_q3_miss_frames++;
            }
            if (g_aim_q3_miss_frames >= AIM_Q3_CONFIRM_MISS_LIMIT)
            {
                Aim_Q3RestartScan();
                return;
            }
        }
    }

    if (Aim_StateElapsedMs() > AIM_Q3_STABLE_CONFIRM_TIMEOUT_MS)
    {
        Aim_Q3RestartScan();
    }
}

static void Aim_RunQuestion3Aim(void)
{
    Vision_Target_t target;
    uint8_t has_target = 0U;
    uint8_t new_frame = 0U;
    uint8_t fresh_frame;

    if (Aim_Q3TimedOut() != 0U)
    {
        return;
    }

    if (Gimbal_IsBusy() != 0U)
    {
        return;
    }

    fresh_frame = Aim_ReadVisionFrame(&target, &has_target, &new_frame);
    if ((fresh_frame == 0U) || (has_target == 0U))
    {
        g_aim_last_error = AIM_ERR_NO_TARGET;
        if ((fresh_frame != 0U) && (new_frame != 0U))
        {
            if (g_aim_q3_miss_frames < AIM_Q3_CONFIRM_MISS_LIMIT)
            {
                g_aim_q3_miss_frames++;
            }
            if (g_aim_q3_miss_frames >= AIM_Q3_CONFIRM_MISS_LIMIT)
            {
                Aim_Q3RestartScan();
            }
        }
        return;
    }

    g_aim_q3_miss_frames = 0U;
    if (Aim_FilterReady() == 0U)
    {
        return;
    }

    if (Aim_FilteredWithinTolerance(&g_aim_filtered_target, AIM_LOCK_TOLERANCE_PX) != 0U)
    {
        Aim_BeginLockVerify(AIM_STATE_Q3_LOCK_VERIFY);
        return;
    }

    Aim_ResetLockCounters();
    if (new_frame == 0U)
    {
        return;
    }

    if (Aim_MoveByFilteredError(&g_aim_filtered_target) == 0U)
    {
        g_aim_last_error = (Gimbal_IsCalibrated() == 0U) ? AIM_ERR_CAL : AIM_ERR_GIMBAL;
        Aim_EnterState(AIM_STATE_ERROR);
        return;
    }

    g_aim_last_error = AIM_ERR_NONE;
    g_aim_last_step_ms = HAL_GetTick();
    Aim_FilterClear();
}

static void Aim_RunQuestion3LockVerify(void)
{
    Vision_Target_t target;
    Aim_LockResult_t lock_result;
    uint8_t has_target = 0U;
    uint8_t new_frame = 0U;
    uint8_t fresh_frame;

    if (Aim_Q3TimedOut() != 0U)
    {
        return;
    }

    if (Gimbal_IsBusy() != 0U)
    {
        return;
    }

    if (Aim_StateElapsedMs() < AIM_LOCK_VERIFY_SETTLE_MS)
    {
        return;
    }

    fresh_frame = Aim_ReadVisionFrame(&target, &has_target, &new_frame);
    if ((fresh_frame == 0U) || (has_target == 0U))
    {
        g_aim_last_error = AIM_ERR_NO_TARGET;
        if ((fresh_frame != 0U) && (new_frame != 0U))
        {
            if (g_aim_q3_miss_frames < AIM_Q3_CONFIRM_MISS_LIMIT)
            {
                g_aim_q3_miss_frames++;
            }
            if (g_aim_q3_miss_frames >= AIM_Q3_CONFIRM_MISS_LIMIT)
            {
                Aim_Q3RestartScan();
            }
        }
        return;
    }

    g_aim_q3_miss_frames = 0U;
    if (Aim_FilterReady() == 0U)
    {
        return;
    }

    lock_result = Aim_UpdateLockConfirm(&target, &g_aim_filtered_target, new_frame);
    if (lock_result == AIM_LOCK_RESULT_CONFIRMED)
    {
        g_aim_last_error = AIM_ERR_NONE;
        Aim_EnterState(AIM_STATE_LOCKED);
        return;
    }
    if (lock_result == AIM_LOCK_RESULT_WAIT)
    {
        return;
    }

    Aim_FilterClear();
    Aim_ResetLockCounters();
    g_aim_last_error = AIM_ERR_NONE;
    Aim_EnterState(AIM_STATE_Q3_AIM);
}

static void Aim_TrackEnterPhase(Aim_TrackPhase_t phase)
{
    g_aim_track_phase = phase;
}

static const char *Aim_GetTrackPhaseName(Aim_TrackPhase_t phase)
{
    switch (phase)
    {
    case AIM_TRACK_PHASE_IDLE:
        return "IDLE";
    case AIM_TRACK_PHASE_ACQUIRE:
        return "ACQUIRE";
    case AIM_TRACK_PHASE_LINE_TRACK:
        return "LINE_TRACK";
    case AIM_TRACK_PHASE_LINE_PREDICT:
        return "LINE_PREDICT";
    case AIM_TRACK_PHASE_TURN_PREFEED:
        return "TURN_PREFEED";
    case AIM_TRACK_PHASE_TURN_FEED:
        return "TURN_FEED";
    case AIM_TRACK_PHASE_RECOVER_AIM:
        return "RECOVER_AIM";
    case AIM_TRACK_PHASE_HOLD:
        return "HOLD";
    case AIM_TRACK_PHASE_CORRECT:
        return "CORRECT";
    case AIM_TRACK_PHASE_MOVING:
        return "MOVING";
    case AIM_TRACK_PHASE_MISS_HOLD:
        return "MISS_HOLD";
    case AIM_TRACK_PHASE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
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
