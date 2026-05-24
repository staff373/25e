#include "app_turn.h"

#include "app_imu.h"
#include "app_motion.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

#define APP_TURN_UPDATE_PERIOD_MS  (10U)
#define APP_TURN_DONE_DEG          (2.5f)   /* 距离目标角度还剩多少度时判定转弯完成 */
#define APP_TURN_RATE_POINT_STEP_DEG  (15.0f)
#define APP_TURN_RATE_POINT_COUNT     (7U)
#define APP_TURN_RATE_POINT_LAST      (APP_TURN_RATE_POINT_COUNT - 1U)
#define APP_TURN_MIN_RATE_SCALE       (0.35f)
#define APP_TURN_SETTLE_MAX_MS        (250U)

#define APP_TURN_DEFAULT_OUT_DUTY  (42.0f)  /* 外侧轮前进 duty，越大转得越快 */
#define APP_TURN_DEFAULT_IN_DUTY   (-1.5f)  /* 内侧轮反转 duty，越负越像原地甩弯 */
#define APP_TURN_DEFAULT_ANGLE_DEG (85.0f)  /* JY61P yaw 目标角度，决定转多少 */
#define APP_TURN_DEFAULT_MAX_MS    (90000U)   /* 转弯超时保护，防止 IMU/轮子异常一直转 */
#define APP_TURN_DEFAULT_RAMP_DUTY (1.0f)   /* 每 10ms 最大 duty 改变量，降低输出台阶 */
#define APP_TURN_DEFAULT_RATE_SCALE (0.60f) /* 角速度曲线整体倍率，>1 更快，<1 更稳 */
#define APP_TURN_DEFAULT_RATE_KP   (0.015f) /* 超过角速度限制后 duty 收缩强度 */
#define APP_TURN_DEFAULT_STOP_RATE (25.0f)  /* settle 阶段认为车身基本停稳的 gyro_z 阈值 */
#define APP_TURN_PARAM_TIME_MS_MAX (4294967040.0f)
#define APP_TURN_PARAM_RATE_MAX    (720.0f)
#define APP_TURN_PARAM_RATE_SCALE_MAX (3.0f)

typedef enum
{
    TURN_PHASE_IDLE = 0,
    TURN_PHASE_DRIVE,
    TURN_PHASE_SETTLE
} Turn_Phase_t;

static uint8_t g_turn_initialized = 0U;
static uint8_t g_turn_active = 0U;
static uint8_t g_turn_last_timeout = 0U;
static Turn_Phase_t g_turn_phase = TURN_PHASE_IDLE;
static int8_t g_turn_direction = TURN_DIR_NONE;
static uint32_t g_turn_last_update_ms = 0U;
static uint32_t g_turn_start_ms = 0U;
static uint32_t g_turn_settle_start_ms = 0U;
static float g_turn_start_yaw_deg = 0.0f;
static float g_turn_progress_deg = 0.0f;
static float g_turn_remaining_deg = 0.0f;
static float g_turn_current_yaw_deg = 0.0f;
static float g_turn_current_gyro_z_dps = 0.0f;
static float g_turn_out_duty = APP_TURN_DEFAULT_OUT_DUTY;
static float g_turn_in_duty = APP_TURN_DEFAULT_IN_DUTY;
static float g_turn_angle_deg = APP_TURN_DEFAULT_ANGLE_DEG;
static uint32_t g_turn_max_turn_ms = APP_TURN_DEFAULT_MAX_MS;
static float g_turn_ramp_duty = APP_TURN_DEFAULT_RAMP_DUTY;
static float g_turn_rate_scale = APP_TURN_DEFAULT_RATE_SCALE;
static float g_turn_rate_kp = APP_TURN_DEFAULT_RATE_KP;
static float g_turn_stop_rate_dps = APP_TURN_DEFAULT_STOP_RATE;
static float g_turn_rate_curve_dps[APP_TURN_RATE_POINT_COUNT] = {
    25.0f, 70.0f, 115.0f, 155.0f, 190.0f, 220.0f, 220.0f
};
static float g_turn_rate_limit_dps = 0.0f;
static float g_turn_output_scale = 1.0f;
static float g_turn_applied_out_duty = 0.0f;
static float g_turn_applied_in_duty = 0.0f;

static float Turn_NormalizeAngle180(float angle_deg);
static float Turn_Clamp(float value, float min_value, float max_value);
static float Turn_Abs(float value);
static float Turn_RampToward(float current, float target, float step);
static void Turn_ResetRateCurve(void);
static const char *Turn_GetPhaseName(void);
static void Turn_UpdateMeasurements(void);
static void Turn_EnterSettle(void);
static uint8_t Turn_IsSettled(uint32_t now_ms);
static float Turn_LookupRateLimitDps(float remaining_deg);
static float Turn_CalculateOutputScale(float gyro_abs_dps, float rate_limit_dps);
static void Turn_LoadInitialAppliedOutput(void);
static void Turn_ApplyCurrentOutput(void);
static void Turn_ApplyTargetOutput(float target_out_duty, float target_in_duty);
static uint8_t Turn_SetRatePointParam(const char *name, float value);
static uint8_t Turn_GetRatePointParam(const char *name, float *value);

void Turn_Init(void)
{
    g_turn_initialized = 1U;
    g_turn_active = 0U;
    g_turn_last_timeout = 0U;
    g_turn_phase = TURN_PHASE_IDLE;
    g_turn_direction = TURN_DIR_NONE;
    g_turn_last_update_ms = HAL_GetTick();
    g_turn_start_ms = 0U;
    g_turn_settle_start_ms = 0U;
    g_turn_start_yaw_deg = 0.0f;
    g_turn_progress_deg = 0.0f;
    g_turn_remaining_deg = 0.0f;
    g_turn_current_yaw_deg = 0.0f;
    g_turn_current_gyro_z_dps = 0.0f;
    g_turn_out_duty = APP_TURN_DEFAULT_OUT_DUTY;
    g_turn_in_duty = APP_TURN_DEFAULT_IN_DUTY;
    g_turn_angle_deg = APP_TURN_DEFAULT_ANGLE_DEG;
    g_turn_max_turn_ms = APP_TURN_DEFAULT_MAX_MS;
    g_turn_ramp_duty = APP_TURN_DEFAULT_RAMP_DUTY;
    g_turn_rate_scale = APP_TURN_DEFAULT_RATE_SCALE;
    g_turn_rate_kp = APP_TURN_DEFAULT_RATE_KP;
    g_turn_stop_rate_dps = APP_TURN_DEFAULT_STOP_RATE;
    g_turn_rate_limit_dps = 0.0f;
    g_turn_output_scale = 1.0f;
    g_turn_applied_out_duty = 0.0f;
    g_turn_applied_in_duty = 0.0f;
    Turn_ResetRateCurve();
}

void Turn_Poll(void)
{
    uint32_t now_ms;
    uint32_t elapsed_ms;
    float gyro_abs_dps;
    float target_out_duty;
    float target_in_duty;

    if ((g_turn_initialized == 0U) || (g_turn_active == 0U))
    {
        return;
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - g_turn_last_update_ms) < APP_TURN_UPDATE_PERIOD_MS)
    {
        return;
    }

    g_turn_last_update_ms = now_ms;
    if (Imu_IsOnline() == 0U)
    {
        g_turn_last_timeout = 1U;
        Turn_Stop();
        return;
    }

    Turn_UpdateMeasurements();
    elapsed_ms = (uint32_t)(now_ms - g_turn_start_ms);

    if (elapsed_ms >= g_turn_max_turn_ms)
    {
        g_turn_last_timeout = 1U;
        Turn_Stop();
        return;
    }

    if (g_turn_phase == TURN_PHASE_DRIVE)
    {
        if (g_turn_remaining_deg <= APP_TURN_DONE_DEG)
        {
            g_turn_last_timeout = 0U;
            Turn_EnterSettle();
            Turn_ApplyTargetOutput(0.0f, 0.0f);
            return;
        }

        gyro_abs_dps = Turn_Abs(g_turn_current_gyro_z_dps);
        g_turn_rate_limit_dps = Turn_LookupRateLimitDps(g_turn_remaining_deg);
        g_turn_output_scale = Turn_CalculateOutputScale(gyro_abs_dps, g_turn_rate_limit_dps);
        target_out_duty = Turn_Clamp(g_turn_out_duty * g_turn_output_scale, 0.0f, 100.0f);
        target_in_duty = Turn_Clamp(g_turn_in_duty * g_turn_output_scale, -100.0f, 0.0f);
        Turn_ApplyTargetOutput(target_out_duty, target_in_duty);
        return;
    }

    if (g_turn_phase == TURN_PHASE_SETTLE)
    {
        Turn_ApplyTargetOutput(0.0f, 0.0f);
        if (Turn_IsSettled(now_ms) != 0U)
        {
            Turn_Stop();
        }
    }
}

uint8_t Turn_Start(int8_t direction)
{
    if ((g_turn_initialized == 0U) || (Imu_IsOnline() == 0U))
    {
        return 0U;
    }

    if ((direction != TURN_DIR_LEFT) && (direction != TURN_DIR_RIGHT))
    {
        return 0U;
    }

    g_turn_last_timeout = 0U;
    g_turn_active = 1U;
    g_turn_phase = TURN_PHASE_DRIVE;
    g_turn_direction = direction;
    g_turn_start_ms = HAL_GetTick();
    g_turn_settle_start_ms = 0U;
    g_turn_last_update_ms = g_turn_start_ms;
    g_turn_start_yaw_deg = Imu_GetYawDeg();
    g_turn_current_yaw_deg = g_turn_start_yaw_deg;
    g_turn_current_gyro_z_dps = Imu_GetGyroZDps();
    g_turn_progress_deg = 0.0f;
    g_turn_remaining_deg = g_turn_angle_deg;
    g_turn_rate_limit_dps = Turn_LookupRateLimitDps(g_turn_remaining_deg);
    g_turn_output_scale = 1.0f;
    Turn_LoadInitialAppliedOutput();
    Turn_ApplyCurrentOutput();
    return 1U;
}

void Turn_Stop(void)
{
    g_turn_active = 0U;
    g_turn_phase = TURN_PHASE_IDLE;
    g_turn_direction = TURN_DIR_NONE;
    g_turn_settle_start_ms = 0U;
    g_turn_applied_out_duty = 0.0f;
    g_turn_applied_in_duty = 0.0f;
    g_turn_output_scale = 1.0f;
    Motion_Stop();
}

uint8_t Turn_IsActive(void)
{
    return g_turn_active;
}

uint8_t Turn_WasLastTimeout(void)
{
    return g_turn_last_timeout;
}

int8_t Turn_GetDirection(void)
{
    return g_turn_direction;
}

float Turn_GetProgressDeg(void)
{
    return g_turn_progress_deg;
}

float Turn_GetTargetAngleDeg(void)
{
    return g_turn_angle_deg;
}

uint8_t Turn_SetParam(const char *name, float value)
{
    if (name == (const char *)0)
    {
        return 0U;
    }

    if (strcmp(name, "TURN_OUT") == 0)
    {
        g_turn_out_duty = Turn_Clamp(value, 0.0f, 100.0f); /* 外侧轮速度，越大越快 */
        return 1U;
    }

    if (strcmp(name, "TURN_IN") == 0)
    {
        g_turn_in_duty = Turn_Clamp(value, -100.0f, 0.0f); /* 内侧轮反转强度，越负越利索 */
        return 1U;
    }

    if (strcmp(name, "TURN_ANGLE") == 0)
    {
        g_turn_angle_deg = Turn_Clamp(value, 45.0f, 180.0f); /* 角度大就转多，角度小就更早停 */
        return 1U;
    }

    if (strcmp(name, "MAX_TURN_MS") == 0)
    {
        g_turn_max_turn_ms = (uint32_t)Turn_Clamp(value, 100.0f, APP_TURN_PARAM_TIME_MS_MAX); /* 大了更宽松，小了更早保护停机 */
        return 1U;
    }

    if (strcmp(name, "TURN_RAMP") == 0)
    {
        g_turn_ramp_duty = Turn_Clamp(value, 0.1f, 100.0f); /* 每 10ms 最大 duty 变化，越小越柔和 */
        return 1U;
    }

    if (strcmp(name, "TURN_RATE_SCALE") == 0)
    {
        g_turn_rate_scale = Turn_Clamp(value, 0.20f, APP_TURN_PARAM_RATE_SCALE_MAX); /* 角速度曲线整体倍率 */
        return 1U;
    }

    if (strcmp(name, "TURN_RATE_KP") == 0)
    {
        g_turn_rate_kp = Turn_Clamp(value, 0.0f, 0.10f); /* 超速后 duty 收缩强度，0 表示只用斜坡 */
        return 1U;
    }

    if (strcmp(name, "TURN_STOP_RATE") == 0)
    {
        g_turn_stop_rate_dps = Turn_Clamp(value, 1.0f, APP_TURN_PARAM_RATE_MAX); /* settle 阶段 gyro_z 低于该值后放行 */
        return 1U;
    }

    if (Turn_SetRatePointParam(name, value) != 0U)
    {
        return 1U;
    }

    return 0U;
}

uint8_t Turn_GetParam(const char *name, float *value)
{
    if ((name == (const char *)0) || (value == (float *)0))
    {
        return 0U;
    }

    if (strcmp(name, "TURN_OUT") == 0)
    {
        *value = g_turn_out_duty;
        return 1U;
    }

    if (strcmp(name, "TURN_IN") == 0)
    {
        *value = g_turn_in_duty;
        return 1U;
    }

    if (strcmp(name, "TURN_ANGLE") == 0)
    {
        *value = g_turn_angle_deg;
        return 1U;
    }

    if (strcmp(name, "MAX_TURN_MS") == 0)
    {
        *value = (float)g_turn_max_turn_ms;
        return 1U;
    }

    if (strcmp(name, "TURN_RAMP") == 0)
    {
        *value = g_turn_ramp_duty;
        return 1U;
    }

    if (strcmp(name, "TURN_RATE_SCALE") == 0)
    {
        *value = g_turn_rate_scale;
        return 1U;
    }

    if (strcmp(name, "TURN_RATE_KP") == 0)
    {
        *value = g_turn_rate_kp;
        return 1U;
    }

    if (strcmp(name, "TURN_STOP_RATE") == 0)
    {
        *value = g_turn_stop_rate_dps;
        return 1U;
    }

    if (Turn_GetRatePointParam(name, value) != 0U)
    {
        return 1U;
    }

    return 0U;
}

void Turn_FormatStatus(char *buffer, size_t buffer_size, const char *prefix)
{
    const char *dir_text = "N";

    if ((buffer == (char *)0) || (buffer_size == 0U))
    {
        return;
    }

    if (prefix == (const char *)0)
    {
        prefix = "OK";
    }

    if (g_turn_direction == TURN_DIR_LEFT)
    {
        dir_text = "L";
    }
    else if (g_turn_direction == TURN_DIR_RIGHT)
    {
        dir_text = "R";
    }

    (void)snprintf(buffer,
                   buffer_size,
                   "%s TURN active=%u phase=%s dir=%s prog=%.1f remain=%.1f target=%.1f yaw=%.1f gyro_z=%.1f rate_lim=%.1f scale=%.2f out=%.1f in=%.1f timeout=%u",
                   prefix,
                   (unsigned int)g_turn_active,
                   Turn_GetPhaseName(),
                   dir_text,
                   g_turn_progress_deg,
                   g_turn_remaining_deg,
                   g_turn_angle_deg,
                   g_turn_current_yaw_deg,
                   g_turn_current_gyro_z_dps,
                   g_turn_rate_limit_dps,
                   g_turn_output_scale,
                   g_turn_applied_out_duty,
                   g_turn_applied_in_duty,
                   (unsigned int)g_turn_last_timeout);
}

static float Turn_NormalizeAngle180(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static float Turn_Clamp(float value, float min_value, float max_value)
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

static float Turn_Abs(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}

static float Turn_RampToward(float current, float target, float step)
{
    float delta;

    if (step >= 100.0f)
    {
        return target;
    }

    delta = target - current;
    if (delta > step)
    {
        return current + step;
    }

    if (delta < -step)
    {
        return current - step;
    }

    return target;
}

static void Turn_ResetRateCurve(void)
{
    g_turn_rate_curve_dps[0] = 25.0f;
    g_turn_rate_curve_dps[1] = 70.0f;
    g_turn_rate_curve_dps[2] = 115.0f;
    g_turn_rate_curve_dps[3] = 155.0f;
    g_turn_rate_curve_dps[4] = 190.0f;
    g_turn_rate_curve_dps[5] = 220.0f;
    g_turn_rate_curve_dps[6] = 220.0f;
}

static const char *Turn_GetPhaseName(void)
{
    switch (g_turn_phase)
    {
    case TURN_PHASE_IDLE:
        return "IDLE";
    case TURN_PHASE_DRIVE:
        return "DRIVE";
    case TURN_PHASE_SETTLE:
        return "SETTLE";
    default:
        return "UNKNOWN";
    }
}

static void Turn_UpdateMeasurements(void)
{
    g_turn_current_yaw_deg = Imu_GetYawDeg();
    g_turn_current_gyro_z_dps = Imu_GetGyroZDps();
    if (g_turn_direction == TURN_DIR_RIGHT)
    {
        g_turn_progress_deg = Turn_NormalizeAngle180(g_turn_start_yaw_deg - g_turn_current_yaw_deg);
    }
    else
    {
        g_turn_progress_deg = Turn_NormalizeAngle180(g_turn_current_yaw_deg - g_turn_start_yaw_deg);
    }

    g_turn_remaining_deg = g_turn_angle_deg - g_turn_progress_deg;
    if (g_turn_remaining_deg < 0.0f)
    {
        g_turn_remaining_deg = 0.0f;
    }
}

static void Turn_EnterSettle(void)
{
    g_turn_phase = TURN_PHASE_SETTLE;
    g_turn_settle_start_ms = HAL_GetTick();
    g_turn_rate_limit_dps = g_turn_stop_rate_dps;
    g_turn_output_scale = 0.0f;
}

static uint8_t Turn_IsSettled(uint32_t now_ms)
{
    if ((Turn_Abs(g_turn_applied_out_duty) <= 0.01f) &&
        (Turn_Abs(g_turn_applied_in_duty) <= 0.01f) &&
        ((Turn_Abs(g_turn_current_gyro_z_dps) <= g_turn_stop_rate_dps) ||
         ((uint32_t)(now_ms - g_turn_settle_start_ms) >= APP_TURN_SETTLE_MAX_MS)))
    {
        return 1U;
    }

    return 0U;
}

static float Turn_LookupRateLimitDps(float remaining_deg)
{
    uint32_t lower_index;
    float lower_deg;
    float fraction;
    float lower_rate;
    float upper_rate;
    float rate_limit;

    if (remaining_deg <= 0.0f)
    {
        return Turn_Clamp(g_turn_rate_curve_dps[0] * g_turn_rate_scale, 1.0f, APP_TURN_PARAM_RATE_MAX);
    }

    lower_index = (uint32_t)(remaining_deg / APP_TURN_RATE_POINT_STEP_DEG);
    if (lower_index >= APP_TURN_RATE_POINT_LAST)
    {
        return Turn_Clamp(g_turn_rate_curve_dps[APP_TURN_RATE_POINT_LAST] * g_turn_rate_scale,
                          1.0f,
                          APP_TURN_PARAM_RATE_MAX);
    }

    lower_deg = (float)lower_index * APP_TURN_RATE_POINT_STEP_DEG;
    fraction = (remaining_deg - lower_deg) / APP_TURN_RATE_POINT_STEP_DEG;
    lower_rate = g_turn_rate_curve_dps[lower_index];
    upper_rate = g_turn_rate_curve_dps[lower_index + 1U];
    rate_limit = lower_rate + ((upper_rate - lower_rate) * fraction);
    return Turn_Clamp(rate_limit * g_turn_rate_scale, 1.0f, APP_TURN_PARAM_RATE_MAX);
}

static float Turn_CalculateOutputScale(float gyro_abs_dps, float rate_limit_dps)
{
    float over_dps;
    float scale;

    over_dps = gyro_abs_dps - rate_limit_dps;
    if ((over_dps <= 0.0f) || (g_turn_rate_kp <= 0.0f))
    {
        return 1.0f;
    }

    scale = 1.0f - (over_dps * g_turn_rate_kp);
    return Turn_Clamp(scale, APP_TURN_MIN_RATE_SCALE, 1.0f);
}

static void Turn_LoadInitialAppliedOutput(void)
{
    if (g_turn_direction == TURN_DIR_LEFT)
    {
        g_turn_applied_out_duty = Turn_Clamp((Motion_GetDuty(MOTION_WHEEL_RF) + Motion_GetDuty(MOTION_WHEEL_RB)) * 0.5f,
                                             0.0f,
                                             100.0f);
    }
    else
    {
        g_turn_applied_out_duty = Turn_Clamp((Motion_GetDuty(MOTION_WHEEL_LF) + Motion_GetDuty(MOTION_WHEEL_LB)) * 0.5f,
                                             0.0f,
                                             100.0f);
    }

    g_turn_applied_in_duty = 0.0f;
}

static void Turn_ApplyCurrentOutput(void)
{
    if (g_turn_direction == TURN_DIR_LEFT)
    {
        Motion_SetDuty4(g_turn_applied_in_duty,
                        g_turn_applied_out_duty,
                        g_turn_applied_in_duty,
                        g_turn_applied_out_duty);
    }
    else if (g_turn_direction == TURN_DIR_RIGHT)
    {
        Motion_SetDuty4(g_turn_applied_out_duty,
                        g_turn_applied_in_duty,
                        g_turn_applied_out_duty,
                        g_turn_applied_in_duty);
    }
}

static void Turn_ApplyTargetOutput(float target_out_duty, float target_in_duty)
{
    g_turn_applied_out_duty = Turn_RampToward(g_turn_applied_out_duty,
                                              Turn_Clamp(target_out_duty, 0.0f, 100.0f),
                                              g_turn_ramp_duty);
    g_turn_applied_in_duty = Turn_RampToward(g_turn_applied_in_duty,
                                             Turn_Clamp(target_in_duty, -100.0f, 0.0f),
                                             g_turn_ramp_duty);
    Turn_ApplyCurrentOutput();
}

static uint8_t Turn_SetRatePointParam(const char *name, float value)
{
    float rate;

    if (name == (const char *)0)
    {
        return 0U;
    }

    rate = Turn_Clamp(value, 1.0f, APP_TURN_PARAM_RATE_MAX);
    if (strcmp(name, "TURN_R0") == 0)
    {
        g_turn_rate_curve_dps[0] = rate;
        return 1U;
    }
    if (strcmp(name, "TURN_R15") == 0)
    {
        g_turn_rate_curve_dps[1] = rate;
        return 1U;
    }
    if (strcmp(name, "TURN_R30") == 0)
    {
        g_turn_rate_curve_dps[2] = rate;
        return 1U;
    }
    if (strcmp(name, "TURN_R45") == 0)
    {
        g_turn_rate_curve_dps[3] = rate;
        return 1U;
    }
    if (strcmp(name, "TURN_R60") == 0)
    {
        g_turn_rate_curve_dps[4] = rate;
        return 1U;
    }
    if (strcmp(name, "TURN_R75") == 0)
    {
        g_turn_rate_curve_dps[5] = rate;
        return 1U;
    }
    if (strcmp(name, "TURN_R90") == 0)
    {
        g_turn_rate_curve_dps[6] = rate;
        return 1U;
    }

    return 0U;
}

static uint8_t Turn_GetRatePointParam(const char *name, float *value)
{
    if ((name == (const char *)0) || (value == (float *)0))
    {
        return 0U;
    }

    if (strcmp(name, "TURN_R0") == 0)
    {
        *value = g_turn_rate_curve_dps[0];
        return 1U;
    }
    if (strcmp(name, "TURN_R15") == 0)
    {
        *value = g_turn_rate_curve_dps[1];
        return 1U;
    }
    if (strcmp(name, "TURN_R30") == 0)
    {
        *value = g_turn_rate_curve_dps[2];
        return 1U;
    }
    if (strcmp(name, "TURN_R45") == 0)
    {
        *value = g_turn_rate_curve_dps[3];
        return 1U;
    }
    if (strcmp(name, "TURN_R60") == 0)
    {
        *value = g_turn_rate_curve_dps[4];
        return 1U;
    }
    if (strcmp(name, "TURN_R75") == 0)
    {
        *value = g_turn_rate_curve_dps[5];
        return 1U;
    }
    if (strcmp(name, "TURN_R90") == 0)
    {
        *value = g_turn_rate_curve_dps[6];
        return 1U;
    }

    return 0U;
}
