#include "app_turn.h"

#include "app_imu.h"
#include "app_motion.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

#define APP_TURN_UPDATE_PERIOD_MS  (10U)
#define APP_TURN_DONE_DEG          (2.5f)   /* 距离目标角度还剩多少度时判定转弯完成 */
#define APP_TURN_SLOWDOWN_DEG      (12.0f)  /* 接近目标角度多少度后开始减速 */
#define APP_TURN_SLOW_SCALE        (0.12f)  /* 接近终点后的减速比例 */

#define APP_TURN_DEFAULT_OUT_DUTY  (75.0f)  /* 外侧轮前进 duty，越大转得越快 */
#define APP_TURN_DEFAULT_IN_DUTY   (-40.0f) /* 内侧轮反转 duty，越负越像原地甩弯 */
#define APP_TURN_DEFAULT_ANGLE_DEG (88.0f)  /* JY61P yaw 目标角度，决定转多少 */
#define APP_TURN_DEFAULT_MAX_MS    (90000U)   /* 转弯超时保护，防止 IMU/轮子异常一直转 */
#define APP_TURN_PARAM_TIME_MS_MAX (4294967040.0f)

static uint8_t g_turn_initialized = 0U;
static uint8_t g_turn_active = 0U;
static uint8_t g_turn_last_timeout = 0U;
static int8_t g_turn_direction = TURN_DIR_NONE;
static uint32_t g_turn_last_update_ms = 0U;
static uint32_t g_turn_start_ms = 0U;
static float g_turn_start_yaw_deg = 0.0f;
static float g_turn_progress_deg = 0.0f;
static float g_turn_current_yaw_deg = 0.0f;
static float g_turn_current_gyro_z_dps = 0.0f;
static float g_turn_out_duty = APP_TURN_DEFAULT_OUT_DUTY;
static float g_turn_in_duty = APP_TURN_DEFAULT_IN_DUTY;
static float g_turn_angle_deg = APP_TURN_DEFAULT_ANGLE_DEG;
static uint32_t g_turn_max_turn_ms = APP_TURN_DEFAULT_MAX_MS;

static float Turn_NormalizeAngle180(float angle_deg);
static float Turn_Clamp(float value, float min_value, float max_value);
static void Turn_ApplyOutput(float scale);

void Turn_Init(void)
{
    g_turn_initialized = 1U;
    g_turn_active = 0U;
    g_turn_last_timeout = 0U;
    g_turn_direction = TURN_DIR_NONE;
    g_turn_last_update_ms = HAL_GetTick();
    g_turn_start_ms = 0U;
    g_turn_start_yaw_deg = 0.0f;
    g_turn_progress_deg = 0.0f;
    g_turn_current_yaw_deg = 0.0f;
    g_turn_current_gyro_z_dps = 0.0f;
    g_turn_out_duty = APP_TURN_DEFAULT_OUT_DUTY;
    g_turn_in_duty = APP_TURN_DEFAULT_IN_DUTY;
    g_turn_angle_deg = APP_TURN_DEFAULT_ANGLE_DEG;
    g_turn_max_turn_ms = APP_TURN_DEFAULT_MAX_MS;
}

void Turn_Poll(void)
{
    uint32_t now_ms;
    uint32_t elapsed_ms;
    float angle_error_deg;
    float scale = 1.0f;

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

    angle_error_deg = g_turn_angle_deg - g_turn_progress_deg;
    elapsed_ms = (uint32_t)(now_ms - g_turn_start_ms);

    if (elapsed_ms >= g_turn_max_turn_ms)
    {
        g_turn_last_timeout = 1U;
        Turn_Stop();
        return;
    }

    if (angle_error_deg <= APP_TURN_DONE_DEG)
    {
        g_turn_last_timeout = 0U;
        Turn_Stop();
        return;
    }

    if (angle_error_deg <= APP_TURN_SLOWDOWN_DEG)
    {
        scale = APP_TURN_SLOW_SCALE;
    }

    Turn_ApplyOutput(scale);
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
    g_turn_direction = direction;
    g_turn_start_ms = HAL_GetTick();
    g_turn_last_update_ms = g_turn_start_ms;
    g_turn_start_yaw_deg = Imu_GetYawDeg();
    g_turn_current_yaw_deg = g_turn_start_yaw_deg;
    g_turn_current_gyro_z_dps = Imu_GetGyroZDps();
    g_turn_progress_deg = 0.0f;
    Turn_ApplyOutput(1.0f);
    return 1U;
}

void Turn_Stop(void)
{
    g_turn_active = 0U;
    g_turn_direction = TURN_DIR_NONE;
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
                   "%s TURN active=%u dir=%s prog=%.1f target=%.1f yaw=%.1f gyro_z=%.1f timeout=%u",
                   prefix,
                   (unsigned int)g_turn_active,
                   dir_text,
                   g_turn_progress_deg,
                   g_turn_angle_deg,
                   g_turn_current_yaw_deg,
                   g_turn_current_gyro_z_dps,
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

static void Turn_ApplyOutput(float scale)
{
    float out_duty;
    float in_duty;

    out_duty = Turn_Clamp(g_turn_out_duty * scale, 0.0f, 100.0f);
    in_duty = Turn_Clamp(g_turn_in_duty * scale, -100.0f, 0.0f);

    if (g_turn_direction == TURN_DIR_LEFT)
    {
        Motion_SetDuty4(in_duty, out_duty, in_duty, out_duty);
    }
    else if (g_turn_direction == TURN_DIR_RIGHT)
    {
        Motion_SetDuty4(out_duty, in_duty, out_duty, in_duty);
    }
}
