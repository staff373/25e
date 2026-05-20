#include "pid_core.h"

/* 私有工具函数声明 */
static uint8_t PID_Core_IsNaN(float value);
static float PID_Core_ClampFloat(float value, float min_value, float max_value);
static float PID_Core_AbsFloat(float value);
static void PID_Core_SanitizeParams(PID_Params_t *params);

void PID_Core_Init(PID_Handle_t *pid, const PID_Params_t *params)
{
    PID_Params_t safe_params;

    if (pid == (PID_Handle_t *)0)
    {
        return;
    }

    safe_params.kp = 0.0f;
    safe_params.ki = 0.0f;
    safe_params.kd = 0.0f;
    safe_params.kf = 0.0f;
    safe_params.integral_min = 0.0f;
    safe_params.integral_max = 0.0f;
    safe_params.output_min = 0.0f;
    safe_params.output_max = 0.0f;
    safe_params.deadband = 0.0f;
    safe_params.deadband_mode = PID_DEADBAND_MODE_CLEAR;

    if (params != (const PID_Params_t *)0)
    {
        safe_params = *params;
    }

    pid->params = safe_params;
    PID_Core_SanitizeParams(&pid->params);
    PID_Core_Reset(pid);
}

void PID_Core_Reset(PID_Handle_t *pid)
{
    if (pid == (PID_Handle_t *)0)
    {
        return;
    }

    pid->state.target = 0.0f;
    pid->state.measurement = 0.0f;
    pid->state.feedforward = 0.0f;

    pid->state.error = 0.0f;
    pid->state.effective_error = 0.0f;
    pid->state.last_error = 0.0f;

    pid->state.integral_sum = 0.0f;

    pid->state.pid_output = 0.0f;
    pid->state.output = 0.0f;

    pid->state.p_out = 0.0f;
    pid->state.i_out = 0.0f;
    pid->state.d_out = 0.0f;
    pid->state.f_out = 0.0f;
}

float PID_Core_Calculate(PID_Handle_t *pid, float target, float measurement, float feedforward)
{
    PID_Params_t *params;
    float error;

    if (pid == (PID_Handle_t *)0)
    {
        return 0.0f;
    }

    if (PID_Core_IsNaN(target) != 0U)
    {
        target = 0.0f;
    }

    if (PID_Core_IsNaN(measurement) != 0U)
    {
        measurement = 0.0f;
    }

    if (PID_Core_IsNaN(feedforward) != 0U)
    {
        feedforward = 0.0f;
    }

    PID_Core_SanitizeParams(&pid->params);
    params = &pid->params;

    pid->state.target = target;
    pid->state.measurement = measurement;
    pid->state.feedforward = feedforward;
    pid->state.integral_sum = PID_Core_ClampFloat(pid->state.integral_sum,
                                                  params->integral_min,
                                                  params->integral_max);

    error = target - measurement;
    pid->state.error = error;

    /*
     * 死区命中时只抑制 PID 项，不屏蔽前馈：
     * - CLEAR：清空 PID 状态，输出只剩当前前馈项
     * - FREEZE：冻结上一拍 PID 输出，仅叠加当前前馈项
     */
    if (PID_Core_AbsFloat(error) <= params->deadband)
    {
        pid->state.effective_error = 0.0f;
        pid->state.f_out = params->kf * feedforward;

        if (params->deadband_mode == PID_DEADBAND_MODE_CLEAR)
        {
            pid->state.last_error = 0.0f;
            pid->state.integral_sum = 0.0f;
            pid->state.p_out = 0.0f;
            pid->state.i_out = 0.0f;
            pid->state.d_out = 0.0f;
            pid->state.pid_output = 0.0f;
            pid->state.output = PID_Core_ClampFloat(pid->state.f_out,
                                                    params->output_min,
                                                    params->output_max);
        }
        else
        {
            pid->state.output = PID_Core_ClampFloat(pid->state.pid_output + pid->state.f_out,
                                                    params->output_min,
                                                    params->output_max);
        }

        return pid->state.output;
    }

    pid->state.effective_error = error;
    pid->state.p_out = params->kp * pid->state.effective_error;

    pid->state.integral_sum += pid->state.effective_error;
    pid->state.integral_sum = PID_Core_ClampFloat(pid->state.integral_sum,
                                                  params->integral_min,
                                                  params->integral_max);
    pid->state.i_out = params->ki * pid->state.integral_sum;

    pid->state.d_out = params->kd * (pid->state.effective_error - pid->state.last_error);
    pid->state.f_out = params->kf * feedforward;

    pid->state.pid_output = pid->state.p_out + pid->state.i_out + pid->state.d_out;
    pid->state.output = PID_Core_ClampFloat(pid->state.pid_output + pid->state.f_out,
                                            params->output_min,
                                            params->output_max);
    pid->state.last_error = pid->state.effective_error;

    return pid->state.output;
}

static uint8_t PID_Core_IsNaN(float value)
{
    uint8_t is_nan = 0U;

    if (value != value)
    {
        is_nan = 1U;
    }

    return is_nan;
}

static float PID_Core_ClampFloat(float value, float min_value, float max_value)
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

static float PID_Core_AbsFloat(float value)
{
    float abs_value = value;

    if (abs_value < 0.0f)
    {
        abs_value = -abs_value;
    }

    return abs_value;
}

static void PID_Core_SanitizeParams(PID_Params_t *params)
{
    float swap_temp;

    if (params == (PID_Params_t *)0)
    {
        return;
    }

    if (PID_Core_IsNaN(params->kp) != 0U)
    {
        params->kp = 0.0f;
    }

    if (PID_Core_IsNaN(params->ki) != 0U)
    {
        params->ki = 0.0f;
    }

    if (PID_Core_IsNaN(params->kd) != 0U)
    {
        params->kd = 0.0f;
    }

    if (PID_Core_IsNaN(params->kf) != 0U)
    {
        params->kf = 0.0f;
    }

    if (PID_Core_IsNaN(params->integral_min) != 0U)
    {
        params->integral_min = 0.0f;
    }

    if (PID_Core_IsNaN(params->integral_max) != 0U)
    {
        params->integral_max = 0.0f;
    }

    if (PID_Core_IsNaN(params->output_min) != 0U)
    {
        params->output_min = 0.0f;
    }

    if (PID_Core_IsNaN(params->output_max) != 0U)
    {
        params->output_max = 0.0f;
    }

    if (PID_Core_IsNaN(params->deadband) != 0U)
    {
        params->deadband = 0.0f;
    }

    if (params->integral_min > params->integral_max)
    {
        swap_temp = params->integral_min;
        params->integral_min = params->integral_max;
        params->integral_max = swap_temp;
    }

    if (params->output_min > params->output_max)
    {
        swap_temp = params->output_min;
        params->output_min = params->output_max;
        params->output_max = swap_temp;
    }

    params->deadband = PID_Core_AbsFloat(params->deadband);

    if ((params->deadband_mode != PID_DEADBAND_MODE_CLEAR) &&
        (params->deadband_mode != PID_DEADBAND_MODE_FREEZE))
    {
        params->deadband_mode = PID_DEADBAND_MODE_CLEAR;
    }
}
