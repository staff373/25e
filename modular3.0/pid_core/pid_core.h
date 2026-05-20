/*
 * pid_core.h
 * 通用 PID 数学内核（纯 C / 平台无关）
 * 说明：
 * 1) 本模块只做位置式 PID、前馈、限幅与死区处理，不包含 HAL/BSP/串口依赖。
 * 2) 通过句柄化设计支持多实例复用，可分别服务于底盘环与云台环。
 * 3) 参数区与状态区公开，便于应用层直接读写和调试。
 */
#ifndef PID_CORE_H
#define PID_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 死区模式：进入死区后的状态保持策略 */
typedef enum
{
    PID_DEADBAND_MODE_CLEAR = 0,
    PID_DEADBAND_MODE_FREEZE = 1
} PID_DeadbandMode_t;

/* PID 参数区：支持应用层直接在线改写 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float kf;

    float integral_min;
    float integral_max;

    float output_min;
    float output_max;

    float deadband;
    PID_DeadbandMode_t deadband_mode;
} PID_Params_t;

/* PID 状态区：便于调试时直接观察每个分量 */
typedef struct
{
    float target;
    float measurement;
    float feedforward;

    float error;
    float effective_error;
    float last_error;

    float integral_sum;

    float pid_output;
    float output;

    float p_out;
    float i_out;
    float d_out;
    float f_out;
} PID_State_t;

/* PID 实例句柄：一条控制链路对应一个 PID_Handle_t */
typedef struct
{
    PID_Params_t params;
    PID_State_t state;
} PID_Handle_t;

/* 生命周期与计算接口 */
void PID_Core_Init(PID_Handle_t *pid, const PID_Params_t *params);
void PID_Core_Reset(PID_Handle_t *pid);
float PID_Core_Calculate(PID_Handle_t *pid, float target, float measurement, float feedforward);

#ifdef __cplusplus
}
#endif

#endif /* PID_CORE_H */
