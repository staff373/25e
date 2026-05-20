/*
 * bsp_jy61p.h
 * JY61P 姿态传感器 BSP 接口（STM32F4 + HAL UART）
 * 说明：
 * 1) 本模块只输出姿态缓存（pitch/roll/yaw，单位 rad）。
 * 2) 本模块内部托管单字节 UART IT 接收，不包含 PID、printf、遥测。
 * 3) 建议在外部 200Hz 调度调用 BSP_JY61P_Update。
 */
#ifndef BSP_JY61P_H
#define BSP_JY61P_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define BSP_JY61P_OK                 (0)
#define BSP_JY61P_ERR_ARG            (-1)
#define BSP_JY61P_ERR_BINDING        (-2)
#define BSP_JY61P_ERR_UART_REINIT    (-3)
#define BSP_JY61P_ERR_UART_TX        (-4)
#define BSP_JY61P_ERR_RX_START       (-5)

typedef struct
{
    /* UART 绑定与运行参数 */
    UART_HandleTypeDef *huart;
    uint32_t baudrate;
    uint32_t offline_timeout_ms;

    /* 单字节接收状态 */
    uint8_t rx_byte;
    uint8_t frame[11];
    uint8_t frame_index;

    /* 对外姿态缓存（单位：rad） */
    float pitch_rad;
    float roll_rad;
    float yaw_rad;

    /* 内部 IMU 缓存 */
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temperature_c;

    /* 运行态状态 */
    uint32_t last_frame_ms;
    uint32_t sample_seq;
    uint8_t initialized;
    uint8_t online;
    uint8_t data_valid;
} BSP_JY61P_t;

int BSP_JY61P_Init(BSP_JY61P_t *imu);
void BSP_JY61P_Update(BSP_JY61P_t *imu);
void BSP_JY61P_HandleRxCplt(BSP_JY61P_t *imu);
void BSP_JY61P_HandleUartError(BSP_JY61P_t *imu);

static inline float BSP_JY61P_GetPitch(const BSP_JY61P_t *imu)
{
    if (imu == (const BSP_JY61P_t *)0)
    {
        return 0.0f;
    }

    return imu->pitch_rad;
}

static inline float BSP_JY61P_GetRoll(const BSP_JY61P_t *imu)
{
    if (imu == (const BSP_JY61P_t *)0)
    {
        return 0.0f;
    }

    return imu->roll_rad;
}

static inline float BSP_JY61P_GetYaw(const BSP_JY61P_t *imu)
{
    if (imu == (const BSP_JY61P_t *)0)
    {
        return 0.0f;
    }

    return imu->yaw_rad;
}

static inline uint8_t BSP_JY61P_IsOnline(const BSP_JY61P_t *imu)
{
    if (imu == (const BSP_JY61P_t *)0)
    {
        return 0U;
    }

    return imu->online;
}

#ifdef __cplusplus
}
#endif

#endif /* BSP_JY61P_H */
