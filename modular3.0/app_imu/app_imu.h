#ifndef APP_IMU_H
#define APP_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include <stddef.h>
#include <stdint.h>

void Imu_Init(void);
void Imu_Poll(void);
void Imu_HandleRxCplt(UART_HandleTypeDef *huart);
void Imu_HandleUartError(UART_HandleTypeDef *huart);
int Imu_GetInitStatus(void);
uint8_t Imu_IsOnline(void);
float Imu_GetYawDeg(void);
float Imu_GetGyroZDps(void);
void Imu_FormatStatus(char *buffer, size_t buffer_size, const char *prefix);

#ifdef __cplusplus
}
#endif

#endif /* APP_IMU_H */
