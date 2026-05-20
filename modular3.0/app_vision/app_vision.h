#ifndef APP_VISION_H
#define APP_VISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include <stddef.h>
#include <stdint.h>

void Vision_Init(void);
void Vision_Poll(void);
void Vision_HandleRxCplt(UART_HandleTypeDef *huart);
void Vision_HandleUartError(UART_HandleTypeDef *huart);
void Vision_SetEnabled(uint8_t enabled);
uint8_t Vision_IsEnabled(void);
uint8_t Vision_IsOnline(void);
void Vision_FormatStatus(char *buffer, size_t buffer_size, const char *prefix);

#ifdef __cplusplus
}
#endif

#endif /* APP_VISION_H */
