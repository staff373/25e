#ifndef APP_VISION_H
#define APP_VISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    VISION_STATE_DISABLED = 0,
    VISION_STATE_LISTENING,
    VISION_STATE_LINE_PENDING,
    VISION_STATE_TARGET_VALID,
    VISION_STATE_NO_TARGET,
    VISION_STATE_TIMEOUT
} Vision_State_t;

typedef struct
{
    uint16_t seq;
    uint8_t valid;
    int16_t x;
    int16_t y;
    int16_t dx;
    int16_t dy;
    uint16_t area;
    uint32_t age_ms;
    uint32_t frames_ok;
    uint32_t frames_bad;
    uint32_t rx_total;
} Vision_Target_t;

void Vision_Init(void);
void Vision_Poll(void);
void Vision_HandleRxCplt(UART_HandleTypeDef *huart);
void Vision_HandleUartError(UART_HandleTypeDef *huart);
void Vision_SetEnabled(uint8_t enabled);
uint8_t Vision_IsEnabled(void);
uint8_t Vision_IsOnline(void);
uint8_t Vision_GetTarget(Vision_Target_t *target);
Vision_State_t Vision_GetState(void);
const char *Vision_GetStateName(void);
void Vision_FormatStatus(char *buffer, size_t buffer_size, const char *prefix);

#ifdef __cplusplus
}
#endif

#endif /* APP_VISION_H */
