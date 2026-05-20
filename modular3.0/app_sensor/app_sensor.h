#ifndef APP_SENSOR_H
#define APP_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_sensor.h"

#include <stddef.h>
#include <stdint.h>

void AppSensor_Init(void);
void AppSensor_Poll(void);
void AppSensor_ReadNow(void);
uint8_t AppSensor_GetRawState(void);
void AppSensor_GetRawArray(uint8_t out[SENSOR_CHANNEL_COUNT]);
float AppSensor_GetNormError(void);
Sensor_State_t AppSensor_GetStateType(void);
int8_t AppSensor_GetCornerDirection(void);
uint8_t AppSensor_IsAllWhite(void);
uint8_t AppSensor_IsAllBlack(void);
void AppSensor_FormatStatus(char *buffer, size_t buffer_size, const char *prefix);

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSOR_H */
