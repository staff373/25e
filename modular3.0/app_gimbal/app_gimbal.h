#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    GIMBAL_STATE_IDLE = 0,
    GIMBAL_STATE_MOVING,
    GIMBAL_STATE_STOPPING,
    GIMBAL_STATE_ERROR
} Gimbal_State_t;

void Gimbal_Init(void);
void Gimbal_Poll(void);

uint8_t Gimbal_MoveRelativeSteps(int32_t x_steps, int32_t y_steps, float speed_sps);
uint8_t Gimbal_MoveByPixelError(int16_t dx, int16_t dy);
void Gimbal_Zero(void);
void Gimbal_Stop(void);
void Gimbal_EStop(void);
void Gimbal_SetHoldEnabled(uint8_t enabled);
uint8_t Gimbal_IsBusy(void);
uint8_t Gimbal_IsCalibrated(void);
void Gimbal_SetCalibration(float a, float b, float c, float d);
void Gimbal_GetCalibration(float *a, float *b, float *c, float *d);
Gimbal_State_t Gimbal_GetState(void);
const char *Gimbal_GetStateName(void);
void Gimbal_FormatStatus(char *buffer, size_t buffer_size, const char *prefix);

#ifdef __cplusplus
}
#endif

#endif /* APP_GIMBAL_H */
