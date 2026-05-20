#ifndef APP_MOTION_H
#define APP_MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define APP_MOTION_WHEEL_COUNT (4U)

typedef enum
{
    MOTION_WHEEL_LF = 0,
    MOTION_WHEEL_RF = 1,
    MOTION_WHEEL_LB = 2,
    MOTION_WHEEL_RB = 3
} Motion_WheelId_t;

void Motion_Init(void);
void Motion_Poll(void);
void Motion_SetDuty4(float lf, float rf, float lb, float rb);
void Motion_Stop(void);
float Motion_GetDuty(uint8_t wheel_id);
int32_t Motion_GetEncoderAccum(uint8_t wheel_id);
int32_t Motion_GetEncoderDelta(uint8_t wheel_id);

#ifdef __cplusplus
}
#endif

#endif /* APP_MOTION_H */
