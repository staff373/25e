#ifndef APP_AIM_H
#define APP_AIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    AIM_STATE_IDLE = 0,
    AIM_STATE_ONCE,
    AIM_STATE_TRACKING,
    AIM_STATE_LOCKED,
    AIM_STATE_ERROR
} Aim_State_t;

void Aim_Init(void);
void Aim_Poll(void);
uint8_t Aim_StartOnce(uint32_t timeout_ms);
uint8_t Aim_StartTrack(void);
void Aim_Stop(void);
Aim_State_t Aim_GetState(void);
const char *Aim_GetStateName(void);
void Aim_FormatStatus(char *buffer, size_t buffer_size, const char *prefix);

#ifdef __cplusplus
}
#endif

#endif /* APP_AIM_H */
