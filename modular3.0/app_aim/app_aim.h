#ifndef APP_AIM_H
#define APP_AIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#include "app_track.h"

typedef enum
{
    AIM_STATE_IDLE = 0,
    AIM_STATE_ONCE,
    AIM_STATE_TRACKING,
    AIM_STATE_LOCK_VERIFY,
    AIM_STATE_Q3_SCAN_REV,
    AIM_STATE_Q3_SCAN_SETTLE,
    AIM_STATE_Q3_STABLE_CONFIRM,
    AIM_STATE_Q3_AIM,
    AIM_STATE_Q3_LOCK_VERIFY,
    AIM_STATE_LOCKED,
    AIM_STATE_ERROR
} Aim_State_t;

void Aim_Init(void);
void Aim_Poll(void);
uint8_t Aim_StartOnce(uint32_t timeout_ms);
uint8_t Aim_StartTrack(void);
uint8_t Aim_StartQuestion3(uint32_t timeout_ms);
void Aim_Stop(void);
void Aim_UpdateTrackHint(Track_State_t track_state,
                         int8_t turn_dir,
                         float turn_progress_deg,
                         float turn_gyro_z_dps);
Aim_State_t Aim_GetState(void);
const char *Aim_GetStateName(void);
uint8_t Aim_SetParam(const char *name, float value);
uint8_t Aim_GetParam(const char *name, float *value);
void Aim_FormatStatus(char *buffer, size_t buffer_size, const char *prefix);

#ifdef __cplusplus
}
#endif

#endif /* APP_AIM_H */
