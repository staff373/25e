#ifndef APP_TRACK_H
#define APP_TRACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    TRACK_STATE_IDLE = 0,
    TRACK_STATE_LINE_FOLLOW,
    TRACK_STATE_CORNER_DETECT,
    TRACK_STATE_TURNING,
    TRACK_STATE_RECOVER_LINE,
    TRACK_STATE_FINISHED,
    TRACK_STATE_STOPPED
} Track_State_t;

typedef enum
{
    TRACK_STOP_REASON_NONE = 0,
    TRACK_STOP_REASON_USER,
    TRACK_STOP_REASON_COMPLETE,
    TRACK_STOP_REASON_TURN_TIMEOUT,
    TRACK_STOP_REASON_TURN_START_FAIL,
    TRACK_STOP_REASON_LOST_LINE
} Track_StopReason_t;

void Track_Init(void);
void Track_Poll(void);
uint8_t Track_Start(void);
void Track_Stop(void);
Track_State_t Track_GetState(void);
const char *Track_GetStateName(void);
Track_StopReason_t Track_GetStopReason(void);
const char *Track_GetStopReasonName(void);
uint8_t Track_IsRunning(void);
uint8_t Track_SetTargetLaps(uint8_t laps);
uint8_t Track_GetTargetLaps(void);
uint8_t Track_GetLapsDone(void);
uint8_t Track_GetCornerCount(void);
uint32_t Track_GetElapsedMs(void);
uint8_t Track_SetParam(const char *name, float value);
uint8_t Track_GetParam(const char *name, float *value);
void Track_FormatStatus(char *buffer, size_t buffer_size, const char *prefix);

#ifdef __cplusplus
}
#endif

#endif /* APP_TRACK_H */
