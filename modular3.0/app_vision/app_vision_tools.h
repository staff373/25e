#ifndef APP_VISION_TOOLS_H
#define APP_VISION_TOOLS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_vision.h"

typedef enum
{
    VISION_CAPTURE_STATE_IDLE = 0,
    VISION_CAPTURE_STATE_WAIT_ACK,
    VISION_CAPTURE_STATE_AUTO_WAIT,
    VISION_CAPTURE_STATE_AUTO_WAIT_ACK
} Vision_CaptureState_t;

typedef enum
{
    VISION_STREAM_STATE_OFF = 0,
    VISION_STREAM_STATE_READY,
    VISION_STREAM_STATE_DROPPING
} Vision_StreamState_t;

void Vision_ToolsInit(void);
void Vision_ToolsPoll(void);
uint8_t Vision_ToolsTryParseLine(char *line);
void Vision_ToolsOnFrame(const Vision_Target_t *target, const char *raw_line);

uint8_t Vision_CaptureRequest(uint32_t *request_id);
uint8_t Vision_CaptureAutoStart(uint32_t interval_ms);
void Vision_CaptureAutoStop(void);
uint8_t Vision_CaptureTakeNotice(char *buffer, size_t buffer_size);
Vision_CaptureState_t Vision_CaptureGetState(void);
const char *Vision_CaptureGetStateName(void);
void Vision_CaptureFormatStatus(char *buffer, size_t buffer_size, const char *prefix);

void Vision_StreamSetEnabled(uint8_t enabled);
uint8_t Vision_StreamIsEnabled(void);
Vision_StreamState_t Vision_StreamGetState(void);
const char *Vision_StreamGetStateName(void);
uint8_t Vision_StreamTakeLine(char *buffer, size_t buffer_size);
void Vision_StreamFormatStatus(char *buffer, size_t buffer_size, const char *prefix);

#ifdef __cplusplus
}
#endif

#endif /* APP_VISION_TOOLS_H */
