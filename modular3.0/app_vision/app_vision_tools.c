#include "app_vision_tools.h"

#include "app_vision_config.h"
#include "usart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define APP_VISION_CAPTURE_TX_TIMEOUT_MS  (10U)
#define APP_VISION_CAPTURE_ACK_TIMEOUT_MS (2000U)
#define APP_VISION_CAPTURE_NOTICE_SIZE    (128U)
#define APP_VISION_CAPTURE_TOKEN_SIZE     (32U)
#define APP_VISION_STREAM_LINE_SIZE       (192U)

#if APP_VISION_ENABLE_CAPTURE
static Vision_CaptureState_t g_capture_state = VISION_CAPTURE_STATE_IDLE;
static uint32_t g_capture_next_id = 1U;
static uint32_t g_capture_pending_id = 0U;
static uint32_t g_capture_pending_ms = 0U;
static uint32_t g_capture_auto_interval_ms = 0U;
static uint32_t g_capture_last_trigger_ms = 0U;
static uint32_t g_capture_last_id = 0U;
static uint8_t g_capture_auto_enabled = 0U;
static uint8_t g_capture_last_ok = 0U;
static uint8_t g_capture_notice_pending = 0U;
static char g_capture_last_file[APP_VISION_CAPTURE_TOKEN_SIZE] = "-";
static char g_capture_last_error[APP_VISION_CAPTURE_TOKEN_SIZE] = "-";
static char g_capture_notice[APP_VISION_CAPTURE_NOTICE_SIZE];

static void Vision_CaptureInit(void);
static void Vision_CapturePoll(void);
static uint8_t Vision_CaptureStartRequest(uint8_t auto_request, uint32_t *request_id);
static void Vision_CaptureComplete(uint32_t request_id, uint8_t ok, const char *detail);
static void Vision_CaptureSetNotice(uint8_t ok, uint32_t request_id, const char *detail);
static uint8_t Vision_CaptureParseAck(char *line);
static uint8_t Vision_CaptureParseId(char **cursor, uint32_t *request_id);
static uint8_t Vision_CaptureReadToken(char **cursor, char *token, size_t token_size);
static void Vision_CaptureCopyToken(char *dest, size_t dest_size, const char *src);
static uint32_t Vision_CaptureElapsedMs(void);
#endif

#if APP_VISION_ENABLE_STREAM
static Vision_StreamState_t g_stream_state = VISION_STREAM_STATE_OFF;
static uint8_t g_stream_line_pending = 0U;
static uint32_t g_stream_last_frame_ms = 0U;
static uint32_t g_stream_last_dt_ms = 0U;
static uint32_t g_stream_last_hz_x10 = 0U;
static uint32_t g_stream_sent = 0U;
static uint32_t g_stream_drop = 0U;
static uint16_t g_stream_last_seq = 0U;
static char g_stream_line[APP_VISION_STREAM_LINE_SIZE];

static void Vision_StreamInit(void);
static void Vision_StreamOnFrame(const Vision_Target_t *target, const char *raw_line);
#endif

void Vision_ToolsInit(void)
{
#if APP_VISION_ENABLE_CAPTURE
    Vision_CaptureInit();
#endif
#if APP_VISION_ENABLE_STREAM
    Vision_StreamInit();
#endif
}

void Vision_ToolsPoll(void)
{
#if APP_VISION_ENABLE_CAPTURE
    Vision_CapturePoll();
#endif
}

uint8_t Vision_ToolsTryParseLine(char *line)
{
#if APP_VISION_ENABLE_CAPTURE
    return Vision_CaptureParseAck(line);
#else
    (void)line;
    return 0U;
#endif
}

void Vision_ToolsOnFrame(const Vision_Target_t *target, const char *raw_line)
{
#if APP_VISION_ENABLE_STREAM
    Vision_StreamOnFrame(target, raw_line);
#else
    (void)target;
    (void)raw_line;
#endif
}

#if APP_VISION_ENABLE_CAPTURE
uint8_t Vision_CaptureRequest(uint32_t *request_id)
{
    return Vision_CaptureStartRequest(0U, request_id);
}

uint8_t Vision_CaptureAutoStart(uint32_t interval_ms)
{
    if ((g_capture_state == VISION_CAPTURE_STATE_WAIT_ACK) ||
        (g_capture_state == VISION_CAPTURE_STATE_AUTO_WAIT_ACK))
    {
        return 0U;
    }

    if (interval_ms < 50U)
    {
        interval_ms = 50U;
    }

    g_capture_auto_enabled = 1U;
    g_capture_auto_interval_ms = interval_ms;
    g_capture_last_trigger_ms = (uint32_t)(HAL_GetTick() - interval_ms);
    g_capture_state = VISION_CAPTURE_STATE_AUTO_WAIT;
    return 1U;
}

void Vision_CaptureAutoStop(void)
{
    g_capture_auto_enabled = 0U;
    g_capture_auto_interval_ms = 0U;

    if (g_capture_state == VISION_CAPTURE_STATE_AUTO_WAIT)
    {
        g_capture_state = VISION_CAPTURE_STATE_IDLE;
    }
    else if (g_capture_state == VISION_CAPTURE_STATE_AUTO_WAIT_ACK)
    {
        g_capture_state = VISION_CAPTURE_STATE_WAIT_ACK;
    }
}

uint8_t Vision_CaptureTakeNotice(char *buffer, size_t buffer_size)
{
    size_t length;

    if ((buffer == (char *)0) || (buffer_size == 0U) || (g_capture_notice_pending == 0U))
    {
        return 0U;
    }

    length = strlen(g_capture_notice);
    if (length >= buffer_size)
    {
        length = buffer_size - 1U;
    }

    (void)memcpy(buffer, g_capture_notice, length);
    buffer[length] = '\0';
    g_capture_notice_pending = 0U;
    return 1U;
}

Vision_CaptureState_t Vision_CaptureGetState(void)
{
    return g_capture_state;
}

const char *Vision_CaptureGetStateName(void)
{
    switch (g_capture_state)
    {
    case VISION_CAPTURE_STATE_IDLE:
        return "IDLE";
    case VISION_CAPTURE_STATE_WAIT_ACK:
        return "WAIT_ACK";
    case VISION_CAPTURE_STATE_AUTO_WAIT:
        return "AUTO_WAIT";
    case VISION_CAPTURE_STATE_AUTO_WAIT_ACK:
        return "AUTO_WAIT_ACK";
    default:
        return "UNKNOWN";
    }
}

void Vision_CaptureFormatStatus(char *buffer, size_t buffer_size, const char *prefix)
{
    if ((buffer == (char *)0) || (buffer_size == 0U))
    {
        return;
    }

    if (prefix == (const char *)0)
    {
        prefix = "OK";
    }

    (void)snprintf(buffer,
                   buffer_size,
                   "%s CAP state=%s auto=%u interval=%lu pending=%lu next=%lu last_id=%lu last_ok=%u file=%s err=%s",
                   prefix,
                   Vision_CaptureGetStateName(),
                   (unsigned int)g_capture_auto_enabled,
                   (unsigned long)g_capture_auto_interval_ms,
                   (unsigned long)g_capture_pending_id,
                   (unsigned long)g_capture_next_id,
                   (unsigned long)g_capture_last_id,
                   (unsigned int)g_capture_last_ok,
                   g_capture_last_file,
                   g_capture_last_error);
}
#else
uint8_t Vision_CaptureRequest(uint32_t *request_id)
{
    if (request_id != (uint32_t *)0)
    {
        *request_id = 0U;
    }
    return 0U;
}

uint8_t Vision_CaptureAutoStart(uint32_t interval_ms)
{
    (void)interval_ms;
    return 0U;
}

void Vision_CaptureAutoStop(void)
{
}

uint8_t Vision_CaptureTakeNotice(char *buffer, size_t buffer_size)
{
    (void)buffer;
    (void)buffer_size;
    return 0U;
}

Vision_CaptureState_t Vision_CaptureGetState(void)
{
    return VISION_CAPTURE_STATE_IDLE;
}

const char *Vision_CaptureGetStateName(void)
{
    return "DISABLED";
}

void Vision_CaptureFormatStatus(char *buffer, size_t buffer_size, const char *prefix)
{
    if ((buffer == (char *)0) || (buffer_size == 0U))
    {
        return;
    }

    (void)snprintf(buffer, buffer_size, "%s CAP state=DISABLED", (prefix != (const char *)0) ? prefix : "ERR");
}
#endif

#if APP_VISION_ENABLE_STREAM
void Vision_StreamSetEnabled(uint8_t enabled)
{
    if (enabled != 0U)
    {
        Vision_StreamInit();
        g_stream_state = VISION_STREAM_STATE_READY;
        return;
    }

    g_stream_state = VISION_STREAM_STATE_OFF;
    g_stream_line_pending = 0U;
    g_stream_line[0] = '\0';
}

uint8_t Vision_StreamIsEnabled(void)
{
    return (uint8_t)(g_stream_state != VISION_STREAM_STATE_OFF);
}

Vision_StreamState_t Vision_StreamGetState(void)
{
    return g_stream_state;
}

const char *Vision_StreamGetStateName(void)
{
    switch (g_stream_state)
    {
    case VISION_STREAM_STATE_OFF:
        return "OFF";
    case VISION_STREAM_STATE_READY:
        return "READY";
    case VISION_STREAM_STATE_DROPPING:
        return "DROPPING";
    default:
        return "UNKNOWN";
    }
}

uint8_t Vision_StreamTakeLine(char *buffer, size_t buffer_size)
{
    size_t length;

    if ((buffer == (char *)0) || (buffer_size == 0U) || (g_stream_line_pending == 0U))
    {
        return 0U;
    }

    length = strlen(g_stream_line);
    if (length >= buffer_size)
    {
        length = buffer_size - 1U;
    }

    (void)memcpy(buffer, g_stream_line, length);
    buffer[length] = '\0';
    g_stream_line_pending = 0U;
    g_stream_sent++;
    if (g_stream_state == VISION_STREAM_STATE_DROPPING)
    {
        g_stream_state = VISION_STREAM_STATE_READY;
    }
    return 1U;
}

void Vision_StreamFormatStatus(char *buffer, size_t buffer_size, const char *prefix)
{
    if ((buffer == (char *)0) || (buffer_size == 0U))
    {
        return;
    }

    if (prefix == (const char *)0)
    {
        prefix = "OK";
    }

    (void)snprintf(buffer,
                   buffer_size,
                   "%s VSTREAM state=%s en=%u pending=%u sent=%lu drop=%lu seq=%u dt=%lu hz=%lu.%lu",
                   prefix,
                   Vision_StreamGetStateName(),
                   (unsigned int)Vision_StreamIsEnabled(),
                   (unsigned int)g_stream_line_pending,
                   (unsigned long)g_stream_sent,
                   (unsigned long)g_stream_drop,
                   (unsigned int)g_stream_last_seq,
                   (unsigned long)g_stream_last_dt_ms,
                   (unsigned long)(g_stream_last_hz_x10 / 10U),
                   (unsigned long)(g_stream_last_hz_x10 % 10U));
}
#else
void Vision_StreamSetEnabled(uint8_t enabled)
{
    (void)enabled;
}

uint8_t Vision_StreamIsEnabled(void)
{
    return 0U;
}

Vision_StreamState_t Vision_StreamGetState(void)
{
    return VISION_STREAM_STATE_OFF;
}

const char *Vision_StreamGetStateName(void)
{
    return "DISABLED";
}

uint8_t Vision_StreamTakeLine(char *buffer, size_t buffer_size)
{
    (void)buffer;
    (void)buffer_size;
    return 0U;
}

void Vision_StreamFormatStatus(char *buffer, size_t buffer_size, const char *prefix)
{
    if ((buffer == (char *)0) || (buffer_size == 0U))
    {
        return;
    }

    (void)snprintf(buffer, buffer_size, "%s VSTREAM state=DISABLED en=0", (prefix != (const char *)0) ? prefix : "ERR");
}
#endif

#if APP_VISION_ENABLE_CAPTURE
static void Vision_CaptureInit(void)
{
    g_capture_state = VISION_CAPTURE_STATE_IDLE;
    g_capture_next_id = 1U;
    g_capture_pending_id = 0U;
    g_capture_pending_ms = 0U;
    g_capture_auto_interval_ms = 0U;
    g_capture_last_trigger_ms = 0U;
    g_capture_last_id = 0U;
    g_capture_auto_enabled = 0U;
    g_capture_last_ok = 0U;
    g_capture_notice_pending = 0U;
    Vision_CaptureCopyToken(g_capture_last_file, sizeof(g_capture_last_file), "-");
    Vision_CaptureCopyToken(g_capture_last_error, sizeof(g_capture_last_error), "-");
    g_capture_notice[0] = '\0';
}

static void Vision_CapturePoll(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t request_id;

    if ((g_capture_state == VISION_CAPTURE_STATE_WAIT_ACK) ||
        (g_capture_state == VISION_CAPTURE_STATE_AUTO_WAIT_ACK))
    {
        if (Vision_CaptureElapsedMs() > APP_VISION_CAPTURE_ACK_TIMEOUT_MS)
        {
            Vision_CaptureComplete(g_capture_pending_id, 0U, "timeout");
        }
        return;
    }

    if ((g_capture_auto_enabled != 0U) &&
        (g_capture_state == VISION_CAPTURE_STATE_AUTO_WAIT) &&
        ((uint32_t)(now - g_capture_last_trigger_ms) >= g_capture_auto_interval_ms))
    {
        g_capture_last_trigger_ms = now;
        if (Vision_CaptureStartRequest(1U, &request_id) == 0U)
        {
            g_capture_last_id = g_capture_next_id;
            g_capture_last_ok = 0U;
            Vision_CaptureCopyToken(g_capture_last_error, sizeof(g_capture_last_error), "tx_failed");
            Vision_CaptureSetNotice(0U, g_capture_last_id, g_capture_last_error);
        }
    }
}

static uint8_t Vision_CaptureStartRequest(uint8_t auto_request, uint32_t *request_id)
{
    char command[32];
    uint32_t id;
    HAL_StatusTypeDef status;

    if ((g_capture_state == VISION_CAPTURE_STATE_WAIT_ACK) ||
        (g_capture_state == VISION_CAPTURE_STATE_AUTO_WAIT_ACK))
    {
        return 0U;
    }

    id = g_capture_next_id;
    (void)snprintf(command, sizeof(command), "CAP %lu\n", (unsigned long)id);
    status = HAL_UART_Transmit(&huart2,
                               (uint8_t *)command,
                               (uint16_t)strlen(command),
                               APP_VISION_CAPTURE_TX_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        Vision_CaptureCopyToken(g_capture_last_error, sizeof(g_capture_last_error), "tx_failed");
        return 0U;
    }

    g_capture_next_id++;
    g_capture_pending_id = id;
    g_capture_pending_ms = HAL_GetTick();
    g_capture_state = ((auto_request != 0U) || (g_capture_auto_enabled != 0U)) ?
                          VISION_CAPTURE_STATE_AUTO_WAIT_ACK :
                          VISION_CAPTURE_STATE_WAIT_ACK;

    if (request_id != (uint32_t *)0)
    {
        *request_id = id;
    }

    return 1U;
}

static uint8_t Vision_CaptureParseAck(char *line)
{
    char *cursor;
    uint32_t request_id;
    char detail[APP_VISION_CAPTURE_TOKEN_SIZE];

    if (line == (char *)0)
    {
        return 0U;
    }

    if (strncmp(line, "CAP OK ", 7U) == 0)
    {
        cursor = line + 7;
        if ((Vision_CaptureParseId(&cursor, &request_id) == 0U) ||
            (Vision_CaptureReadToken(&cursor, detail, sizeof(detail)) == 0U))
        {
            return 1U;
        }

        Vision_CaptureComplete(request_id, 1U, detail);
        return 1U;
    }

    if (strncmp(line, "CAP ERR ", 8U) == 0)
    {
        cursor = line + 8;
        if ((Vision_CaptureParseId(&cursor, &request_id) == 0U) ||
            (Vision_CaptureReadToken(&cursor, detail, sizeof(detail)) == 0U))
        {
            return 1U;
        }

        Vision_CaptureComplete(request_id, 0U, detail);
        return 1U;
    }

    return 0U;
}

static void Vision_CaptureComplete(uint32_t request_id, uint8_t ok, const char *detail)
{
    if (((g_capture_state == VISION_CAPTURE_STATE_WAIT_ACK) ||
         (g_capture_state == VISION_CAPTURE_STATE_AUTO_WAIT_ACK)) &&
        (request_id != g_capture_pending_id))
    {
        Vision_CaptureCopyToken(g_capture_last_error, sizeof(g_capture_last_error), "id_mismatch");
        Vision_CaptureSetNotice(0U, request_id, g_capture_last_error);
        return;
    }

    g_capture_last_id = request_id;
    g_capture_last_ok = (ok != 0U) ? 1U : 0U;

    if (ok != 0U)
    {
        Vision_CaptureCopyToken(g_capture_last_file, sizeof(g_capture_last_file), detail);
        Vision_CaptureCopyToken(g_capture_last_error, sizeof(g_capture_last_error), "-");
    }
    else
    {
        Vision_CaptureCopyToken(g_capture_last_error, sizeof(g_capture_last_error), detail);
    }

    g_capture_pending_id = 0U;
    g_capture_pending_ms = 0U;
    Vision_CaptureSetNotice(ok, request_id, detail);

    if (g_capture_auto_enabled != 0U)
    {
        g_capture_state = VISION_CAPTURE_STATE_AUTO_WAIT;
        g_capture_last_trigger_ms = HAL_GetTick();
    }
    else
    {
        g_capture_state = VISION_CAPTURE_STATE_IDLE;
    }
}

static void Vision_CaptureSetNotice(uint8_t ok, uint32_t request_id, const char *detail)
{
    if (detail == (const char *)0)
    {
        detail = "-";
    }

    if (ok != 0U)
    {
        (void)snprintf(g_capture_notice,
                       sizeof(g_capture_notice),
                       "OK CAP DONE id=%lu file=%s",
                       (unsigned long)request_id,
                       detail);
    }
    else
    {
        (void)snprintf(g_capture_notice,
                       sizeof(g_capture_notice),
                       "ERR CAP DONE id=%lu reason=%s",
                       (unsigned long)request_id,
                       detail);
    }

    g_capture_notice_pending = 1U;
}

static uint8_t Vision_CaptureParseId(char **cursor, uint32_t *request_id)
{
    char *end;
    unsigned long value;

    if ((cursor == (char **)0) || (*cursor == (char *)0) || (request_id == (uint32_t *)0))
    {
        return 0U;
    }

    value = strtoul(*cursor, &end, 10);
    if (end == *cursor)
    {
        return 0U;
    }

    if ((*end != ' ') && (*end != '\t') && (*end != '\0'))
    {
        return 0U;
    }

    while ((*end == ' ') || (*end == '\t'))
    {
        end++;
    }

    *request_id = (uint32_t)value;
    *cursor = end;
    return 1U;
}

static uint8_t Vision_CaptureReadToken(char **cursor, char *token, size_t token_size)
{
    size_t i = 0U;
    char *src;

    if ((cursor == (char **)0) || (*cursor == (char *)0) ||
        (token == (char *)0) || (token_size == 0U))
    {
        return 0U;
    }

    src = *cursor;
    while ((*src == ' ') || (*src == '\t'))
    {
        src++;
    }

    if (*src == '\0')
    {
        return 0U;
    }

    while ((*src != '\0') && (*src != ' ') && (*src != '\t') && (i < (token_size - 1U)))
    {
        token[i++] = *src++;
    }
    token[i] = '\0';
    *cursor = src;
    return (uint8_t)(i > 0U);
}

static void Vision_CaptureCopyToken(char *dest, size_t dest_size, const char *src)
{
    size_t length;

    if ((dest == (char *)0) || (dest_size == 0U))
    {
        return;
    }

    if (src == (const char *)0)
    {
        src = "-";
    }

    length = strlen(src);
    if (length >= dest_size)
    {
        length = dest_size - 1U;
    }

    (void)memcpy(dest, src, length);
    dest[length] = '\0';
}

static uint32_t Vision_CaptureElapsedMs(void)
{
    if (g_capture_pending_ms == 0U)
    {
        return 0U;
    }

    return (uint32_t)(HAL_GetTick() - g_capture_pending_ms);
}
#endif

#if APP_VISION_ENABLE_STREAM
static void Vision_StreamInit(void)
{
    g_stream_state = VISION_STREAM_STATE_OFF;
    g_stream_line_pending = 0U;
    g_stream_last_frame_ms = 0U;
    g_stream_last_dt_ms = 0U;
    g_stream_last_hz_x10 = 0U;
    g_stream_sent = 0U;
    g_stream_drop = 0U;
    g_stream_last_seq = 0U;
    g_stream_line[0] = '\0';
}

static void Vision_StreamOnFrame(const Vision_Target_t *target, const char *raw_line)
{
    uint32_t now;

    if ((Vision_StreamIsEnabled() == 0U) || (target == (const Vision_Target_t *)0))
    {
        return;
    }

    now = HAL_GetTick();
    if (g_stream_last_frame_ms != 0U)
    {
        g_stream_last_dt_ms = (uint32_t)(now - g_stream_last_frame_ms);
        if (g_stream_last_dt_ms != 0U)
        {
            g_stream_last_hz_x10 = (uint32_t)(10000U / g_stream_last_dt_ms);
        }
        else
        {
            g_stream_last_hz_x10 = 0U;
        }
    }
    else
    {
        g_stream_last_dt_ms = 0U;
        g_stream_last_hz_x10 = 0U;
    }
    g_stream_last_frame_ms = now;
    g_stream_last_seq = target->seq;

    if (raw_line == (const char *)0)
    {
        raw_line = "-";
    }

    if (g_stream_line_pending != 0U)
    {
        g_stream_drop++;
        g_stream_state = VISION_STREAM_STATE_DROPPING;
    }
    else if (g_stream_state != VISION_STREAM_STATE_DROPPING)
    {
        g_stream_state = VISION_STREAM_STATE_READY;
    }

    (void)snprintf(g_stream_line,
                   sizeof(g_stream_line),
                   "OK VRAW dt=%lu hz=%lu.%lu seq=%u valid=%u x=%d y=%d dx=%d dy=%d area=%u drop=%lu line=%s",
                   (unsigned long)g_stream_last_dt_ms,
                   (unsigned long)(g_stream_last_hz_x10 / 10U),
                   (unsigned long)(g_stream_last_hz_x10 % 10U),
                   (unsigned int)target->seq,
                   (unsigned int)target->valid,
                   (int)target->x,
                   (int)target->y,
                   (int)target->dx,
                   (int)target->dy,
                   (unsigned int)target->area,
                   (unsigned long)g_stream_drop,
                   raw_line);
    g_stream_line_pending = 1U;
}
#endif
