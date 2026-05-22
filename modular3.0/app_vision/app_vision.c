#include "app_vision.h"

#include "usart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define APP_VISION_ONLINE_TIMEOUT_MS (1000U)
#define APP_VISION_LINE_BUF_SIZE     (128U)
#define APP_VISION_FIELD_COUNT       (7U)
#define APP_VISION_CAPTURE_TX_TIMEOUT_MS  (10U)
#define APP_VISION_CAPTURE_ACK_TIMEOUT_MS (2000U)
#define APP_VISION_CAPTURE_NOTICE_SIZE    (128U)
#define APP_VISION_CAPTURE_TOKEN_SIZE     (32U)

static uint8_t g_vision_enabled = 1U;
static uint8_t g_vision_listening = 0U;
static uint8_t g_vision_rx_byte = 0U;
static uint32_t g_vision_rx_total = 0U;
static uint32_t g_vision_last_rx_ms = 0U;
static uint32_t g_vision_frames_ok = 0U;
static uint32_t g_vision_frames_bad = 0U;
static Vision_State_t g_vision_state = VISION_STATE_DISABLED;
static Vision_Target_t g_vision_target;

static char g_vision_build_buf[APP_VISION_LINE_BUF_SIZE];
static uint16_t g_vision_build_len = 0U;
static uint8_t g_vision_build_overflow = 0U;
static volatile uint8_t g_vision_line_pending = 0U;
static char g_vision_pending_line[APP_VISION_LINE_BUF_SIZE];
static char g_vision_last_line[APP_VISION_LINE_BUF_SIZE];

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

static void Vision_StartReceive(void);
static void Vision_FinalizeLine(void);
static void Vision_ResetLineBuffers(void);
static void Vision_ProcessPendingLine(void);
static uint8_t Vision_ParseFrame(char *line);
static uint8_t Vision_ParseCaptureAck(char *line);
static uint8_t Vision_ChecksumIsValid(const char *line, char **payload_end);
static int8_t Vision_HexNibble(char ch);
static uint8_t Vision_ParseFields(char *payload, long fields[APP_VISION_FIELD_COUNT]);
static uint8_t Vision_ValueInRange(long value, long min_value, long max_value);
static uint32_t Vision_GetAgeMs(void);
static const char *Vision_GetStatusLine(void);
static void Vision_CaptureInit(void);
static void Vision_CapturePoll(void);
static uint8_t Vision_CaptureStartRequest(uint8_t auto_request, uint32_t *request_id);
static void Vision_CaptureComplete(uint32_t request_id, uint8_t ok, const char *detail);
static void Vision_CaptureSetNotice(uint8_t ok, uint32_t request_id, const char *detail);
static uint8_t Vision_CaptureParseId(char **cursor, uint32_t *request_id);
static uint8_t Vision_CaptureReadToken(char **cursor, char *token, size_t token_size);
static void Vision_CaptureCopyToken(char *dest, size_t dest_size, const char *src);
static uint32_t Vision_CaptureElapsedMs(void);

void Vision_Init(void)
{
    g_vision_enabled = 1U;
    g_vision_listening = 0U;
    g_vision_rx_byte = 0U;
    g_vision_rx_total = 0U;
    g_vision_last_rx_ms = 0U;
    g_vision_frames_ok = 0U;
    g_vision_frames_bad = 0U;
    g_vision_state = VISION_STATE_LISTENING;
    (void)memset(&g_vision_target, 0, sizeof(g_vision_target));
    Vision_ResetLineBuffers();
    Vision_CaptureInit();
    Vision_StartReceive();
}

void Vision_Poll(void)
{
    if (g_vision_enabled == 0U)
    {
        g_vision_state = VISION_STATE_DISABLED;
        return;
    }

    if (g_vision_listening == 0U)
    {
        Vision_StartReceive();
    }

    Vision_ProcessPendingLine();
    Vision_CapturePoll();

    if ((g_vision_last_rx_ms != 0U) &&
        ((uint32_t)(HAL_GetTick() - g_vision_last_rx_ms) > APP_VISION_ONLINE_TIMEOUT_MS))
    {
        g_vision_state = VISION_STATE_TIMEOUT;
    }
}

void Vision_HandleRxCplt(UART_HandleTypeDef *huart)
{
    uint8_t byte;

    if (huart != &huart2)
    {
        return;
    }

    g_vision_listening = 0U;
    if (g_vision_enabled == 0U)
    {
        return;
    }

    byte = g_vision_rx_byte;
    g_vision_rx_total++;
    g_vision_last_rx_ms = HAL_GetTick();

    if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n'))
    {
        Vision_FinalizeLine();
    }
    else if ((byte >= 32U) && (byte <= 126U))
    {
        if (g_vision_build_overflow == 0U)
        {
            if (g_vision_build_len < (APP_VISION_LINE_BUF_SIZE - 1U))
            {
                g_vision_build_buf[g_vision_build_len++] = (char)byte;
                g_vision_build_buf[g_vision_build_len] = '\0';
            }
            else
            {
                g_vision_build_overflow = 1U;
                g_vision_build_len = 0U;
                g_vision_build_buf[0] = '\0';
            }
        }
    }

    Vision_StartReceive();
}

void Vision_HandleUartError(UART_HandleTypeDef *huart)
{
    if (huart != &huart2)
    {
        return;
    }

    g_vision_listening = 0U;
    if (g_vision_enabled == 0U)
    {
        return;
    }

    (void)HAL_UART_AbortReceive_IT(&huart2);
    Vision_StartReceive();
}

void Vision_SetEnabled(uint8_t enabled)
{
    g_vision_enabled = (enabled != 0U) ? 1U : 0U;

    if (g_vision_enabled == 0U)
    {
        g_vision_listening = 0U;
        g_vision_state = VISION_STATE_DISABLED;
        Vision_ResetLineBuffers();
        (void)HAL_UART_AbortReceive(&huart2);
        return;
    }

    g_vision_state = VISION_STATE_LISTENING;
    Vision_ResetLineBuffers();
    Vision_StartReceive();
}

uint8_t Vision_IsEnabled(void)
{
    return g_vision_enabled;
}

uint8_t Vision_IsOnline(void)
{
    if ((g_vision_enabled == 0U) || (g_vision_last_rx_ms == 0U))
    {
        return 0U;
    }

    return (uint8_t)(Vision_GetAgeMs() <= APP_VISION_ONLINE_TIMEOUT_MS);
}

uint8_t Vision_GetTarget(Vision_Target_t *target)
{
    if (target != (Vision_Target_t *)0)
    {
        *target = g_vision_target;
        target->age_ms = Vision_GetAgeMs();
        target->frames_ok = g_vision_frames_ok;
        target->frames_bad = g_vision_frames_bad;
        target->rx_total = g_vision_rx_total;
    }

    return (uint8_t)((Vision_IsOnline() != 0U) && (g_vision_target.valid != 0U));
}

Vision_State_t Vision_GetState(void)
{
    if (g_vision_enabled == 0U)
    {
        return VISION_STATE_DISABLED;
    }

    if ((g_vision_last_rx_ms != 0U) &&
        ((uint32_t)(HAL_GetTick() - g_vision_last_rx_ms) > APP_VISION_ONLINE_TIMEOUT_MS))
    {
        return VISION_STATE_TIMEOUT;
    }

    if (g_vision_line_pending != 0U)
    {
        return VISION_STATE_LINE_PENDING;
    }

    return g_vision_state;
}

const char *Vision_GetStateName(void)
{
    switch (Vision_GetState())
    {
    case VISION_STATE_DISABLED:
        return "DISABLED";
    case VISION_STATE_LISTENING:
        return "LISTENING";
    case VISION_STATE_LINE_PENDING:
        return "LINE_PENDING";
    case VISION_STATE_TARGET_VALID:
        return "TARGET_VALID";
    case VISION_STATE_NO_TARGET:
        return "NO_TARGET";
    case VISION_STATE_TIMEOUT:
        return "TIMEOUT";
    default:
        return "UNKNOWN";
    }
}

void Vision_FormatStatus(char *buffer, size_t buffer_size, const char *prefix)
{
    const char *line = Vision_GetStatusLine();

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
                   "%s VISION state=%s en=%u online=%u age=%lu rx=%lu ok=%lu bad=%lu seq=%u valid=%u x=%d y=%d dx=%d dy=%d area=%u line=%s",
                   prefix,
                   Vision_GetStateName(),
                   (unsigned int)g_vision_enabled,
                   (unsigned int)Vision_IsOnline(),
                   (unsigned long)Vision_GetAgeMs(),
                   (unsigned long)g_vision_rx_total,
                   (unsigned long)g_vision_frames_ok,
                   (unsigned long)g_vision_frames_bad,
                   (unsigned int)g_vision_target.seq,
                   (unsigned int)g_vision_target.valid,
                   (int)g_vision_target.x,
                   (int)g_vision_target.y,
                   (int)g_vision_target.dx,
                   (int)g_vision_target.dy,
                   (unsigned int)g_vision_target.area,
                   line);
}

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

static void Vision_StartReceive(void)
{
    if (g_vision_enabled == 0U)
    {
        return;
    }

    if (HAL_UART_Receive_IT(&huart2, &g_vision_rx_byte, 1U) == HAL_OK)
    {
        g_vision_listening = 1U;
    }
}

static void Vision_FinalizeLine(void)
{
    if (g_vision_build_overflow != 0U)
    {
        g_vision_build_overflow = 0U;
        g_vision_build_len = 0U;
        g_vision_build_buf[0] = '\0';
        g_vision_frames_bad++;
        return;
    }

    if (g_vision_build_len == 0U)
    {
        return;
    }

    (void)memcpy(g_vision_pending_line, g_vision_build_buf, g_vision_build_len + 1U);
    (void)memcpy(g_vision_last_line, g_vision_build_buf, g_vision_build_len + 1U);
    g_vision_line_pending = 1U;
    g_vision_build_len = 0U;
    g_vision_build_buf[0] = '\0';
}

static void Vision_ResetLineBuffers(void)
{
    g_vision_build_len = 0U;
    g_vision_build_overflow = 0U;
    g_vision_line_pending = 0U;
    g_vision_build_buf[0] = '\0';
    g_vision_pending_line[0] = '\0';
    g_vision_last_line[0] = '\0';
}

static void Vision_ProcessPendingLine(void)
{
    char line[APP_VISION_LINE_BUF_SIZE];

    if (g_vision_line_pending == 0U)
    {
        return;
    }

    __disable_irq();
    (void)memcpy(line, g_vision_pending_line, sizeof(line));
    g_vision_line_pending = 0U;
    __enable_irq();

    g_vision_state = VISION_STATE_LINE_PENDING;
    if (Vision_ParseCaptureAck(line) != 0U)
    {
        return;
    }

    if (Vision_ParseFrame(line) != 0U)
    {
        g_vision_frames_ok++;
        g_vision_target.frames_ok = g_vision_frames_ok;
        g_vision_target.frames_bad = g_vision_frames_bad;
        g_vision_target.rx_total = g_vision_rx_total;
        g_vision_target.age_ms = Vision_GetAgeMs();
        g_vision_state = (g_vision_target.valid != 0U) ? VISION_STATE_TARGET_VALID : VISION_STATE_NO_TARGET;
    }
    else
    {
        g_vision_frames_bad++;
        if (g_vision_frames_ok == 0U)
        {
            g_vision_state = VISION_STATE_LISTENING;
        }
    }
}

static uint8_t Vision_ParseFrame(char *line)
{
    char *payload_end;
    long fields[APP_VISION_FIELD_COUNT];

    if (line == (char *)0)
    {
        return 0U;
    }

    if ((line[0] != '$') || (line[1] != 'V') || (line[2] != ','))
    {
        return 0U;
    }

    if (Vision_ChecksumIsValid(line, &payload_end) == 0U)
    {
        return 0U;
    }

    *payload_end = '\0';
    if (Vision_ParseFields(&line[3], fields) == 0U)
    {
        return 0U;
    }

    if ((Vision_ValueInRange(fields[0], 0L, 65535L) == 0U) ||
        (Vision_ValueInRange(fields[1], 0L, 1L) == 0U) ||
        (Vision_ValueInRange(fields[2], -32768L, 32767L) == 0U) ||
        (Vision_ValueInRange(fields[3], -32768L, 32767L) == 0U) ||
        (Vision_ValueInRange(fields[4], -32768L, 32767L) == 0U) ||
        (Vision_ValueInRange(fields[5], -32768L, 32767L) == 0U) ||
        (Vision_ValueInRange(fields[6], 0L, 65535L) == 0U))
    {
        return 0U;
    }

    g_vision_target.seq = (uint16_t)fields[0];
    g_vision_target.valid = (uint8_t)fields[1];
    g_vision_target.x = (int16_t)fields[2];
    g_vision_target.y = (int16_t)fields[3];
    g_vision_target.dx = (int16_t)fields[4];
    g_vision_target.dy = (int16_t)fields[5];
    g_vision_target.area = (uint16_t)fields[6];
    return 1U;
}

static uint8_t Vision_ParseCaptureAck(char *line)
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

static uint8_t Vision_ChecksumIsValid(const char *line, char **payload_end)
{
    const char *cursor;
    const char *star;
    uint8_t checksum = 0U;
    int8_t high;
    int8_t low;
    uint8_t expected;

    if ((line == (const char *)0) || (payload_end == (char **)0))
    {
        return 0U;
    }

    star = strchr(line, '*');
    if ((star == (const char *)0) || (star[1] == '\0') || (star[2] == '\0') || (star[3] != '\0'))
    {
        return 0U;
    }

    for (cursor = line + 1; cursor < star; cursor++)
    {
        checksum = (uint8_t)(checksum ^ (uint8_t)(*cursor));
    }

    high = Vision_HexNibble(star[1]);
    low = Vision_HexNibble(star[2]);
    if ((high < 0) || (low < 0))
    {
        return 0U;
    }

    expected = (uint8_t)(((uint8_t)high << 4) | (uint8_t)low);
    if (checksum != expected)
    {
        return 0U;
    }

    *payload_end = (char *)star;
    return 1U;
}

static int8_t Vision_HexNibble(char ch)
{
    if ((ch >= '0') && (ch <= '9'))
    {
        return (int8_t)(ch - '0');
    }

    if ((ch >= 'A') && (ch <= 'F'))
    {
        return (int8_t)(ch - 'A' + 10);
    }

    if ((ch >= 'a') && (ch <= 'f'))
    {
        return (int8_t)(ch - 'a' + 10);
    }

    return -1;
}

static uint8_t Vision_ParseFields(char *payload, long fields[APP_VISION_FIELD_COUNT])
{
    char *cursor;
    char *end;
    uint32_t i;

    if ((payload == (char *)0) || (fields == (long *)0))
    {
        return 0U;
    }

    cursor = payload;
    for (i = 0U; i < APP_VISION_FIELD_COUNT; i++)
    {
        fields[i] = strtol(cursor, &end, 10);
        if (end == cursor)
        {
            return 0U;
        }

        if (i < (APP_VISION_FIELD_COUNT - 1U))
        {
            if (*end != ',')
            {
                return 0U;
            }
            cursor = end + 1;
        }
        else if (*end != '\0')
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t Vision_ValueInRange(long value, long min_value, long max_value)
{
    return (uint8_t)((value >= min_value) && (value <= max_value));
}

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

static uint32_t Vision_GetAgeMs(void)
{
    if (g_vision_last_rx_ms == 0U)
    {
        return 0U;
    }

    return (uint32_t)(HAL_GetTick() - g_vision_last_rx_ms);
}

static const char *Vision_GetStatusLine(void)
{
    if (g_vision_last_line[0] == '\0')
    {
        return "-";
    }

    return g_vision_last_line;
}
