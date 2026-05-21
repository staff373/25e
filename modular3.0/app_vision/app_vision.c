#include "app_vision.h"

#include "usart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define APP_VISION_ONLINE_TIMEOUT_MS (1000U)
#define APP_VISION_LINE_BUF_SIZE     (128U)
#define APP_VISION_FIELD_COUNT       (7U)

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

static void Vision_StartReceive(void);
static void Vision_FinalizeLine(void);
static void Vision_ResetLineBuffers(void);
static void Vision_ProcessPendingLine(void);
static uint8_t Vision_ParseFrame(char *line);
static uint8_t Vision_ChecksumIsValid(const char *line, char **payload_end);
static int8_t Vision_HexNibble(char ch);
static uint8_t Vision_ParseFields(char *payload, long fields[APP_VISION_FIELD_COUNT]);
static uint8_t Vision_ValueInRange(long value, long min_value, long max_value);
static uint32_t Vision_GetAgeMs(void);
static const char *Vision_GetStatusLine(void);

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
