#include "app_vision.h"

#include "usart.h"

#include <stdio.h>
#include <string.h>

#define APP_VISION_ONLINE_TIMEOUT_MS (1000U)
#define APP_VISION_LINE_BUF_SIZE     (64U)

static uint8_t g_vision_enabled = 1U;
static uint8_t g_vision_listening = 0U;
static uint8_t g_vision_rx_byte = 0U;
static uint32_t g_vision_rx_total = 0U;
static uint32_t g_vision_last_rx_ms = 0U;
static char g_vision_build_buf[APP_VISION_LINE_BUF_SIZE];
static uint16_t g_vision_build_len = 0U;
static char g_vision_last_line[APP_VISION_LINE_BUF_SIZE];

static void Vision_StartReceive(void);
static void Vision_FinalizeLine(void);

void Vision_Init(void)
{
    g_vision_enabled = 1U;
    g_vision_listening = 0U;
    g_vision_rx_byte = 0U;
    g_vision_rx_total = 0U;
    g_vision_last_rx_ms = 0U;
    g_vision_build_len = 0U;
    g_vision_build_buf[0] = '\0';
    g_vision_last_line[0] = '\0';
    Vision_StartReceive();
}

void Vision_Poll(void)
{
    if ((g_vision_enabled != 0U) && (g_vision_listening == 0U))
    {
        Vision_StartReceive();
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
        if (g_vision_build_len < (APP_VISION_LINE_BUF_SIZE - 1U))
        {
            g_vision_build_buf[g_vision_build_len++] = (char)byte;
            g_vision_build_buf[g_vision_build_len] = '\0';
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
        (void)HAL_UART_AbortReceive(&huart2);
        return;
    }

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

    return (uint8_t)((uint32_t)(HAL_GetTick() - g_vision_last_rx_ms) <= APP_VISION_ONLINE_TIMEOUT_MS);
}

void Vision_FormatStatus(char *buffer, size_t buffer_size, const char *prefix)
{
    const char *line = g_vision_last_line;
    uint32_t age_ms = 0U;

    if ((buffer == (char *)0) || (buffer_size == 0U))
    {
        return;
    }

    if (prefix == (const char *)0)
    {
        prefix = "OK";
    }

    if (g_vision_last_rx_ms != 0U)
    {
        age_ms = (uint32_t)(HAL_GetTick() - g_vision_last_rx_ms);
    }

    if (line[0] == '\0')
    {
        line = "-";
    }

    (void)snprintf(buffer,
                   buffer_size,
                   "%s VISION en=%u online=%u age=%lu rx=%lu line=%s",
                   prefix,
                   (unsigned int)g_vision_enabled,
                   (unsigned int)Vision_IsOnline(),
                   (unsigned long)age_ms,
                   (unsigned long)g_vision_rx_total,
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
    if (g_vision_build_len == 0U)
    {
        return;
    }

    (void)memcpy(g_vision_last_line, g_vision_build_buf, g_vision_build_len + 1U);
    g_vision_build_len = 0U;
    g_vision_build_buf[0] = '\0';
}
