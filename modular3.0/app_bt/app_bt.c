#include "app_bt.h"

#include "app_bt_internal.h"
#include "app_imu.h"
#include "app_vision.h"
#include "dma.h"
#include "usart.h"

#include <string.h>

#define APP_BT_RX_DMA_BUF_SIZE (128U)
#define APP_BT_LINE_BUF_SIZE   (96U)
#define APP_BT_QUEUE_DEPTH     (4U)
#define APP_BT_TX_TIMEOUT_MS   (20U)

static uint8_t g_bt_dma_rx_buf[APP_BT_RX_DMA_BUF_SIZE];
static char g_bt_line_build[APP_BT_LINE_BUF_SIZE];
static uint16_t g_bt_line_build_len = 0U;
static char g_bt_line_queue[APP_BT_QUEUE_DEPTH][APP_BT_LINE_BUF_SIZE];
static uint8_t g_bt_queue_head = 0U;
static uint8_t g_bt_queue_tail = 0U;
static uint8_t g_bt_rx_armed = 0U;
static uint8_t g_bt_rx_pending = 0U;

static void BT_StartRx(void);
static void BT_RestartRx(void);
static void BT_FeedBytes(const uint8_t *data, uint16_t size);
static void BT_QueueLine(void);
static uint8_t BT_DequeueLine(char *out_line, uint16_t out_size);

void BT_Init(void)
{
    g_bt_line_build_len = 0U;
    g_bt_line_build[0] = '\0';
    g_bt_queue_head = 0U;
    g_bt_queue_tail = 0U;
    g_bt_rx_armed = 0U;
    g_bt_rx_pending = 0U;
    BT_StartRx();
}

void BT_Poll(void)
{
    char line[APP_BT_LINE_BUF_SIZE];

    if (g_bt_rx_armed == 0U)
    {
        BT_StartRx();
    }

    BT_SendCaptureNotices();

    while (BT_DequeueLine(line, (uint16_t)sizeof(line)) != 0U)
    {
        BT_ProcessLine(line);
    }

    BT_SendCaptureNotices();
    BT_SendVisionStream();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != &huart5)
    {
        return;
    }

    g_bt_rx_armed = 0U;
    BT_FeedBytes(g_bt_dma_rx_buf, Size);
    if (g_bt_rx_pending != 0U)
    {
        BT_QueueLine();
        g_bt_rx_pending = 0U;
    }
    BT_StartRx();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    Imu_HandleRxCplt(huart);
    Vision_HandleRxCplt(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart5)
    {
        BT_RestartRx();
        return;
    }

    Imu_HandleUartError(huart);
    Vision_HandleUartError(huart);
}

void BT_WriteText(const char *text)
{
    if (text == (const char *)0)
    {
        return;
    }

    (void)HAL_UART_Transmit(&huart5, (uint8_t *)text, (uint16_t)strlen(text), APP_BT_TX_TIMEOUT_MS);
}

void BT_WriteLine(const char *text)
{
    BT_WriteText(text);
    BT_WriteText("\r\n");
}

static void BT_StartRx(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart5, g_bt_dma_rx_buf, APP_BT_RX_DMA_BUF_SIZE) == HAL_OK)
    {
        g_bt_rx_armed = 1U;
        if (huart5.hdmarx != (DMA_HandleTypeDef *)0)
        {
            __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
        }
    }
    else
    {
        g_bt_rx_armed = 0U;
    }
}

static void BT_RestartRx(void)
{
    g_bt_rx_armed = 0U;
    (void)HAL_UART_DMAStop(&huart5);
    BT_StartRx();
}

static void BT_FeedBytes(const uint8_t *data, uint16_t size)
{
    uint16_t i;
    uint8_t byte;

    if (data == (const uint8_t *)0)
    {
        return;
    }

    for (i = 0U; i < size; i++)
    {
        byte = data[i];
        if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n'))
        {
            BT_QueueLine();
            g_bt_rx_pending = 0U;
            continue;
        }

        if ((byte < 32U) || (byte > 126U))
        {
            continue;
        }

        if (g_bt_line_build_len < (APP_BT_LINE_BUF_SIZE - 1U))
        {
            g_bt_line_build[g_bt_line_build_len++] = (char)byte;
            g_bt_line_build[g_bt_line_build_len] = '\0';
            g_bt_rx_pending = 1U;
        }
    }
}

static void BT_QueueLine(void)
{
    uint8_t next_tail;

    if (g_bt_line_build_len == 0U)
    {
        return;
    }

    next_tail = (uint8_t)((g_bt_queue_tail + 1U) % APP_BT_QUEUE_DEPTH);
    if (next_tail == g_bt_queue_head)
    {
        g_bt_line_build_len = 0U;
        g_bt_line_build[0] = '\0';
        return;
    }

    (void)memcpy(g_bt_line_queue[g_bt_queue_tail], g_bt_line_build, g_bt_line_build_len + 1U);
    g_bt_queue_tail = next_tail;
    g_bt_line_build_len = 0U;
    g_bt_line_build[0] = '\0';
    g_bt_rx_pending = 0U;
}

static uint8_t BT_DequeueLine(char *out_line, uint16_t out_size)
{
    size_t length;

    if ((out_line == (char *)0) || (out_size == 0U))
    {
        return 0U;
    }

    if (g_bt_queue_head == g_bt_queue_tail)
    {
        return 0U;
    }

    length = strlen(g_bt_line_queue[g_bt_queue_head]);
    if (length >= out_size)
    {
        length = (size_t)out_size - 1U;
    }

    (void)memcpy(out_line, g_bt_line_queue[g_bt_queue_head], length);
    out_line[length] = '\0';
    g_bt_queue_head = (uint8_t)((g_bt_queue_head + 1U) % APP_BT_QUEUE_DEPTH);
    return 1U;
}
