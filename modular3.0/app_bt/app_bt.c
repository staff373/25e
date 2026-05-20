#include "app_bt.h"

#include "app_imu.h"
#include "app_motion.h"
#include "app_sensor.h"
#include "app_task.h"
#include "app_track.h"
#include "app_turn.h"
#include "app_vision.h"
#include "dma.h"
#include "usart.h"

#include <stdio.h>
#include <stdlib.h>
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
static void BT_ProcessLine(char *line);
static char *BT_Trim(char *text);
static uint8_t BT_IsSpace(char ch);
static uint8_t BT_ParseFourFloats(const char *text, float *a, float *b, float *c, float *d);
static uint8_t BT_ParseNameValue(char *text, char **name, float *value);
static void BT_WriteText(const char *text);
static void BT_WriteLine(const char *text);
static void BT_SendStatus(void);
static void BT_SendParams(void);
static void BT_SendSetResult(const char *name, float value);
static uint8_t BT_SetWhitelistedParam(const char *name, float value);
static uint8_t BT_ParseTaskQuestion(const char *text, uint8_t *question_id);

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

    while (BT_DequeueLine(line, (uint16_t)sizeof(line)) != 0U)
    {
        BT_ProcessLine(line);
    }
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

static void BT_ProcessLine(char *line)
{
    char response[256];
    char *cmd;
    char *param_name;
    float value;
    float lf;
    float rf;
    float lb;
    float rb;

    cmd = BT_Trim(line);
    if ((cmd == (char *)0) || (*cmd == '\0'))
    {
        return;
    }

    if (strcmp(cmd, "PING") == 0)
    {
        BT_WriteLine("OK PONG");
        return;
    }

    if (strcmp(cmd, "STOP") == 0)
    {
        Task_Stop();
        BT_WriteLine("OK STOP");
        return;
    }

    if (strcmp(cmd, "STATUS") == 0)
    {
        BT_SendStatus();
        return;
    }

    if (strcmp(cmd, "SENSOR?") == 0)
    {
        AppSensor_FormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "IMU?") == 0)
    {
        Imu_FormatStatus(response, sizeof(response), (Imu_GetInitStatus() == 0) ? "OK" : "ERR");
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "VISION?") == 0)
    {
        Vision_FormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "VISION ON") == 0)
    {
        Vision_SetEnabled(1U);
        Vision_FormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "VISION OFF") == 0)
    {
        Vision_SetEnabled(0U);
        Vision_FormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strncmp(cmd, "MOTOR", 5U) == 0)
    {
        if ((cmd[5] == '\0') || (BT_IsSpace(cmd[5]) == 0U) ||
            (BT_ParseFourFloats(&cmd[5], &lf, &rf, &lb, &rb) == 0U))
        {
            BT_WriteLine("ERR MOTOR");
            return;
        }

        Task_Stop();
        Motion_SetDuty4(lf, rf, lb, rb);
        (void)snprintf(response,
                       sizeof(response),
                       "OK MOTOR lf=%.1f rf=%.1f lb=%.1f rb=%.1f",
                       lf,
                       rf,
                       lb,
                       rb);
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "TURN L") == 0)
    {
        Task_Stop();
        if (Turn_Start(TURN_DIR_LEFT) != 0U)
        {
            BT_WriteLine("OK TURN L");
        }
        else
        {
            BT_WriteLine("ERR TURN");
        }
        return;
    }

    if (strcmp(cmd, "TURN R") == 0)
    {
        Task_Stop();
        if (Turn_Start(TURN_DIR_RIGHT) != 0U)
        {
            BT_WriteLine("OK TURN R");
        }
        else
        {
            BT_WriteLine("ERR TURN");
        }
        return;
    }

    if (strcmp(cmd, "TRACK START") == 0)
    {
        if ((Task_Select(TASK_MODE_Q1_TRACK) != 0U) && (Task_Start() != 0U))
        {
            BT_WriteLine("OK TRACK START");
        }
        else
        {
            BT_WriteLine("ERR TRACK");
        }
        return;
    }

    if (strcmp(cmd, "TRACK STOP") == 0)
    {
        Task_Stop();
        BT_WriteLine("OK TRACK STOP");
        return;
    }

    if (strcmp(cmd, "TASK?") == 0)
    {
        Task_FormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "TASK START") == 0)
    {
        if (Task_Start() != 0U)
        {
            BT_WriteLine("OK TASK START");
        }
        else
        {
            BT_WriteLine("ERR TASK");
        }
        return;
    }

    if (strcmp(cmd, "TASK STOP") == 0)
    {
        Task_Stop();
        BT_WriteLine("OK TASK STOP");
        return;
    }

    if (strcmp(cmd, "TASK RESET") == 0)
    {
        Task_Reset();
        BT_WriteLine("OK TASK RESET");
        return;
    }

    if ((strncmp(cmd, "TASK", 4U) == 0) && (BT_IsSpace(cmd[4]) != 0U))
    {
        uint8_t question_id;

        if ((BT_ParseTaskQuestion(&cmd[4], &question_id) != 0U) &&
            (Task_SelectQuestion(question_id) != 0U))
        {
            (void)snprintf(response, sizeof(response), "OK TASK %u", (unsigned int)question_id);
            BT_WriteLine(response);
        }
        else
        {
            BT_WriteLine("ERR TASK");
        }
        return;
    }

    if (strcmp(cmd, "GET") == 0)
    {
        BT_SendParams();
        return;
    }

    if ((strncmp(cmd, "SET", 3U) == 0) && (BT_IsSpace(cmd[3]) != 0U))
    {
        if (BT_ParseNameValue(&cmd[3], &param_name, &value) == 0U)
        {
            BT_WriteLine("ERR SET");
            return;
        }

        if (BT_SetWhitelistedParam(param_name, value) == 0U)
        {
            BT_WriteLine("ERR SET");
            return;
        }

        BT_SendSetResult(param_name, value);
        return;
    }

    BT_WriteLine("ERR CMD");
}

static char *BT_Trim(char *text)
{
    char *start;
    char *end;

    if (text == (char *)0)
    {
        return (char *)0;
    }

    start = text;
    while (BT_IsSpace(*start) != 0U)
    {
        start++;
    }

    end = start + strlen(start);
    while ((end > start) && (BT_IsSpace(*(end - 1)) != 0U))
    {
        end--;
    }
    *end = '\0';
    return start;
}

static uint8_t BT_IsSpace(char ch)
{
    return (uint8_t)((ch == ' ') || (ch == '\t') || (ch == '\r') || (ch == '\n'));
}

static uint8_t BT_ParseFourFloats(const char *text, float *a, float *b, float *c, float *d)
{
    char *end;
    float values[4];
    const char *cursor;
    uint32_t i;

    if ((text == (const char *)0) || (a == (float *)0) || (b == (float *)0) ||
        (c == (float *)0) || (d == (float *)0))
    {
        return 0U;
    }

    cursor = text;
    for (i = 0U; i < 4U; i++)
    {
        while (BT_IsSpace(*cursor) != 0U)
        {
            cursor++;
        }

        if (*cursor == '\0')
        {
            return 0U;
        }

        values[i] = strtof(cursor, &end);
        if ((end == cursor) || (values[i] != values[i]))
        {
            return 0U;
        }

        cursor = end;
    }

    while (BT_IsSpace(*cursor) != 0U)
    {
        cursor++;
    }

    if (*cursor != '\0')
    {
        return 0U;
    }

    *a = values[0];
    *b = values[1];
    *c = values[2];
    *d = values[3];
    return 1U;
}

static uint8_t BT_ParseNameValue(char *text, char **name, float *value)
{
    char *cursor;
    char *end;

    if ((text == (char *)0) || (name == (char **)0) || (value == (float *)0))
    {
        return 0U;
    }

    cursor = BT_Trim(text);
    if ((cursor == (char *)0) || (*cursor == '\0'))
    {
        return 0U;
    }

    *name = cursor;
    while ((*cursor != '\0') && (BT_IsSpace(*cursor) == 0U))
    {
        cursor++;
    }

    if (*cursor == '\0')
    {
        return 0U;
    }

    *cursor = '\0';
    cursor++;
    while (BT_IsSpace(*cursor) != 0U)
    {
        cursor++;
    }

    if (*cursor == '\0')
    {
        return 0U;
    }

    *value = strtof(cursor, &end);
    if ((end == cursor) || (*value != *value))
    {
        return 0U;
    }

    while (BT_IsSpace(*end) != 0U)
    {
        end++;
    }

    return (uint8_t)(*end == '\0');
}

static uint8_t BT_ParseTaskQuestion(const char *text, uint8_t *question_id)
{
    char *end;
    unsigned long value;
    const char *cursor;

    if ((text == (const char *)0) || (question_id == (uint8_t *)0))
    {
        return 0U;
    }

    cursor = text;
    while (BT_IsSpace(*cursor) != 0U)
    {
        cursor++;
    }

    if (*cursor == '\0')
    {
        return 0U;
    }

    value = strtoul(cursor, &end, 10);
    if ((end == cursor) || (value > 255UL))
    {
        return 0U;
    }

    while (BT_IsSpace(*end) != 0U)
    {
        end++;
    }

    if (*end != '\0')
    {
        return 0U;
    }

    *question_id = (uint8_t)value;
    return 1U;
}

static void BT_WriteText(const char *text)
{
    if (text == (const char *)0)
    {
        return;
    }

    (void)HAL_UART_Transmit(&huart5, (uint8_t *)text, (uint16_t)strlen(text), APP_BT_TX_TIMEOUT_MS);
}

static void BT_WriteLine(const char *text)
{
    BT_WriteText(text);
    BT_WriteText("\r\n");
}

static void BT_SendStatus(void)
{
    char response[256];

    (void)snprintf(response,
                   sizeof(response),
                   "OK STATUS task=%s mode=%s task_stop=%s track=%s laps=%u/%u corners=%u elapsed=%lu track_stop=%s imu=%u yaw=%.1f vision=%u sensor=0x%02X turn=%u",
                   Task_GetStateName(),
                   Task_GetModeName(),
                   Task_GetStopReasonName(),
                   Track_GetStateName(),
                   (unsigned int)Track_GetLapsDone(),
                   (unsigned int)Track_GetTargetLaps(),
                   (unsigned int)Track_GetCornerCount(),
                   (unsigned long)Task_GetElapsedMs(),
                   Track_GetStopReasonName(),
                   (unsigned int)Imu_IsOnline(),
                   Imu_GetYawDeg(),
                   (unsigned int)Vision_IsEnabled(),
                   (unsigned int)AppSensor_GetRawState(),
                   (unsigned int)Turn_IsActive());
    BT_WriteLine(response);
}

static void BT_SendParams(void)
{
    char response[256];
    float base = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float turn_out = 0.0f;
    float turn_in = 0.0f;
    float turn_angle = 0.0f;
    float corner_ms = 0.0f;
    float recover_ms = 0.0f;
    float max_turn_ms = 0.0f;
    float laps = 0.0f;

    (void)Track_GetParam("BASE", &base);
    (void)Track_GetParam("KP", &kp);
    (void)Track_GetParam("KD", &kd);
    (void)Turn_GetParam("TURN_OUT", &turn_out);
    (void)Turn_GetParam("TURN_IN", &turn_in);
    (void)Turn_GetParam("TURN_ANGLE", &turn_angle);
    (void)Track_GetParam("CORNER_MS", &corner_ms);
    (void)Track_GetParam("RECOVER_MS", &recover_ms);
    (void)Turn_GetParam("MAX_TURN_MS", &max_turn_ms);
    (void)Track_GetParam("LAPS", &laps);

    (void)snprintf(response,
                   sizeof(response),
                   "OK BASE=%.1f KP=%.1f KD=%.1f TURN_OUT=%.1f TURN_IN=%.1f TURN_ANGLE=%.1f CORNER_MS=%.0f RECOVER_MS=%.0f MAX_TURN_MS=%.0f LAPS=%.0f",
                   base,
                   kp,
                   kd,
                   turn_out,
                   turn_in,
                   turn_angle,
                   corner_ms,
                   recover_ms,
                   max_turn_ms,
                   laps);
    BT_WriteLine(response);
}

static void BT_SendSetResult(const char *name, float value)
{
    char response[80];
    float actual_value = value;

    if ((Track_GetParam(name, &actual_value) == 0U) &&
        (Turn_GetParam(name, &actual_value) == 0U))
    {
        actual_value = value;
    }

    (void)snprintf(response, sizeof(response), "OK %s=%.3f", name, actual_value);
    BT_WriteLine(response);
}

static uint8_t BT_SetWhitelistedParam(const char *name, float value)
{
    if (Track_SetParam(name, value) != 0U)
    {
        return 1U;
    }

    if (Turn_SetParam(name, value) != 0U)
    {
        return 1U;
    }

    return 0U;
}
