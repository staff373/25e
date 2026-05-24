#include "app_bt_internal.h"
#include "app_aim.h"
#include "app_gimbal.h"
#include "app_imu.h"
#include "app_motion.h"
#include "app_sensor.h"
#include "app_task.h"
#include "app_track.h"
#include "app_turn.h"
#include "app_vision.h"
#include "app_vision_tools.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char *BT_Trim(char *text);
static uint8_t BT_IsSpace(char ch);
static uint8_t BT_ParseFourFloats(const char *text, float *a, float *b, float *c, float *d);
static uint8_t BT_ParseEnableValue(const char *text, uint8_t *enabled);
static uint8_t BT_ParseLongAndFloat(const char *text, int32_t *steps, float *speed);
static uint8_t BT_ParseTwoLongsAndFloat(const char *text, int32_t *x_steps, int32_t *y_steps, float *speed);
static uint8_t BT_ParseNameValue(char *text, char **name, float *value);
static uint8_t BT_ParseUint32(const char *text, uint32_t *value);
static void BT_SendStatus(void);
static void BT_SendParams(void);
static void BT_SendSetResult(const char *name, float value);
static uint8_t BT_SetWhitelistedParam(const char *name, float value);
static uint8_t BT_ParseTaskQuestion(const char *text, uint8_t *question_id);

void BT_ProcessLine(char *line)
{
    char response[768];
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
        Aim_Stop();
        Gimbal_Stop();
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

    if (strcmp(cmd, "TRACK?") == 0)
    {
        Track_FormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "TURN?") == 0)
    {
        Turn_FormatStatus(response, sizeof(response), "OK");
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

    if (strcmp(cmd, "VISION STREAM ON") == 0)
    {
        Vision_StreamSetEnabled(1U);
        Vision_StreamFormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "VISION STREAM OFF") == 0)
    {
        Vision_StreamSetEnabled(0U);
        Vision_StreamFormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "VISION STREAM?") == 0)
    {
        Vision_StreamFormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "CAP") == 0)
    {
        uint32_t request_id;

        if (Vision_CaptureRequest(&request_id) != 0U)
        {
            (void)snprintf(response,
                           sizeof(response),
                           "OK CAP SENT id=%lu",
                           (unsigned long)request_id);
            BT_WriteLine(response);
        }
        else
        {
            Vision_CaptureFormatStatus(response, sizeof(response), "ERR");
            BT_WriteLine(response);
        }
        return;
    }

    if (strcmp(cmd, "CAP?") == 0)
    {
        Vision_CaptureFormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "CAP AUTO OFF") == 0)
    {
        Vision_CaptureAutoStop();
        Vision_CaptureFormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strncmp(cmd, "CAP AUTO", 8U) == 0)
    {
        uint32_t interval_ms;

        if ((cmd[8] == '\0') || (BT_IsSpace(cmd[8]) == 0U) ||
            (BT_ParseUint32(&cmd[8], &interval_ms) == 0U) ||
            (interval_ms < 50U))
        {
            BT_WriteLine("ERR CAP AUTO");
            return;
        }

        if (Vision_CaptureAutoStart(interval_ms) != 0U)
        {
            (void)snprintf(response,
                           sizeof(response),
                           "OK CAP AUTO interval=%lu",
                           (unsigned long)interval_ms);
            BT_WriteLine(response);
        }
        else
        {
            Vision_CaptureFormatStatus(response, sizeof(response), "ERR");
            BT_WriteLine(response);
        }
        return;
    }

    if (strcmp(cmd, "GIMBAL?") == 0)
    {
        Gimbal_FormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "GIMBAL ZERO") == 0)
    {
        Gimbal_Zero();
        Gimbal_FormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strncmp(cmd, "GIMBAL EN", 9U) == 0)
    {
        uint8_t enabled;

        if ((cmd[9] == '\0') || (BT_IsSpace(cmd[9]) == 0U) ||
            (BT_ParseEnableValue(&cmd[9], &enabled) == 0U))
        {
            BT_WriteLine("ERR GIMBAL EN");
            return;
        }

        Gimbal_SetHoldEnabled(enabled);
        Gimbal_FormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "GIMBAL STOP") == 0)
    {
        Gimbal_Stop();
        Gimbal_FormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "GIMBAL ESTOP") == 0)
    {
        Aim_Stop();
        Gimbal_EStop();
        Gimbal_FormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strncmp(cmd, "GIMBAL MOVE XY", 14U) == 0)
    {
        int32_t x_steps;
        int32_t y_steps;
        float speed;

        if ((cmd[14] == '\0') || (BT_IsSpace(cmd[14]) == 0U) ||
            (BT_ParseTwoLongsAndFloat(&cmd[14], &x_steps, &y_steps, &speed) == 0U))
        {
            BT_WriteLine("ERR GIMBAL MOVE");
            return;
        }

        if (Gimbal_MoveRelativeSteps(x_steps, y_steps, speed) == 0U)
        {
            Gimbal_FormatStatus(response, sizeof(response), "ERR");
            BT_WriteLine(response);
            return;
        }

        Gimbal_FormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strncmp(cmd, "GIMBAL MOVE X", 13U) == 0)
    {
        int32_t steps;
        float speed;

        if ((cmd[13] == '\0') || (BT_IsSpace(cmd[13]) == 0U) ||
            (BT_ParseLongAndFloat(&cmd[13], &steps, &speed) == 0U))
        {
            BT_WriteLine("ERR GIMBAL MOVE");
            return;
        }

        if (Gimbal_MoveRelativeSteps(steps, 0, speed) == 0U)
        {
            Gimbal_FormatStatus(response, sizeof(response), "ERR");
            BT_WriteLine(response);
            return;
        }

        Gimbal_FormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strncmp(cmd, "GIMBAL MOVE Y", 13U) == 0)
    {
        int32_t steps;
        float speed;

        if ((cmd[13] == '\0') || (BT_IsSpace(cmd[13]) == 0U) ||
            (BT_ParseLongAndFloat(&cmd[13], &steps, &speed) == 0U))
        {
            BT_WriteLine("ERR GIMBAL MOVE");
            return;
        }

        if (Gimbal_MoveRelativeSteps(0, steps, speed) == 0U)
        {
            Gimbal_FormatStatus(response, sizeof(response), "ERR");
            BT_WriteLine(response);
            return;
        }

        Gimbal_FormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "GIMBAL CAL?") == 0)
    {
        float a;
        float b;
        float c;
        float d;

        Gimbal_GetCalibration(&a, &b, &c, &d);
        (void)snprintf(response,
                       sizeof(response),
                       "OK GIMBAL CAL valid=%u a=%.5f b=%.5f c=%.5f d=%.5f",
                       (unsigned int)Gimbal_IsCalibrated(),
                       a,
                       b,
                       c,
                       d);
        BT_WriteLine(response);
        return;
    }

    if (strncmp(cmd, "GIMBAL CAL SET", 14U) == 0)
    {
        float a;
        float b;
        float c;
        float d;

        if ((cmd[14] == '\0') || (BT_IsSpace(cmd[14]) == 0U) ||
            (BT_ParseFourFloats(&cmd[14], &a, &b, &c, &d) == 0U))
        {
            BT_WriteLine("ERR GIMBAL CAL");
            return;
        }

        Gimbal_SetCalibration(a, b, c, d);
        (void)snprintf(response,
                       sizeof(response),
                       "OK GIMBAL CAL a=%.5f b=%.5f c=%.5f d=%.5f",
                       a,
                       b,
                       c,
                       d);
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "AIM?") == 0)
    {
        Aim_FormatStatus(response, sizeof(response), "OK");
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "AIM ONCE") == 0)
    {
        if (Aim_StartOnce(0U) != 0U)
        {
            Aim_FormatStatus(response, sizeof(response), "OK");
        }
        else
        {
            Aim_FormatStatus(response, sizeof(response), "ERR");
        }
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "AIM TRACK") == 0)
    {
        if (Aim_StartTrack() != 0U)
        {
            Aim_FormatStatus(response, sizeof(response), "OK");
        }
        else
        {
            Aim_FormatStatus(response, sizeof(response), "ERR");
        }
        BT_WriteLine(response);
        return;
    }

    if (strcmp(cmd, "AIM STOP") == 0)
    {
        Aim_Stop();
        Aim_FormatStatus(response, sizeof(response), "OK");
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

    if ((strcmp(cmd, "TASK ADV1") == 0) || (strcmp(cmd, "TASK A1") == 0))
    {
        if (Task_Select(TASK_MODE_ADV1_AIM_TRACK) != 0U)
        {
            BT_WriteLine("OK TASK ADV1");
        }
        else
        {
            BT_WriteLine("ERR TASK");
        }
        return;
    }

    if ((strcmp(cmd, "TASK ADVTRACK") == 0) || (strcmp(cmd, "TASK AT") == 0))
    {
        if (Task_Select(TASK_MODE_ADV_TRACK_TEST) != 0U)
        {
            BT_WriteLine("OK TASK ADVTRACK");
        }
        else
        {
            BT_WriteLine("ERR TASK");
        }
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

static uint8_t BT_ParseEnableValue(const char *text, uint8_t *enabled)
{
    char *end;
    unsigned long value;
    const char *cursor;

    if ((text == (const char *)0) || (enabled == (uint8_t *)0))
    {
        return 0U;
    }

    cursor = text;
    while (BT_IsSpace(*cursor) != 0U)
    {
        cursor++;
    }

    value = strtoul(cursor, &end, 10);
    if ((end == cursor) || (value > 1UL))
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

    *enabled = (uint8_t)value;
    return 1U;
}

static uint8_t BT_ParseLongAndFloat(const char *text, int32_t *steps, float *speed)
{
    char *end;
    long step_value;
    float speed_value;
    const char *cursor;

    if ((text == (const char *)0) || (steps == (int32_t *)0) || (speed == (float *)0))
    {
        return 0U;
    }

    cursor = text;
    while (BT_IsSpace(*cursor) != 0U)
    {
        cursor++;
    }

    step_value = strtol(cursor, &end, 10);
    if (end == cursor)
    {
        return 0U;
    }
    cursor = end;

    while (BT_IsSpace(*cursor) != 0U)
    {
        cursor++;
    }

    speed_value = strtof(cursor, &end);
    if ((end == cursor) || (speed_value != speed_value))
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

    *steps = (int32_t)step_value;
    *speed = speed_value;
    return 1U;
}

static uint8_t BT_ParseTwoLongsAndFloat(const char *text, int32_t *x_steps, int32_t *y_steps, float *speed)
{
    char *end;
    long x_value;
    long y_value;
    float speed_value;
    const char *cursor;

    if ((text == (const char *)0) || (x_steps == (int32_t *)0) ||
        (y_steps == (int32_t *)0) || (speed == (float *)0))
    {
        return 0U;
    }

    cursor = text;
    while (BT_IsSpace(*cursor) != 0U)
    {
        cursor++;
    }

    x_value = strtol(cursor, &end, 10);
    if (end == cursor)
    {
        return 0U;
    }
    cursor = end;

    while (BT_IsSpace(*cursor) != 0U)
    {
        cursor++;
    }

    y_value = strtol(cursor, &end, 10);
    if (end == cursor)
    {
        return 0U;
    }
    cursor = end;

    while (BT_IsSpace(*cursor) != 0U)
    {
        cursor++;
    }

    speed_value = strtof(cursor, &end);
    if ((end == cursor) || (speed_value != speed_value))
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

    *x_steps = (int32_t)x_value;
    *y_steps = (int32_t)y_value;
    *speed = speed_value;
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

static uint8_t BT_ParseUint32(const char *text, uint32_t *value)
{
    char *end;
    unsigned long parsed;
    const char *cursor;

    if ((text == (const char *)0) || (value == (uint32_t *)0))
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

    parsed = strtoul(cursor, &end, 10);
    if (end == cursor)
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

    *value = (uint32_t)parsed;
    return 1U;
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

void BT_SendCaptureNotices(void)
{
    char notice[128];

    while (Vision_CaptureTakeNotice(notice, sizeof(notice)) != 0U)
    {
        BT_WriteLine(notice);
    }
}

void BT_SendVisionStream(void)
{
    char line[192];

    while (Vision_StreamTakeLine(line, sizeof(line)) != 0U)
    {
        BT_WriteLine(line);
    }
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
    char response[1280];
    float base = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float center_bias = 0.0f;
    float corner_advance_ms = 0.0f;
    float turn_out = 0.0f;
    float turn_in = 0.0f;
    float turn_angle = 0.0f;
    float turn_ramp = 0.0f;
    float turn_rate_scale = 0.0f;
    float turn_rate_kp = 0.0f;
    float turn_stop_rate = 0.0f;
    float turn_r0 = 0.0f;
    float turn_r15 = 0.0f;
    float turn_r30 = 0.0f;
    float turn_r45 = 0.0f;
    float turn_r60 = 0.0f;
    float turn_r75 = 0.0f;
    float turn_r90 = 0.0f;
    float recover_ms = 0.0f;
    float max_turn_ms = 0.0f;
    float laps = 0.0f;
    float left_trim = 0.0f;
    float right_trim = 0.0f;
    float aim_line_pred_x = 0.0f;
    float aim_line_pred_y = 0.0f;
    float aim_turn_prefeed_x = 0.0f;
    float aim_turn_prefeed_y = 0.0f;
    float aim_turn_ff_x_per_deg = 0.0f;
    float aim_turn_ff_y_per_deg = 0.0f;
    float aim_turn_ff_gyro_x = 0.0f;
    float aim_turn_ff_gyro_y = 0.0f;
    float aim_turn_ff_max_step = 0.0f;
    float aim_turn_ff_speed_sps = 0.0f;

    (void)Track_GetParam("BASE", &base);
    (void)Track_GetParam("KP", &kp);
    (void)Track_GetParam("KD", &kd);
    (void)Track_GetParam("CENTER_BIAS", &center_bias);
    (void)Track_GetParam("CORNER_ADVANCE_MS", &corner_advance_ms);
    (void)Turn_GetParam("TURN_OUT", &turn_out);
    (void)Turn_GetParam("TURN_IN", &turn_in);
    (void)Turn_GetParam("TURN_ANGLE", &turn_angle);
    (void)Turn_GetParam("TURN_RAMP", &turn_ramp);
    (void)Turn_GetParam("TURN_RATE_SCALE", &turn_rate_scale);
    (void)Turn_GetParam("TURN_RATE_KP", &turn_rate_kp);
    (void)Turn_GetParam("TURN_STOP_RATE", &turn_stop_rate);
    (void)Turn_GetParam("TURN_R0", &turn_r0);
    (void)Turn_GetParam("TURN_R15", &turn_r15);
    (void)Turn_GetParam("TURN_R30", &turn_r30);
    (void)Turn_GetParam("TURN_R45", &turn_r45);
    (void)Turn_GetParam("TURN_R60", &turn_r60);
    (void)Turn_GetParam("TURN_R75", &turn_r75);
    (void)Turn_GetParam("TURN_R90", &turn_r90);
    (void)Track_GetParam("RECOVER_MS", &recover_ms);
    (void)Turn_GetParam("MAX_TURN_MS", &max_turn_ms);
    (void)Track_GetParam("LAPS", &laps);
    (void)Track_GetParam("LEFT_TRIM", &left_trim);
    (void)Track_GetParam("RIGHT_TRIM", &right_trim);
    (void)Aim_GetParam("AIM_LINE_PRED_X", &aim_line_pred_x);
    (void)Aim_GetParam("AIM_LINE_PRED_Y", &aim_line_pred_y);
    (void)Aim_GetParam("AIM_TURN_PREFEED_X", &aim_turn_prefeed_x);
    (void)Aim_GetParam("AIM_TURN_PREFEED_Y", &aim_turn_prefeed_y);
    (void)Aim_GetParam("AIM_TURN_FF_X_PER_DEG", &aim_turn_ff_x_per_deg);
    (void)Aim_GetParam("AIM_TURN_FF_Y_PER_DEG", &aim_turn_ff_y_per_deg);
    (void)Aim_GetParam("AIM_TURN_FF_GYRO_X", &aim_turn_ff_gyro_x);
    (void)Aim_GetParam("AIM_TURN_FF_GYRO_Y", &aim_turn_ff_gyro_y);
    (void)Aim_GetParam("AIM_TURN_FF_MAX_STEP", &aim_turn_ff_max_step);
    (void)Aim_GetParam("AIM_TURN_FF_SPEED_SPS", &aim_turn_ff_speed_sps);

    (void)snprintf(response,
                   sizeof(response),
                   "OK BASE=%.1f KP=%.1f KD=%.1f CENTER_BIAS=%.2f CORNER_ADVANCE_MS=%.0f TURN_OUT=%.1f TURN_IN=%.1f TURN_ANGLE=%.1f TURN_RAMP=%.1f TURN_RATE_SCALE=%.2f TURN_RATE_KP=%.4f TURN_STOP_RATE=%.1f TURN_R0=%.0f TURN_R15=%.0f TURN_R30=%.0f TURN_R45=%.0f TURN_R60=%.0f TURN_R75=%.0f TURN_R90=%.0f RECOVER_MS=%.0f MAX_TURN_MS=%.0f LAPS=%.0f LEFT_TRIM=%.3f RIGHT_TRIM=%.3f AIM_LINE_PRED_X=%.2f AIM_LINE_PRED_Y=%.2f AIM_TURN_PREFEED_X=%.1f AIM_TURN_PREFEED_Y=%.1f AIM_TURN_FF_X_PER_DEG=%.3f AIM_TURN_FF_Y_PER_DEG=%.3f AIM_TURN_FF_GYRO_X=%.3f AIM_TURN_FF_GYRO_Y=%.3f AIM_TURN_FF_MAX_STEP=%.1f AIM_TURN_FF_SPEED_SPS=%.0f",
                   base,
                   kp,
                   kd,
                   center_bias,
                   corner_advance_ms,
                   turn_out,
                   turn_in,
                   turn_angle,
                   turn_ramp,
                   turn_rate_scale,
                   turn_rate_kp,
                   turn_stop_rate,
                   turn_r0,
                   turn_r15,
                   turn_r30,
                   turn_r45,
                   turn_r60,
                   turn_r75,
                   turn_r90,
                   recover_ms,
                   max_turn_ms,
                   laps,
                   left_trim,
                   right_trim,
                   aim_line_pred_x,
                   aim_line_pred_y,
                   aim_turn_prefeed_x,
                   aim_turn_prefeed_y,
                   aim_turn_ff_x_per_deg,
                   aim_turn_ff_y_per_deg,
                   aim_turn_ff_gyro_x,
                   aim_turn_ff_gyro_y,
                   aim_turn_ff_max_step,
                   aim_turn_ff_speed_sps);
    BT_WriteLine(response);
}

static void BT_SendSetResult(const char *name, float value)
{
    char response[80];
    float actual_value = value;

    if ((Track_GetParam(name, &actual_value) == 0U) &&
        (Turn_GetParam(name, &actual_value) == 0U) &&
        (Aim_GetParam(name, &actual_value) == 0U))
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

    if (Aim_SetParam(name, value) != 0U)
    {
        return 1U;
    }

    return 0U;
}
