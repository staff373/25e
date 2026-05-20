#include "app_sensor.h"

#include "main.h"

#include <stdio.h>

#define APP_SENSOR_POLL_PERIOD_MS (10U)

static Sensor_t g_sensor;
static uint8_t g_sensor_initialized = 0U;
static uint32_t g_last_sensor_read_ms = 0U;

static const char *AppSensor_StateName(Sensor_State_t state_type);

void AppSensor_Init(void)
{
    Sensor_Config_t cfg;

    cfg.pins[0].port = L2_GPIO_Port;
    cfg.pins[0].pin = L2_Pin;
    cfg.pins[1].port = L1_GPIO_Port;
    cfg.pins[1].pin = L1_Pin;
    cfg.pins[2].port = M_GPIO_Port;
    cfg.pins[2].pin = M_Pin;
    cfg.pins[3].port = R1_GPIO_Port;
    cfg.pins[3].pin = R1_Pin;
    cfg.pins[4].port = R2_GPIO_Port;
    cfg.pins[4].pin = R2_Pin;
    cfg.lost_threshold = SENSOR_DEFAULT_LOST_THRESHOLD;
    cfg.majority_samples = SENSOR_DEFAULT_MAJORITY_SAMPLES;

    Sensor_InitWithConfig(&g_sensor, &cfg);
    AppSensor_ReadNow();
    g_last_sensor_read_ms = HAL_GetTick();
    g_sensor_initialized = 1U;
}

void AppSensor_Poll(void)
{
    uint32_t now_ms;

    if (g_sensor_initialized == 0U)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - g_last_sensor_read_ms) >= APP_SENSOR_POLL_PERIOD_MS)
    {
        g_last_sensor_read_ms = now_ms;
        AppSensor_ReadNow();
    }
}

void AppSensor_ReadNow(void)
{
    Sensor_Read(&g_sensor);
}

uint8_t AppSensor_GetRawState(void)
{
    return Sensor_GetRawState(&g_sensor);
}

void AppSensor_GetRawArray(uint8_t out[SENSOR_CHANNEL_COUNT])
{
    Sensor_GetRawArray(&g_sensor, out);
}

float AppSensor_GetNormError(void)
{
    return Sensor_GetNormError(&g_sensor);
}

Sensor_State_t AppSensor_GetStateType(void)
{
    return Sensor_GetStateType(&g_sensor);
}

int8_t AppSensor_GetCornerDirection(void)
{
    switch (Sensor_GetStateType(&g_sensor))
    {
    case SENSOR_STATE_SPIN_LEFT:
        return -1;
    case SENSOR_STATE_SPIN_RIGHT:
        return 1;
    default:
        return 0;
    }
}

uint8_t AppSensor_IsAllWhite(void)
{
    return (uint8_t)(Sensor_GetRawState(&g_sensor) == 0x00U);
}

uint8_t AppSensor_IsAllBlack(void)
{
    return (uint8_t)(Sensor_GetRawState(&g_sensor) == 0x1FU);
}

void AppSensor_FormatStatus(char *buffer, size_t buffer_size, const char *prefix)
{
    uint8_t raw[SENSOR_CHANNEL_COUNT];

    if ((buffer == (char *)0) || (buffer_size == 0U))
    {
        return;
    }

    if (prefix == (const char *)0)
    {
        prefix = "OK";
    }

    Sensor_GetRawArray(&g_sensor, raw);
    (void)snprintf(buffer,
                   buffer_size,
                   "%s SENSOR raw=%u%u%u%u%u state=0x%02X norm=%.3f type=%s",
                   prefix,
                   (unsigned int)raw[0],
                   (unsigned int)raw[1],
                   (unsigned int)raw[2],
                   (unsigned int)raw[3],
                   (unsigned int)raw[4],
                   (unsigned int)Sensor_GetRawState(&g_sensor),
                   Sensor_GetNormError(&g_sensor),
                   AppSensor_StateName(Sensor_GetStateType(&g_sensor)));
}

static const char *AppSensor_StateName(Sensor_State_t state_type)
{
    switch (state_type)
    {
    case SENSOR_STATE_NORMAL:
        return "NORMAL";
    case SENSOR_STATE_SPIN_LEFT:
        return "CORNER_L";
    case SENSOR_STATE_SPIN_RIGHT:
        return "CORNER_R";
    case SENSOR_STATE_STOP_BLACK:
        return "ALL_BLACK";
    case SENSOR_STATE_LOST:
        return "LOST";
    default:
        return "UNKNOWN";
    }
}
