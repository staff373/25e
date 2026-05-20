/*
 * bsp_sensor.c
 * 五路灰度传感器实现
 * 黑线输出1，白底输出0
 */
#include "bsp_sensor.h"
#include <string.h>

static const Sensor_Config_t g_sensor_default_config =
{
    .pins =
    {
        {SENSOR_L2_GPIO_Port, SENSOR_L2_Pin},
        {SENSOR_L1_GPIO_Port, SENSOR_L1_Pin},
        {SENSOR_M_GPIO_Port, SENSOR_M_Pin},
        {SENSOR_R1_GPIO_Port, SENSOR_R1_Pin},
        {SENSOR_R2_GPIO_Port, SENSOR_R2_Pin}
    },
    .lost_threshold = SENSOR_DEFAULT_LOST_THRESHOLD,
    .majority_samples = SENSOR_DEFAULT_MAJORITY_SAMPLES
};

static float Sensor_CalcWeightedError(uint8_t state)
{
    static const float weights[5] = {-8.0f, -4.0f, 0.0f, 4.0f, 8.0f};
    float sum = 0.0f;
    float count = 0.0f;
    uint8_t i;

    for (i = 0U; i < 5U; i++)
    {
        if (((state >> (4U - i)) & 0x01U) != 0U)
        {
            sum += weights[i];
            count += 1.0f;
        }
    }

    if (count <= 0.0f)
    {
        return 0.0f;
    }

    return sum / count;
}

static float Sensor_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static float Sensor_CalcNormError(float error)
{
    return Sensor_ClampFloat(error / 8.0f, -1.0f, 1.0f);
}

static void Sensor_LoadDefaultConfig(Sensor_Config_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    *cfg = g_sensor_default_config;
}

static void Sensor_NormalizeConfig(Sensor_Config_t *cfg)
{
    uint8_t i;

    if (cfg == NULL)
    {
        return;
    }

    for (i = 0U; i < SENSOR_CHANNEL_COUNT; i++)
    {
        if ((cfg->pins[i].port == (GPIO_TypeDef *)0) || (cfg->pins[i].pin == 0U))
        {
            cfg->pins[i] = g_sensor_default_config.pins[i];
        }
    }

    if (cfg->lost_threshold == 0U)
    {
        cfg->lost_threshold = SENSOR_DEFAULT_LOST_THRESHOLD;
    }

    if (cfg->majority_samples == 0U)
    {
        cfg->majority_samples = SENSOR_DEFAULT_MAJORITY_SAMPLES;
    }
}

static uint8_t Sensor_ReadPinMajority(GPIO_TypeDef *port, uint16_t pin, uint8_t sample_count)
{
    uint8_t high_count = 0U;
    uint8_t i;
    uint8_t majority_threshold;

    if ((port == (GPIO_TypeDef *)0) || (pin == 0U))
    {
        return 0U;
    }

    if (sample_count == 0U)
    {
        sample_count = SENSOR_DEFAULT_MAJORITY_SAMPLES;
    }

    majority_threshold = (uint8_t)((sample_count / 2U) + 1U);

    for (i = 0U; i < sample_count; i++)
    {
        if (HAL_GPIO_ReadPin(port, pin) != GPIO_PIN_RESET)
        {
            high_count++;
        }
    }

    return (high_count >= majority_threshold) ? 1U : 0U;
}

void Sensor_Init(Sensor_t *s)
{
    Sensor_InitWithConfig(s, (const Sensor_Config_t *)0);
}

void Sensor_InitWithConfig(Sensor_t *s, const Sensor_Config_t *cfg)
{
    if (s == NULL)
    {
        return;
    }

    (void)memset(s, 0, sizeof(Sensor_t));

    if (cfg != NULL)
    {
        s->config = *cfg;
    }
    else
    {
        Sensor_LoadDefaultConfig(&s->config);
    }

    Sensor_NormalizeConfig(&s->config);
    s->error = 0.0f;
    s->norm_error = 0.0f;
    s->state_type = SENSOR_STATE_NORMAL;
    s->lost_count = 0U;
}

void Sensor_Read(Sensor_t *s)
{
    uint8_t i;

    if (s == NULL)
    {
        return;
    }

    /* 读取5个传感器 */
    for (i = 0U; i < SENSOR_CHANNEL_COUNT; i++)
    {
        uint8_t raw_value = Sensor_ReadPinMajority(s->config.pins[i].port,
                                                   s->config.pins[i].pin,
                                                   s->config.majority_samples);
        s->sensor[i] = raw_value;
        s->raw[i] = raw_value;
        s->norm[i] = (float)raw_value;
    }

    /* 压缩状态: (L2<<4) | (L1<<3) | (M<<2) | (R1<<1) | R2 */
    s->state = (s->sensor[0] << 4) | (s->sensor[1] << 3) |
               (s->sensor[2] << 2) | (s->sensor[3] << 1) | s->sensor[4];

    /* 根据状态值计算误差 */
    switch (s->state)
    {
    case 0x04:  /* 00100: 居中 */
        s->error = 0.0f;
        break;
    case 0x0C:  /* 01100: 偏左 */
        s->error = -1.0f;
        break;
    case 0x06:  /* 00110: 偏右 */
        s->error = 1.0f;
        break;
    case 0x08:  /* 01000: 较偏左 */
        s->error = -2.5f;
        break;
    case 0x02:  /* 00010: 较偏右 */
        s->error = 2.5f;
        break;
    case 0x10:  /* 10000: 极左 */
        s->error = -8.0f;           // -5.0f;
        break;
    case 0x01:  /* 00001: 极右 */
        s->error = 8.0f;           // 5.0f;
        break;
    case 0x1C:  /* 11100: 左直角弯 */
        s->error = -8.0f;
        break;
    case 0x1E:  /* 11110: 左强直角弯 */
        s->error = -8.0f;
        break;
    case 0x07:  /* 00111: 右直角弯 */
        s->error = 8.0f;
        break;
    case 0x0F:  /* 01111: 右强直角弯 */
        s->error = 8.0f;
        break;
    case 0x00:  /* 00000: 全白/丢线 */
        /* 丢线计数 */
        s->lost_count++;
        if (s->lost_count >= s->config.lost_threshold)
        {
            s->state_type = SENSOR_STATE_LOST;
        }
        else
        {
            s->state_type = SENSOR_STATE_NORMAL;
        }
        s->error = 0.0f;  /* 保持上次误差或0 */
        s->norm_error = Sensor_CalcNormError(s->error);
        return;
    case 0x1F:  /* 11111: 全黑/宽黑 */
        s->error = 0.0f;
        s->state_type = SENSOR_STATE_STOP_BLACK;
        s->norm_error = Sensor_CalcNormError(s->error);
        return;
    default:
        /* 其他组合状态用加权中心估计，避免沿用上一帧误差 */
        s->error = Sensor_CalcWeightedError(s->state);
        break;
    }

    /* 重置丢线计数 */
    s->lost_count = 0U;

    /* 判断特殊状态 */
    switch (s->state)
    {
    case 0x1C:  /* 11100: 左直角弯 */
    case 0x1E:  /* 11110: 左强直角弯 */
        s->state_type = SENSOR_STATE_SPIN_LEFT;
        break;
    case 0x07:  /* 00111: 右直角弯 */
    case 0x0F:  /* 01111: 右强直角弯 */
        s->state_type = SENSOR_STATE_SPIN_RIGHT;
        break;
    default:
        s->state_type = SENSOR_STATE_NORMAL;
        break;
    }

    s->norm_error = Sensor_CalcNormError(s->error);
}

Sensor_State_t Sensor_GetStateType(Sensor_t *s)
{
    if (s == NULL)
    {
        return SENSOR_STATE_NORMAL;
    }
    return s->state_type;
}

float Sensor_GetError(Sensor_t *s)
{
    if (s == NULL)
    {
        return 0.0f;
    }
    return s->error;
}

float Sensor_GetNormError(const Sensor_t *s)
{
    if (s == NULL)
    {
        return 0.0f;
    }
    return s->norm_error;
}

uint8_t Sensor_GetRawState(Sensor_t *s)
{
    if (s == NULL)
    {
        return 0U;
    }
    return s->state;
}

void Sensor_GetRawArray(const Sensor_t *s, uint8_t out[SENSOR_CHANNEL_COUNT])
{
    if (out == (uint8_t *)0)
    {
        return;
    }

    if (s == (const Sensor_t *)0)
    {
        (void)memset(out, 0, SENSOR_CHANNEL_COUNT);
        return;
    }

    (void)memcpy(out, s->raw, SENSOR_CHANNEL_COUNT);
}

void Sensor_GetNormArray(const Sensor_t *s, float out[SENSOR_CHANNEL_COUNT])
{
    if (out == (float *)0)
    {
        return;
    }

    if (s == (const Sensor_t *)0)
    {
        uint8_t i;
        for (i = 0U; i < SENSOR_CHANNEL_COUNT; i++)
        {
            out[i] = 0.0f;
        }
        return;
    }

    (void)memcpy(out, s->norm, sizeof(s->norm));
}

uint8_t Sensor_IsRightTurnPattern(Sensor_t *s)
{
    if (s == NULL)
    {
        return 0U;
    }
    /* 右转模式: 01111(0x0F), 00111(0x07), 00011(0x03) */
    return (s->state == 0x0F) || (s->state == 0x07) || (s->state == 0x03);
}

uint8_t Sensor_IsLeftTurnPattern(Sensor_t *s)
{
    if (s == NULL)
    {
        return 0U;
    }
    /* 左转模式: 11110(0x1E), 11100(0x1C), 11000(0x18) */
    return (s->state == 0x1E) || (s->state == 0x1C) || (s->state == 0x18);
}

