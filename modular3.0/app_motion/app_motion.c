#include "app_motion.h"

#include "bsp_dcmotor.h"
#include "bsp_encoder.h"
#include "main.h"
#include "tim.h"

typedef struct
{
    DCMotor_t motor;
    BSP_Encoder_t encoder;
    float duty;
} Motion_Wheel_t;

#define APP_MOTION_ENCODER_UPDATE_MS      (1U)
#define APP_MOTION_ENCODER_COUNTS_PER_REV (1)
#define APP_MOTION_ENCODER_UPDATE_HZ      (1000.0f)

static Motion_Wheel_t g_wheels[APP_MOTION_WHEEL_COUNT] =
{
    [MOTION_WHEEL_LF] = {
        .motor = {
            .htim = &htim2,
            .tim_channel = TIM_CHANNEL_3,
            .dir_gpio_port = AIN1_GPIO_Port,
            .dir_pin = AIN1_Pin,
            .dir_forward_state = GPIO_PIN_RESET,
            .dir_reverse_state = GPIO_PIN_SET,
            .dir_aux_gpio_port = AIN2_GPIO_Port,
            .dir_aux_pin = AIN2_Pin,
            .dir_aux_forward_state = GPIO_PIN_SET,
            .dir_aux_reverse_state = GPIO_PIN_RESET,
            .reverse_flag = 0U,
            .stop_use_active_brake = 0U,
            .brake_duty_percent = 0.0f,
            .brake_dir_state = GPIO_PIN_RESET,
            .brake_aux_dir_state = GPIO_PIN_RESET,
        },
        .encoder = {
            .htim = &htim3,
            .counts_per_rev = APP_MOTION_ENCODER_COUNTS_PER_REV,
            .update_hz = APP_MOTION_ENCODER_UPDATE_HZ,
            .reverse_flag = 1U,
        },
        .duty = 0.0f,
    },
    [MOTION_WHEEL_RF] = {
        .motor = {
            .htim = &htim2,
            .tim_channel = TIM_CHANNEL_4,
            .dir_gpio_port = BIN1_GPIO_Port,
            .dir_pin = BIN1_Pin,
            .dir_forward_state = GPIO_PIN_RESET,
            .dir_reverse_state = GPIO_PIN_SET,
            .dir_aux_gpio_port = BIN2_GPIO_Port,
            .dir_aux_pin = BIN2_Pin,
            .dir_aux_forward_state = GPIO_PIN_SET,
            .dir_aux_reverse_state = GPIO_PIN_RESET,
            .reverse_flag = 0U,
            .stop_use_active_brake = 0U,
            .brake_duty_percent = 0.0f,
            .brake_dir_state = GPIO_PIN_RESET,
            .brake_aux_dir_state = GPIO_PIN_RESET,
        },
        .encoder = {
            .htim = &htim4,
            .counts_per_rev = APP_MOTION_ENCODER_COUNTS_PER_REV,
            .update_hz = APP_MOTION_ENCODER_UPDATE_HZ,
            .reverse_flag = 0U,
        },
        .duty = 0.0f,
    },
    [MOTION_WHEEL_LB] = {
        .motor = {
            .htim = &htim1,
            .tim_channel = TIM_CHANNEL_3,
            .dir_gpio_port = TB2_AIN1_GPIO_Port,
            .dir_pin = TB2_AIN1_Pin,
            .dir_forward_state = GPIO_PIN_RESET,
            .dir_reverse_state = GPIO_PIN_SET,
            .dir_aux_gpio_port = TB2_AIN2_GPIO_Port,
            .dir_aux_pin = TB2_AIN2_Pin,
            .dir_aux_forward_state = GPIO_PIN_SET,
            .dir_aux_reverse_state = GPIO_PIN_RESET,
            .reverse_flag = 0U,
            .stop_use_active_brake = 0U,
            .brake_duty_percent = 0.0f,
            .brake_dir_state = GPIO_PIN_RESET,
            .brake_aux_dir_state = GPIO_PIN_RESET,
        },
        .encoder = {
            .htim = &htim5,
            .counts_per_rev = APP_MOTION_ENCODER_COUNTS_PER_REV,
            .update_hz = APP_MOTION_ENCODER_UPDATE_HZ,
            .reverse_flag = 1U,
        },
        .duty = 0.0f,
    },
    [MOTION_WHEEL_RB] = {
        .motor = {
            .htim = &htim1,
            .tim_channel = TIM_CHANNEL_4,
            .dir_gpio_port = TB2_BIN1_GPIO_Port,
            .dir_pin = TB2_BIN1_Pin,
            .dir_forward_state = GPIO_PIN_RESET,
            .dir_reverse_state = GPIO_PIN_SET,
            .dir_aux_gpio_port = TB2_BIN2_GPIO_Port,
            .dir_aux_pin = TB2_BIN2_Pin,
            .dir_aux_forward_state = GPIO_PIN_SET,
            .dir_aux_reverse_state = GPIO_PIN_RESET,
            .reverse_flag = 0U,
            .stop_use_active_brake = 0U,
            .brake_duty_percent = 0.0f,
            .brake_dir_state = GPIO_PIN_RESET,
            .brake_aux_dir_state = GPIO_PIN_RESET,
        },
        .encoder = {
            .htim = &htim8,
            .counts_per_rev = APP_MOTION_ENCODER_COUNTS_PER_REV,
            .update_hz = APP_MOTION_ENCODER_UPDATE_HZ,
            .reverse_flag = 0U,
        },
        .duty = 0.0f,
    },
};

static uint8_t g_motion_initialized = 0U;
static uint32_t g_last_encoder_update_ms = 0U;

static float Motion_Clamp(float value, float min_value, float max_value);
static void Motion_SetDutyArray(const float duty[APP_MOTION_WHEEL_COUNT]);

void Motion_Init(void)
{
    uint32_t i;

    for (i = 0U; i < APP_MOTION_WHEEL_COUNT; i++)
    {
        BSP_DCMotor_Init(&g_wheels[i].motor);
        BSP_DCMotor_Stop(&g_wheels[i].motor);
        BSP_Encoder_Init(&g_wheels[i].encoder);
        g_wheels[i].duty = 0.0f;
    }

    g_last_encoder_update_ms = HAL_GetTick();
    g_motion_initialized = 1U;
}

void Motion_Poll(void)
{
    uint32_t i;
    uint32_t now_ms;

    if (g_motion_initialized == 0U)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - g_last_encoder_update_ms) < APP_MOTION_ENCODER_UPDATE_MS)
    {
        return;
    }

    g_last_encoder_update_ms = now_ms;
    for (i = 0U; i < APP_MOTION_WHEEL_COUNT; i++)
    {
        BSP_Encoder_Update(&g_wheels[i].encoder);
    }
}

void Motion_SetDuty4(float lf, float rf, float lb, float rb)
{
    float duty[APP_MOTION_WHEEL_COUNT];

    duty[MOTION_WHEEL_LF] = lf;
    duty[MOTION_WHEEL_RF] = rf;
    duty[MOTION_WHEEL_LB] = lb;
    duty[MOTION_WHEEL_RB] = rb;
    Motion_SetDutyArray(duty);
}

void Motion_Stop(void)
{
    uint32_t i;

    if (g_motion_initialized == 0U)
    {
        return;
    }

    for (i = 0U; i < APP_MOTION_WHEEL_COUNT; i++)
    {
        g_wheels[i].duty = 0.0f;
        BSP_DCMotor_Stop(&g_wheels[i].motor);
    }
}

float Motion_GetDuty(uint8_t wheel_id)
{
    if (wheel_id >= APP_MOTION_WHEEL_COUNT)
    {
        return 0.0f;
    }

    return g_wheels[wheel_id].duty;
}

int32_t Motion_GetEncoderAccum(uint8_t wheel_id)
{
    if (wheel_id >= APP_MOTION_WHEEL_COUNT)
    {
        return 0;
    }

    return BSP_Encoder_GetAccumCount(&g_wheels[wheel_id].encoder);
}

int32_t Motion_GetEncoderDelta(uint8_t wheel_id)
{
    if (wheel_id >= APP_MOTION_WHEEL_COUNT)
    {
        return 0;
    }

    return BSP_Encoder_GetDeltaCount(&g_wheels[wheel_id].encoder);
}

static float Motion_Clamp(float value, float min_value, float max_value)
{
    if (value != value)
    {
        return 0.0f;
    }

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

static void Motion_SetDutyArray(const float duty[APP_MOTION_WHEEL_COUNT])
{
    uint32_t i;
    float clamped;

    if ((g_motion_initialized == 0U) || (duty == (const float *)0))
    {
        return;
    }

    for (i = 0U; i < APP_MOTION_WHEEL_COUNT; i++)
    {
        clamped = Motion_Clamp(duty[i], -100.0f, 100.0f);
        g_wheels[i].duty = clamped;
        if (clamped == 0.0f)
        {
            BSP_DCMotor_Stop(&g_wheels[i].motor);
        }
        else
        {
            BSP_DCMotor_SetDuty(&g_wheels[i].motor, clamped);
        }
    }
}
