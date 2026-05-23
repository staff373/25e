/*
 * bsp_sensor.h
 * 五路灰度传感器（红外反射式）
 * 用于黑线循迹检测
 *
 * 传感器布局: L2(最左) L1(左) M(中) R1(右) R2(最右)
 * 黑线检测输出1，白底输出0
 */
#ifndef SENSOR_H
#define SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define SENSOR_CHANNEL_COUNT             (5U)
#define SENSOR_DEFAULT_LOST_THRESHOLD    (1250U)
#define SENSOR_DEFAULT_MAJORITY_SAMPLES  (3U)

/* 传感器引脚定义 */
#define SENSOR_L2_GPIO_Port GPIOC
#define SENSOR_L2_Pin GPIO_PIN_0

#define SENSOR_L1_GPIO_Port GPIOC
#define SENSOR_L1_Pin GPIO_PIN_1

#define SENSOR_M_GPIO_Port GPIOC
#define SENSOR_M_Pin GPIO_PIN_2

#define SENSOR_R1_GPIO_Port GPIOC
#define SENSOR_R1_Pin GPIO_PIN_3

#define SENSOR_R2_GPIO_Port GPIOB
#define SENSOR_R2_Pin GPIO_PIN_9

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} Sensor_Pin_t;

typedef struct
{
    Sensor_Pin_t pins[SENSOR_CHANNEL_COUNT];
    uint32_t lost_threshold;
    uint8_t majority_samples;
} Sensor_Config_t;

/* 传感器状态 */
typedef enum
{
    SENSOR_STATE_NORMAL = 0,    /* 正常循迹 */
    SENSOR_STATE_SPIN_LEFT,     /* 左直角弯（11100/11110） */
    SENSOR_STATE_SPIN_RIGHT,    /* 右直角弯（00111/01111） */
    SENSOR_STATE_STOP_BLACK,    /* 全黑/宽黑（11111，是否停车由应用层决定） */
    SENSOR_STATE_LOST           /* 丢线停车（连续全白） */
} Sensor_State_t;

/* 传感器数据结构 */
typedef struct
{
    Sensor_Config_t config;
    uint8_t sensor[SENSOR_CHANNEL_COUNT];  /* 兼容旧字段语义 */
    uint8_t raw[SENSOR_CHANNEL_COUNT];
    float norm[SENSOR_CHANNEL_COUNT];
    uint8_t state;                         /* 压缩后的状态值 0-31 */
    float error;                           /* 误差值，负值偏左，正值偏右 */
    float norm_error;                      /* 归一化误差，范围[-1,1] */
    Sensor_State_t state_type;             /* 状态类型 */
    uint32_t lost_count;                   /* 实例级连续全白计数 */
} Sensor_t;

/* 初始化 */
void Sensor_Init(Sensor_t *s);
void Sensor_InitWithConfig(Sensor_t *s, const Sensor_Config_t *cfg);

/* 读取传感器并计算状态和误差 */
void Sensor_Read(Sensor_t *s);

/* 获取状态类型 */
Sensor_State_t Sensor_GetStateType(Sensor_t *s);

/* 获取误差值 */
float Sensor_GetError(Sensor_t *s);
float Sensor_GetNormError(const Sensor_t *s);

/* 获取原始状态值 */
uint8_t Sensor_GetRawState(Sensor_t *s);
void Sensor_GetRawArray(const Sensor_t *s, uint8_t out[SENSOR_CHANNEL_COUNT]);
void Sensor_GetNormArray(const Sensor_t *s, float out[SENSOR_CHANNEL_COUNT]);

/* 检测是否为右转弯道模式
 * 右转模式: 01111(0x0F), 00111(0x07), 00011(0x03)
 * 返回: 1=是右转模式, 0=不是
 */
uint8_t Sensor_IsRightTurnPattern(Sensor_t *s);

/* 检测是否为左转弯道模式
 * 左转模式: 11110(0x1E), 11100(0x1C), 11000(0x18)
 * 返回: 1=是左转模式, 0=不是
 */
uint8_t Sensor_IsLeftTurnPattern(Sensor_t *s);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_H */
