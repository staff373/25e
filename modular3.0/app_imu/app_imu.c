#include "app_imu.h"

#include "bsp_jy61p.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#define APP_IMU_OFFLINE_TIMEOUT_MS (200U)
#define APP_IMU_RAD_TO_DEG         (57.29577951308232f)

static BSP_JY61P_t g_imu;
static int g_imu_init_status = BSP_JY61P_ERR_ARG;

void Imu_Init(void)
{
    (void)memset(&g_imu, 0, sizeof(g_imu));
    g_imu.huart = &huart3;
    g_imu.offline_timeout_ms = APP_IMU_OFFLINE_TIMEOUT_MS;
    g_imu_init_status = BSP_JY61P_Init(&g_imu);
}

void Imu_Poll(void)
{
    if (g_imu_init_status != BSP_JY61P_OK)
    {
        return;
    }

    BSP_JY61P_Update(&g_imu);
}

void Imu_HandleRxCplt(UART_HandleTypeDef *huart)
{
    if ((huart == g_imu.huart) && (g_imu.huart != (UART_HandleTypeDef *)0))
    {
        BSP_JY61P_HandleRxCplt(&g_imu);
    }
}

void Imu_HandleUartError(UART_HandleTypeDef *huart)
{
    if ((huart == g_imu.huart) && (g_imu.huart != (UART_HandleTypeDef *)0))
    {
        BSP_JY61P_HandleUartError(&g_imu);
    }
}

int Imu_GetInitStatus(void)
{
    return g_imu_init_status;
}

uint8_t Imu_IsOnline(void)
{
    if (g_imu_init_status != BSP_JY61P_OK)
    {
        return 0U;
    }

    return BSP_JY61P_IsOnline(&g_imu);
}

float Imu_GetYawDeg(void)
{
    if (g_imu_init_status != BSP_JY61P_OK)
    {
        return 0.0f;
    }

    return BSP_JY61P_GetYaw(&g_imu) * APP_IMU_RAD_TO_DEG;
}

float Imu_GetGyroZDps(void)
{
    if (g_imu_init_status != BSP_JY61P_OK)
    {
        return 0.0f;
    }

    return g_imu.gyro_z_dps;
}

void Imu_FormatStatus(char *buffer, size_t buffer_size, const char *prefix)
{
    const char *resolved_prefix = prefix;

    if ((buffer == (char *)0) || (buffer_size == 0U))
    {
        return;
    }

    if (resolved_prefix == (const char *)0)
    {
        resolved_prefix = (g_imu_init_status == BSP_JY61P_OK) ? "OK" : "ERR";
    }

    (void)snprintf(buffer,
                   buffer_size,
                   "%s IMU init=%d online=%u yaw=%.2f gyro_z=%.2f",
                   resolved_prefix,
                   g_imu_init_status,
                   (unsigned int)Imu_IsOnline(),
                   Imu_GetYawDeg(),
                   Imu_GetGyroZDps());
}
