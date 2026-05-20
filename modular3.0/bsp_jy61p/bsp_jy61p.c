/*
 * bsp_jy61p.c
 * JY61P UART 姿态采集实现：
 * - 参考外部整机工程中的初始化、探测、寄存器配置与 11 字节帧解析逻辑
 * - 在 BSP 内部托管单字节中断接收
 * - 仅对外暴露姿态缓存与在线状态
 */
#include "bsp_jy61p.h"

#define BSP_JY61P_FRAME_SIZE                (11U)
#define BSP_JY61P_FRAME_HEAD                (0x55U)
#define BSP_JY61P_TYPE_ACC                  (0x51U)
#define BSP_JY61P_TYPE_GYRO                 (0x52U)
#define BSP_JY61P_TYPE_ANGLE                (0x53U)

#define BSP_JY61P_DEFAULT_BAUDRATE          (115200U)
#define BSP_JY61P_FALLBACK_BAUDRATE         (9600U)
#define BSP_JY61P_DEFAULT_OFFLINE_TIMEOUT   (100U)
#define BSP_JY61P_DETECT_TIMEOUT_MS         (300U)
#define BSP_JY61P_CONFIG_STARTUP_DELAY_MS   (200U)
#define BSP_JY61P_CONFIG_STEP_DELAY_MS      (80U)

#define BSP_JY61P_PI                        (3.14159265358979323846f)
#define BSP_JY61P_DEG_TO_RAD                (BSP_JY61P_PI / 180.0f)

static uint8_t BSP_JY61P_IsBindingValid(const BSP_JY61P_t *imu);
static void BSP_JY61P_ResetRuntime(BSP_JY61P_t *imu);
static int BSP_JY61P_StartReceive(BSP_JY61P_t *imu);
static uint8_t BSP_JY61P_WaitForSample(BSP_JY61P_t *imu, uint32_t timeout_ms);
static int BSP_JY61P_SetBaud(BSP_JY61P_t *imu, uint32_t baudrate);
static int BSP_JY61P_SendCommand(BSP_JY61P_t *imu, uint8_t reg, uint16_t value);
static int BSP_JY61P_WriteRegister(BSP_JY61P_t *imu, uint8_t reg, uint16_t value);
static int BSP_JY61P_ApplyConfiguration(BSP_JY61P_t *imu);
static void BSP_JY61P_ParseByte(BSP_JY61P_t *imu, uint8_t byte);
static void BSP_JY61P_ProcessFrame(BSP_JY61P_t *imu, const uint8_t *frame);
static uint8_t BSP_JY61P_IsFrameType(uint8_t type);
static uint8_t BSP_JY61P_Checksum(const uint8_t *frame);
static int16_t BSP_JY61P_ReadInt16(const uint8_t *buf);
static float BSP_JY61P_NormalizeAngle180(float angle_deg);

int BSP_JY61P_Init(BSP_JY61P_t *imu)
{
    if (imu == (BSP_JY61P_t *)0)
    {
        return BSP_JY61P_ERR_ARG;
    }

    if (BSP_JY61P_IsBindingValid(imu) == 0U)
    {
        return BSP_JY61P_ERR_BINDING;
    }

    imu->baudrate = BSP_JY61P_DEFAULT_BAUDRATE;
    if (imu->offline_timeout_ms == 0U)
    {
        imu->offline_timeout_ms = BSP_JY61P_DEFAULT_OFFLINE_TIMEOUT;
    }

    HAL_Delay(BSP_JY61P_CONFIG_STARTUP_DELAY_MS);

    if (BSP_JY61P_SetBaud(imu, BSP_JY61P_DEFAULT_BAUDRATE) != BSP_JY61P_OK)
    {
        return BSP_JY61P_ERR_UART_REINIT;
    }

    BSP_JY61P_ResetRuntime(imu);
    if (BSP_JY61P_WaitForSample(imu, BSP_JY61P_DETECT_TIMEOUT_MS) == 0U)
    {
        if (BSP_JY61P_SetBaud(imu, BSP_JY61P_FALLBACK_BAUDRATE) != BSP_JY61P_OK)
        {
            return BSP_JY61P_ERR_UART_REINIT;
        }

        BSP_JY61P_ResetRuntime(imu);
        if (BSP_JY61P_WaitForSample(imu, BSP_JY61P_DETECT_TIMEOUT_MS) == 0U)
        {
            if (BSP_JY61P_SetBaud(imu, BSP_JY61P_DEFAULT_BAUDRATE) != BSP_JY61P_OK)
            {
                return BSP_JY61P_ERR_UART_REINIT;
            }

            BSP_JY61P_ResetRuntime(imu);
        }
    }

    if (BSP_JY61P_ApplyConfiguration(imu) != BSP_JY61P_OK)
    {
        return BSP_JY61P_ERR_UART_TX;
    }

    if (imu->huart->Init.BaudRate != BSP_JY61P_DEFAULT_BAUDRATE)
    {
        if (BSP_JY61P_SetBaud(imu, BSP_JY61P_DEFAULT_BAUDRATE) != BSP_JY61P_OK)
        {
            return BSP_JY61P_ERR_UART_REINIT;
        }
    }

    BSP_JY61P_ResetRuntime(imu);
    if (BSP_JY61P_StartReceive(imu) != BSP_JY61P_OK)
    {
        return BSP_JY61P_ERR_RX_START;
    }

    imu->initialized = 1U;
    return BSP_JY61P_OK;
}

void BSP_JY61P_Update(BSP_JY61P_t *imu)
{
    uint32_t now_ms;
    uint32_t elapsed_ms;

    if ((imu == (BSP_JY61P_t *)0) || (imu->initialized == 0U))
    {
        return;
    }

    if (BSP_JY61P_IsBindingValid(imu) == 0U)
    {
        return;
    }

    now_ms = HAL_GetTick();
    elapsed_ms = (uint32_t)(now_ms - imu->last_frame_ms);

    if ((imu->data_valid != 0U) && (elapsed_ms <= imu->offline_timeout_ms))
    {
        imu->online = 1U;
    }
    else
    {
        imu->online = 0U;
    }
}

void BSP_JY61P_HandleRxCplt(BSP_JY61P_t *imu)
{
    if (imu == (BSP_JY61P_t *)0)
    {
        return;
    }

    BSP_JY61P_ParseByte(imu, imu->rx_byte);
    if (BSP_JY61P_StartReceive(imu) != BSP_JY61P_OK)
    {
        imu->online = 0U;
    }
}

void BSP_JY61P_HandleUartError(BSP_JY61P_t *imu)
{
    if ((imu == (BSP_JY61P_t *)0) || (BSP_JY61P_IsBindingValid(imu) == 0U))
    {
        return;
    }

    imu->frame_index = 0U;
    imu->rx_byte = 0U;

    __HAL_UART_CLEAR_PEFLAG(imu->huart);
    __HAL_UART_CLEAR_FEFLAG(imu->huart);
    __HAL_UART_CLEAR_NEFLAG(imu->huart);
    __HAL_UART_CLEAR_OREFLAG(imu->huart);

    if (BSP_JY61P_StartReceive(imu) != BSP_JY61P_OK)
    {
        imu->online = 0U;
    }
}

static uint8_t BSP_JY61P_IsBindingValid(const BSP_JY61P_t *imu)
{
    if (imu == (const BSP_JY61P_t *)0)
    {
        return 0U;
    }

    if (imu->huart == (UART_HandleTypeDef *)0)
    {
        return 0U;
    }

    return 1U;
}

static void BSP_JY61P_ResetRuntime(BSP_JY61P_t *imu)
{
    if (imu == (BSP_JY61P_t *)0)
    {
        return;
    }

    imu->rx_byte = 0U;
    imu->frame_index = 0U;

    imu->pitch_rad = 0.0f;
    imu->roll_rad = 0.0f;
    imu->yaw_rad = 0.0f;

    imu->acc_x_g = 0.0f;
    imu->acc_y_g = 0.0f;
    imu->acc_z_g = 0.0f;
    imu->gyro_x_dps = 0.0f;
    imu->gyro_y_dps = 0.0f;
    imu->gyro_z_dps = 0.0f;
    imu->temperature_c = 0.0f;

    imu->last_frame_ms = 0U;
    imu->sample_seq = 0U;
    imu->initialized = 0U;
    imu->online = 0U;
    imu->data_valid = 0U;
}

static int BSP_JY61P_StartReceive(BSP_JY61P_t *imu)
{
    if (BSP_JY61P_IsBindingValid(imu) == 0U)
    {
        return BSP_JY61P_ERR_BINDING;
    }

    if (HAL_UART_Receive_IT(imu->huart, &imu->rx_byte, 1U) != HAL_OK)
    {
        return BSP_JY61P_ERR_RX_START;
    }

    return BSP_JY61P_OK;
}

static uint8_t BSP_JY61P_WaitForSample(BSP_JY61P_t *imu, uint32_t timeout_ms)
{
    uint32_t start_ms;
    uint32_t base_seq;

    if (imu == (BSP_JY61P_t *)0)
    {
        return 0U;
    }

    base_seq = imu->sample_seq;
    start_ms = HAL_GetTick();

    if (BSP_JY61P_StartReceive(imu) != BSP_JY61P_OK)
    {
        return 0U;
    }

    while ((uint32_t)(HAL_GetTick() - start_ms) < timeout_ms)
    {
        if (imu->sample_seq != base_seq)
        {
            (void)HAL_UART_AbortReceive(imu->huart);
            return 1U;
        }
    }

    (void)HAL_UART_AbortReceive(imu->huart);
    return 0U;
}

static int BSP_JY61P_SetBaud(BSP_JY61P_t *imu, uint32_t baudrate)
{
    if (BSP_JY61P_IsBindingValid(imu) == 0U)
    {
        return BSP_JY61P_ERR_BINDING;
    }

    (void)HAL_UART_AbortReceive(imu->huart);
    (void)HAL_UART_DeInit(imu->huart);

    imu->huart->Init.BaudRate = baudrate;
    if (HAL_UART_Init(imu->huart) != HAL_OK)
    {
        return BSP_JY61P_ERR_UART_REINIT;
    }

    return BSP_JY61P_OK;
}

static int BSP_JY61P_SendCommand(BSP_JY61P_t *imu, uint8_t reg, uint16_t value)
{
    uint8_t cmd[5];

    if (BSP_JY61P_IsBindingValid(imu) == 0U)
    {
        return BSP_JY61P_ERR_BINDING;
    }

    cmd[0] = 0xFFU;
    cmd[1] = 0xAAU;
    cmd[2] = reg;
    cmd[3] = (uint8_t)(value & 0x00FFU);
    cmd[4] = (uint8_t)((value >> 8) & 0x00FFU);

    if (HAL_UART_Transmit(imu->huart, cmd, sizeof(cmd), HAL_MAX_DELAY) != HAL_OK)
    {
        return BSP_JY61P_ERR_UART_TX;
    }

    return BSP_JY61P_OK;
}

static int BSP_JY61P_WriteRegister(BSP_JY61P_t *imu, uint8_t reg, uint16_t value)
{
    if (BSP_JY61P_SendCommand(imu, 0x69U, 0xB588U) != BSP_JY61P_OK)
    {
        return BSP_JY61P_ERR_UART_TX;
    }
    HAL_Delay(BSP_JY61P_CONFIG_STEP_DELAY_MS);

    if (BSP_JY61P_SendCommand(imu, reg, value) != BSP_JY61P_OK)
    {
        return BSP_JY61P_ERR_UART_TX;
    }
    HAL_Delay(BSP_JY61P_CONFIG_STEP_DELAY_MS);

    return BSP_JY61P_OK;
}

static int BSP_JY61P_ApplyConfiguration(BSP_JY61P_t *imu)
{
    if (BSP_JY61P_WriteRegister(imu, 0x24U, 0x0001U) != BSP_JY61P_OK)
    {
        return BSP_JY61P_ERR_UART_TX;
    }

    if (BSP_JY61P_WriteRegister(imu, 0x02U, 0x000EU) != BSP_JY61P_OK)
    {
        return BSP_JY61P_ERR_UART_TX;
    }

    if (BSP_JY61P_WriteRegister(imu, 0x03U, 0x0007U) != BSP_JY61P_OK)
    {
        return BSP_JY61P_ERR_UART_TX;
    }

    if (BSP_JY61P_WriteRegister(imu, 0x04U, 0x0006U) != BSP_JY61P_OK)
    {
        return BSP_JY61P_ERR_UART_TX;
    }

    if (BSP_JY61P_SendCommand(imu, 0x00U, 0x0000U) != BSP_JY61P_OK)
    {
        return BSP_JY61P_ERR_UART_TX;
    }
    HAL_Delay(BSP_JY61P_CONFIG_STEP_DELAY_MS);

    if (BSP_JY61P_SendCommand(imu, 0x00U, 0x00FFU) != BSP_JY61P_OK)
    {
        return BSP_JY61P_ERR_UART_TX;
    }
    HAL_Delay(BSP_JY61P_CONFIG_STARTUP_DELAY_MS);

    return BSP_JY61P_OK;
}

static void BSP_JY61P_ParseByte(BSP_JY61P_t *imu, uint8_t byte)
{
    if (imu == (BSP_JY61P_t *)0)
    {
        return;
    }

    if (imu->frame_index == 0U)
    {
        if (byte == BSP_JY61P_FRAME_HEAD)
        {
            imu->frame[0] = byte;
            imu->frame_index = 1U;
        }
        return;
    }

    if (imu->frame_index == 1U)
    {
        if (BSP_JY61P_IsFrameType(byte) == 0U)
        {
            imu->frame_index = (byte == BSP_JY61P_FRAME_HEAD) ? 1U : 0U;
            imu->frame[0] = BSP_JY61P_FRAME_HEAD;
            return;
        }
    }

    imu->frame[imu->frame_index++] = byte;

    if (imu->frame_index < BSP_JY61P_FRAME_SIZE)
    {
        return;
    }

    if (BSP_JY61P_Checksum(imu->frame) == imu->frame[BSP_JY61P_FRAME_SIZE - 1U])
    {
        BSP_JY61P_ProcessFrame(imu, imu->frame);
    }

    imu->frame_index = 0U;
}

static void BSP_JY61P_ProcessFrame(BSP_JY61P_t *imu, const uint8_t *frame)
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    if ((imu == (BSP_JY61P_t *)0) || (frame == (const uint8_t *)0))
    {
        return;
    }

    switch (frame[1])
    {
        case BSP_JY61P_TYPE_ACC:
            imu->acc_x_g = ((float)BSP_JY61P_ReadInt16(&frame[2]) / 32768.0f) * 16.0f;
            imu->acc_y_g = ((float)BSP_JY61P_ReadInt16(&frame[4]) / 32768.0f) * 16.0f;
            imu->acc_z_g = ((float)BSP_JY61P_ReadInt16(&frame[6]) / 32768.0f) * 16.0f;
            imu->temperature_c = (float)BSP_JY61P_ReadInt16(&frame[8]) / 100.0f;
            break;

        case BSP_JY61P_TYPE_GYRO:
            imu->gyro_x_dps = ((float)BSP_JY61P_ReadInt16(&frame[2]) / 32768.0f) * 2000.0f;
            imu->gyro_y_dps = ((float)BSP_JY61P_ReadInt16(&frame[4]) / 32768.0f) * 2000.0f;
            imu->gyro_z_dps = ((float)BSP_JY61P_ReadInt16(&frame[6]) / 32768.0f) * 2000.0f;
            break;

        case BSP_JY61P_TYPE_ANGLE:
            roll_deg = BSP_JY61P_NormalizeAngle180(
                ((float)BSP_JY61P_ReadInt16(&frame[2]) / 32768.0f) * 180.0f);
            pitch_deg = BSP_JY61P_NormalizeAngle180(
                ((float)BSP_JY61P_ReadInt16(&frame[4]) / 32768.0f) * 180.0f);
            yaw_deg = BSP_JY61P_NormalizeAngle180(
                ((float)BSP_JY61P_ReadInt16(&frame[6]) / 32768.0f) * 180.0f);

            imu->roll_rad = roll_deg * BSP_JY61P_DEG_TO_RAD;
            imu->pitch_rad = pitch_deg * BSP_JY61P_DEG_TO_RAD;
            imu->yaw_rad = yaw_deg * BSP_JY61P_DEG_TO_RAD;
            imu->last_frame_ms = HAL_GetTick();
            imu->sample_seq++;
            imu->online = 1U;
            imu->data_valid = 1U;
            break;

        default:
            break;
    }
}

static uint8_t BSP_JY61P_IsFrameType(uint8_t type)
{
    return (uint8_t)((type >= 0x50U) && (type <= 0x5FU));
}

static uint8_t BSP_JY61P_Checksum(const uint8_t *frame)
{
    uint16_t sum = 0U;
    uint8_t i;

    if (frame == (const uint8_t *)0)
    {
        return 0U;
    }

    for (i = 0U; i < (BSP_JY61P_FRAME_SIZE - 1U); i++)
    {
        sum = (uint16_t)(sum + frame[i]);
    }

    return (uint8_t)(sum & 0x00FFU);
}

static int16_t BSP_JY61P_ReadInt16(const uint8_t *buf)
{
    uint16_t value;

    if (buf == (const uint8_t *)0)
    {
        return 0;
    }

    value = (uint16_t)(((uint16_t)buf[1] << 8U) | (uint16_t)buf[0]);
    return (int16_t)value;
}

static float BSP_JY61P_NormalizeAngle180(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg <= -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}
