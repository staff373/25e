/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BIN2_Pin GPIO_PIN_2
#define BIN2_GPIO_Port GPIOE
#define BIN1_Pin GPIO_PIN_3
#define BIN1_GPIO_Port GPIOE
#define AIN1_Pin GPIO_PIN_4
#define AIN1_GPIO_Port GPIOE
#define AIN2_Pin GPIO_PIN_5
#define AIN2_GPIO_Port GPIOE
#define X_PUL_Pin GPIO_PIN_6
#define X_PUL_GPIO_Port GPIOE
#define Y_DIR_Pin GPIO_PIN_13
#define Y_DIR_GPIO_Port GPIOC
#define L2_Pin GPIO_PIN_0
#define L2_GPIO_Port GPIOC
#define L1_Pin GPIO_PIN_1
#define L1_GPIO_Port GPIOC
#define M_Pin GPIO_PIN_2
#define M_GPIO_Port GPIOC
#define R1_Pin GPIO_PIN_3
#define R1_GPIO_Port GPIOC
#define ENC_LB_A_Pin GPIO_PIN_0
#define ENC_LB_A_GPIO_Port GPIOA
#define ENC_LB_B_Pin GPIO_PIN_1
#define ENC_LB_B_GPIO_Port GPIOA
#define TB_FRONT_PWMA_Pin GPIO_PIN_2
#define TB_FRONT_PWMA_GPIO_Port GPIOA
#define TB_FRONT_PWMB_Pin GPIO_PIN_3
#define TB_FRONT_PWMB_GPIO_Port GPIOA
#define ENC_LF_A_Pin GPIO_PIN_6
#define ENC_LF_A_GPIO_Port GPIOA
#define ENC_LF_B_Pin GPIO_PIN_7
#define ENC_LF_B_GPIO_Port GPIOA
#define TB2_AIN1_Pin GPIO_PIN_0
#define TB2_AIN1_GPIO_Port GPIOB
#define TB2_AIN2_Pin GPIO_PIN_1
#define TB2_AIN2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_9
#define LED3_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_10
#define KEY1_GPIO_Port GPIOE
#define TB_REAR_PWMA_Pin GPIO_PIN_13
#define TB_REAR_PWMA_GPIO_Port GPIOE
#define TB_REAR_PWMB_Pin GPIO_PIN_14
#define TB_REAR_PWMB_GPIO_Port GPIOE
#define X_DIR_Pin GPIO_PIN_15
#define X_DIR_GPIO_Port GPIOE
#define JY61P_TX_Pin GPIO_PIN_10
#define JY61P_TX_GPIO_Port GPIOB
#define JY61P_RX_Pin GPIO_PIN_11
#define JY61P_RX_GPIO_Port GPIOB
#define TB2_BIN1_Pin GPIO_PIN_12
#define TB2_BIN1_GPIO_Port GPIOB
#define X_EN_Pin GPIO_PIN_13
#define X_EN_GPIO_Port GPIOB
#define Y_PUL_Pin GPIO_PIN_14
#define Y_PUL_GPIO_Port GPIOB
#define TB2_BIN2_Pin GPIO_PIN_15
#define TB2_BIN2_GPIO_Port GPIOB
#define KEY2_Pin GPIO_PIN_10
#define KEY2_GPIO_Port GPIOD
#define KEY3_Pin GPIO_PIN_11
#define KEY3_GPIO_Port GPIOD
#define ENC_RF_A_Pin GPIO_PIN_12
#define ENC_RF_A_GPIO_Port GPIOD
#define ENC_RF_B_Pin GPIO_PIN_13
#define ENC_RF_B_GPIO_Port GPIOD
#define Y_EN_Pin GPIO_PIN_15
#define Y_EN_GPIO_Port GPIOD
#define ENC_RB_A_Pin GPIO_PIN_6
#define ENC_RB_A_GPIO_Port GPIOC
#define ENC_RB_B_Pin GPIO_PIN_7
#define ENC_RB_B_GPIO_Port GPIOC
#define BT_TX_Pin GPIO_PIN_12
#define BT_TX_GPIO_Port GPIOC
#define BT_RX_Pin GPIO_PIN_2
#define BT_RX_GPIO_Port GPIOD
#define VISION_TX_Pin GPIO_PIN_5
#define VISION_TX_GPIO_Port GPIOD
#define VISION_RX_Pin GPIO_PIN_6
#define VISION_RX_GPIO_Port GPIOD
#define R2_Pin GPIO_PIN_9
#define R2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
