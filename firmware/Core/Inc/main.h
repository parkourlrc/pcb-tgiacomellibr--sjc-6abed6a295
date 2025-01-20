/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define JOINT_SW_R_Pin GPIO_PIN_0
#define JOINT_SW_R_GPIO_Port GPIOC
#define JOINT_SW_L_Pin GPIO_PIN_1
#define JOINT_SW_L_GPIO_Port GPIOC
#define JOINT_ENC_CH1_Pin GPIO_PIN_0
#define JOINT_ENC_CH1_GPIO_Port GPIOA
#define JOINT_ENC_CH2_Pin GPIO_PIN_1
#define JOINT_ENC_CH2_GPIO_Port GPIOA
#define AUX1_ENC_CH2_Pin GPIO_PIN_6
#define AUX1_ENC_CH2_GPIO_Port GPIOA
#define AUX1_ENC_CH1_Pin GPIO_PIN_7
#define AUX1_ENC_CH1_GPIO_Port GPIOA
#define R_MOT_LO_Pin GPIO_PIN_0
#define R_MOT_LO_GPIO_Port GPIOB
#define L_MOT_LO_Pin GPIO_PIN_8
#define L_MOT_LO_GPIO_Port GPIOE
#define L_MOT_HI_Pin GPIO_PIN_9
#define L_MOT_HI_GPIO_Port GPIOE
#define MotorDriver_EN_Pin GPIO_PIN_10
#define MotorDriver_EN_GPIO_Port GPIOE
#define R_MOT_HI_Pin GPIO_PIN_11
#define R_MOT_HI_GPIO_Port GPIOE
#define MotorDriverPSU_EN_Pin GPIO_PIN_12
#define MotorDriverPSU_EN_GPIO_Port GPIOE
#define MotorPSU_EN_Pin GPIO_PIN_13
#define MotorPSU_EN_GPIO_Port GPIOE
#define COM_485_TX_Pin GPIO_PIN_10
#define COM_485_TX_GPIO_Port GPIOB
#define COM_485_RX_Pin GPIO_PIN_11
#define COM_485_RX_GPIO_Port GPIOB
#define AUX_USART_RX_Pin GPIO_PIN_12
#define AUX_USART_RX_GPIO_Port GPIOB
#define AUX_USART_TX_Pin GPIO_PIN_13
#define AUX_USART_TX_GPIO_Port GPIOB
#define COM_485_DE_Pin GPIO_PIN_14
#define COM_485_DE_GPIO_Port GPIOB
#define COM_485_ACT_Pin GPIO_PIN_15
#define COM_485_ACT_GPIO_Port GPIOB
#define AUX2_ENC_CH1_Pin GPIO_PIN_12
#define AUX2_ENC_CH1_GPIO_Port GPIOD
#define AUX2_ENC_CH2_Pin GPIO_PIN_13
#define AUX2_ENC_CH2_GPIO_Port GPIOD
#define FDCAN_ACT_Pin GPIO_PIN_10
#define FDCAN_ACT_GPIO_Port GPIOA
#define DEBUG_USART_TX_Pin GPIO_PIN_10
#define DEBUG_USART_TX_GPIO_Port GPIOC
#define DEBUG_USART_RX_Pin GPIO_PIN_11
#define DEBUG_USART_RX_GPIO_Port GPIOC
#define GP_OUT_A_Pin GPIO_PIN_0
#define GP_OUT_A_GPIO_Port GPIOE
#define GP_OUT_B_Pin GPIO_PIN_1
#define GP_OUT_B_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
