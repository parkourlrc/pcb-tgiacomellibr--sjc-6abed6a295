/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GP_OUT_B_Pin GPIO_PIN_14
#define GP_OUT_B_GPIO_Port GPIOC
#define GP_OUT_A_Pin GPIO_PIN_15
#define GP_OUT_A_GPIO_Port GPIOC
#define GP_ISO_IN_A_Pin GPIO_PIN_0
#define GP_ISO_IN_A_GPIO_Port GPIOC
#define GP_ISO_IN_B_Pin GPIO_PIN_1
#define GP_ISO_IN_B_GPIO_Port GPIOC
#define JOINT_SW_L_Pin GPIO_PIN_2
#define JOINT_SW_L_GPIO_Port GPIOC
#define JOINT_SW_R_Pin GPIO_PIN_3
#define JOINT_SW_R_GPIO_Port GPIOC
#define JOINT_ENC_CH1_Pin GPIO_PIN_0
#define JOINT_ENC_CH1_GPIO_Port GPIOA
#define JOINT_ENC_CH2_Pin GPIO_PIN_1
#define JOINT_ENC_CH2_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_2
#define RS485_TX_GPIO_Port GPIOA
#define RS485_RX_Pin GPIO_PIN_3
#define RS485_RX_GPIO_Port GPIOA
#define RS485_DE_Pin GPIO_PIN_4
#define RS485_DE_GPIO_Port GPIOA
#define AUX1_ENC_CH1_Pin GPIO_PIN_6
#define AUX1_ENC_CH1_GPIO_Port GPIOA
#define AUX1_ENC_CH2_Pin GPIO_PIN_7
#define AUX1_ENC_CH2_GPIO_Port GPIOA
#define CURRENT_SIGNAL_FILT2_Pin GPIO_PIN_4
#define CURRENT_SIGNAL_FILT2_GPIO_Port GPIOC
#define MOTOR_INPUT_VOLTAGE_SIG_Pin GPIO_PIN_5
#define MOTOR_INPUT_VOLTAGE_SIG_GPIO_Port GPIOC
#define CURRENT_SIGNAL_FILT_Pin GPIO_PIN_0
#define CURRENT_SIGNAL_FILT_GPIO_Port GPIOB
#define R_MOT_LO_Pin GPIO_PIN_1
#define R_MOT_LO_GPIO_Port GPIOB
#define L_MOT_LO_Pin GPIO_PIN_13
#define L_MOT_LO_GPIO_Port GPIOB
#define AUX2_ENC_CH1_Pin GPIO_PIN_6
#define AUX2_ENC_CH1_GPIO_Port GPIOC
#define AUX2_ENC_CH2_Pin GPIO_PIN_7
#define AUX2_ENC_CH2_GPIO_Port GPIOC
#define L_MOT_HI_Pin GPIO_PIN_8
#define L_MOT_HI_GPIO_Port GPIOA
#define USB_OTG_FS_VBUS_SIG_Pin GPIO_PIN_9
#define USB_OTG_FS_VBUS_SIG_GPIO_Port GPIOA
#define R_MOT_HI_Pin GPIO_PIN_10
#define R_MOT_HI_GPIO_Port GPIOA
#define DEBUG_USART_TX_Pin GPIO_PIN_10
#define DEBUG_USART_TX_GPIO_Port GPIOC
#define DEBUG_USART_RX_Pin GPIO_PIN_11
#define DEBUG_USART_RX_GPIO_Port GPIOC
#define SYS_HEARTBEAT_Pin GPIO_PIN_12
#define SYS_HEARTBEAT_GPIO_Port GPIOC
#define USB_OTG_FS_PU_Pin GPIO_PIN_2
#define USB_OTG_FS_PU_GPIO_Port GPIOD
#define V_MOTOR_nEN_Pin GPIO_PIN_5
#define V_MOTOR_nEN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
