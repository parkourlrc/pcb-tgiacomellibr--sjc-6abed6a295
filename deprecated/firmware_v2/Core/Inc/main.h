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

#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

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
#define V_MOTOR_DRIVER_nEN_Pin LL_GPIO_PIN_13
#define V_MOTOR_DRIVER_nEN_GPIO_Port GPIOC
#define GP_OUT_B_Pin LL_GPIO_PIN_14
#define GP_OUT_B_GPIO_Port GPIOC
#define GP_OUT_A_Pin LL_GPIO_PIN_15
#define GP_OUT_A_GPIO_Port GPIOC
#define GP_ISO_IN_A_Pin LL_GPIO_PIN_0
#define GP_ISO_IN_A_GPIO_Port GPIOC
#define GP_ISO_IN_B_Pin LL_GPIO_PIN_1
#define GP_ISO_IN_B_GPIO_Port GPIOC
#define JOINT_SW_R_Pin LL_GPIO_PIN_2
#define JOINT_SW_R_GPIO_Port GPIOC
#define JOINT_SW_L_Pin LL_GPIO_PIN_3
#define JOINT_SW_L_GPIO_Port GPIOC
#define JOINT_ENC_CH1_Pin LL_GPIO_PIN_0
#define JOINT_ENC_CH1_GPIO_Port GPIOA
#define JOINT_ENC_CH2_Pin LL_GPIO_PIN_1
#define JOINT_ENC_CH2_GPIO_Port GPIOA
#define RS485_TX_Pin LL_GPIO_PIN_2
#define RS485_TX_GPIO_Port GPIOA
#define RS485_RX_Pin LL_GPIO_PIN_3
#define RS485_RX_GPIO_Port GPIOA
#define RS485_DE_Pin LL_GPIO_PIN_4
#define RS485_DE_GPIO_Port GPIOA
#define AUX1_ENC_CH1_Pin LL_GPIO_PIN_6
#define AUX1_ENC_CH1_GPIO_Port GPIOA
#define AUX1_ENC_CH2_Pin LL_GPIO_PIN_7
#define AUX1_ENC_CH2_GPIO_Port GPIOA
#define CURRENT_SIGNAL_FILT2_Pin LL_GPIO_PIN_4
#define CURRENT_SIGNAL_FILT2_GPIO_Port GPIOC
#define MOTOR_INPUT_VOLTAGE_SIG_Pin LL_GPIO_PIN_5
#define MOTOR_INPUT_VOLTAGE_SIG_GPIO_Port GPIOC
#define CURRENT_SIGNAL_FILT_Pin LL_GPIO_PIN_0
#define CURRENT_SIGNAL_FILT_GPIO_Port GPIOB
#define R_MOT_LO_Pin LL_GPIO_PIN_1
#define R_MOT_LO_GPIO_Port GPIOB
#define NET_ADDR_C_Pin LL_GPIO_PIN_2
#define NET_ADDR_C_GPIO_Port GPIOB
#define NET_ADDR_B_Pin LL_GPIO_PIN_10
#define NET_ADDR_B_GPIO_Port GPIOB
#define L_MOT_LO_Pin LL_GPIO_PIN_13
#define L_MOT_LO_GPIO_Port GPIOB
#define SYS_HEARTBEAT_Pin LL_GPIO_PIN_15
#define SYS_HEARTBEAT_GPIO_Port GPIOB
#define AUX2_ENC_CH1_Pin LL_GPIO_PIN_6
#define AUX2_ENC_CH1_GPIO_Port GPIOC
#define AUX2_ENC_CH2_Pin LL_GPIO_PIN_7
#define AUX2_ENC_CH2_GPIO_Port GPIOC
#define NET_ADDR_A_Pin LL_GPIO_PIN_8
#define NET_ADDR_A_GPIO_Port GPIOC
#define V_MOTOR_nEN_Pin LL_GPIO_PIN_9
#define V_MOTOR_nEN_GPIO_Port GPIOC
#define L_MOT_HI_Pin LL_GPIO_PIN_8
#define L_MOT_HI_GPIO_Port GPIOA
#define COM_USB_VBUS_SIG_Pin LL_GPIO_PIN_9
#define COM_USB_VBUS_SIG_GPIO_Port GPIOA
#define R_MOT_HI_Pin LL_GPIO_PIN_10
#define R_MOT_HI_GPIO_Port GPIOA
#define COM_USB_DM_Pin LL_GPIO_PIN_11
#define COM_USB_DM_GPIO_Port GPIOA
#define COM_USB_DP_Pin LL_GPIO_PIN_12
#define COM_USB_DP_GPIO_Port GPIOA
#define DEBUG_USART_TX_Pin LL_GPIO_PIN_10
#define DEBUG_USART_TX_GPIO_Port GPIOC
#define DEBUG_USART_RX_Pin LL_GPIO_PIN_11
#define DEBUG_USART_RX_GPIO_Port GPIOC
#define DEBUG_485_TX_Pin LL_GPIO_PIN_12
#define DEBUG_485_TX_GPIO_Port GPIOC
#define DEBUG_485_RX_Pin LL_GPIO_PIN_2
#define DEBUG_485_RX_GPIO_Port GPIOD
#define DEBUG_485_DE_Pin LL_GPIO_PIN_5
#define DEBUG_485_DE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
