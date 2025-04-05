/*
 * types.h
 *
 *  Created on: Sep 17, 2023
 *      Author: tiago
 */

#ifndef SJC__BASE__CONFIGS_H_
#define SJC__BASE__CONFIGS_H_

// HAL Includes ------------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#include "adc.h"
#include "tim.h"

// Hardware info ------------------------------------------------------------------ //
// -------------------------------------------------------------------------------- //
#define SJC_HW_VERSION_MAJOR 1U
#define SJC_HW_VERSION_MINOR 0U

// Firmware info ------------------------------------------------------------------ //
// -------------------------------------------------------------------------------- //
#define SJC_FW_VERSION_MAJOR 1U
#define SJC_FW_VERSION_MINOR 0U
#define SJC_FW_VERSION_PATCH 0U

#define SJC_FW_RELEASE_MONTH 04U
#define SJC_FW_RELEASE_YEAR 2024UL

// Current sense mode ------------------------------------------------------------ //
// #define SJC_CURRENT_SENSE_CONFIG_EXTERNAL
// #define SJC_CURRENT_SENSE_CONFIG_INTERNAL_16A
// #define SJC_CURRENT_SENSE_CONFIG_INTERNAL_25A

// PWM Configs -------------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define SJC_PWM_COUNT_PERIOD ((uint32_t)1000)
#define SJC_PWM_MIN_DUTY_CYCLE ((float)0.02)
#define SJC_PWM_MAX_DUTY_CYCLE ((float)0.98)

#define SJC_PWM_INSTANCE TIM1
#define SJC_PWM_LEFT_CHANNEL LL_TIM_CHANNEL_CH1
#define SJC_PWM_LEFT_COMPLEMENTARY_CHANNEL LL_TIM_CHANNEL_CH1N

#define SJC_PWM_RIGHT_CHANNEL LL_TIM_CHANNEL_CH3
#define SJC_PWM_RIGHT_COMPLEMENTARY_CHANNEL LL_TIM_CHANNEL_CH3N

#define SJC_PWM_DEFAULT_FREQUENCY ((uint32_t)20000UL)
#define SJC_PWM_DEFAULT_PERIOD ((float)(1.0f / SJC_PWM_DEFAULT_FREQUENCY))

// Runtime Configs ---------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define SJC_TASK_INIT_DELAY ((uint32_t)2000UL)
#define SJC_DEBUG_TASK_UPDATE_PERIOD_MS ((uint32_t)2000UL)
#define SJC_DATA_LOGGER_TASK_UPDATE_PERIOD_MS ((uint32_t)1UL)
#define SJC_CONTROL_TASK_UPDATE_PERIOD_MS ((uint32_t)1UL)
#define SJC_NETWORK_TASK_UPDATE_PERIOD_MS ((uint32_t)3UL)
#define SJC_HEARTBEAT_CALLBACK_PERIOD_MS ((uint32_t)250UL)
#define SJC_HEARTBEAT_CALIBRATION_PERIOD_MS ((uint32_t)50UL)

// Controller Configs ------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define SJC_CONTROLLER_MIN_UPDATE_RATE_HZ ((float)1.0f)
#define SJC_CONTROLLER_MAX_UPDATE_RATE_HZ ((float)1000.0f)

#define SJC_CONTROLLER_MIN_UPDATE_PERIOD_MS ((uint32_t)1UL)
#define SJC_CONTROLLER_MAX_UPDATE_PERIOD_MS ((uint32_t)1000UL)

#define SJC_CONTROLLER_TIMER_INSTANCE TIM4

// Actuator Configs --------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#if SJC_HW_VERSION_MAJOR > 1
#define SJC_ACTUATOR_DRIVER_ENABLE_PIN V_MOTOR_DRIVER_nEN_Pin
#define SJC_ACTUATOR_DRIVER_ENABLE_PORT V_MOTOR_DRIVER_nEN_GPIO_Port
#endif

#define SJC_ACTUATOR_POWER_OUTPUT_ENABLE_PORT V_MOTOR_nEN_GPIO_Port
#define SJC_ACTUATOR_POWER_OUTPUT_ENABLE_PIN V_MOTOR_nEN_Pin

#define SJC_ACTUATOR_LIMIT_MAX_SPEED ((float)10000.0f)
#define SJC_ACTUATOR_LIMIT_MAX_CURRENT ((float)50.0f)
#define SJC_ACTUATOR_LIMIT_MAX_VOLTAGE ((float)36.0f)

// Encoder Configs ---------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define SJC_MIN_ENCODER_RESOLUTION_PPR ((uint16_t)1UL)
#define GET_ENCODER_QUADRATURE_RESOLUTION_PPR(X) ((uint32_t)(4 * X))

#define SJC_ACTUATOR_JOINT_TIMER_INSTANCE TIM2
#define SJC_AUX_JOINT_A_TIMER_INSTANCE TIM3
#define SJC_AUX_JOINT_B_TIMER_INSTANCE TIM8

// Digital Sensors ---------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define SJC_JOINT_SWITCH_A_PIN JOINT_SW_L_Pin
#define SJC_JOINT_SWITCH_B_PIN JOINT_SW_R_Pin
#define SJC_ISOLATED_INPUT_A_PIN GP_ISO_IN_A_Pin
#define SJC_ISOLATED_INPUT_B_PIN GP_ISO_IN_B_Pin

#define SJC_JOINT_SWITCH_A_PORT JOINT_SW_L_GPIO_Port
#define SJC_JOINT_SWITCH_B_PORT JOINT_SW_R_GPIO_Port
#define SJC_ISOLATED_INPUT_A_PORT GP_ISO_IN_A_GPIO_Port
#define SJC_ISOLATED_INPUT_B_PORT GP_ISO_IN_B_GPIO_Port

#if SJC_HW_VERSION_MAJOR > 1
#define SJC_NET_ADDR_A_INPUT_PIN NET_ADDR_A_Pin
#define SJC_NET_ADDR_B_INPUT_PIN NET_ADDR_B_Pin
#define SJC_NET_ADDR_C_INPUT_PIN NET_ADDR_C_Pin

#define SJC_NET_ADDR_A_INPUT_PORT NET_ADDR_A_GPIO_Port
#define SJC_NET_ADDR_B_INPUT_PORT NET_ADDR_B_GPIO_Port
#define SJC_NET_ADDR_C_INPUT_PORT NET_ADDR_C_GPIO_Port
#endif


// Analog Sensors ----------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define SJC_ADC_INSTANCE (ADC1)
#define SJC_ADC_DMA_INSTANCE DMA2
#define SJC_ADC_DMA_STREAM LL_DMA_STREAM_0
#define SJC_ADC_CHECK_TIMER_INSTANCE TIM4
#define SJC_ADC_CHECK_TIMER_RESOLUTION ((float)1.0e-6)
#define SJC_ADC_CHECK_TIMER_COUNT_PERIOD ((uint32_t)1000)
#define SJC_ADC_READING_ERROR ((float)-1234.5f)

#define ADC_CONV_TIMEOUT_MS ((uint32_t)10)
#define ADC_CONV_RESOLUTION ((uint32_t)4095)
#define ADC_CONV_MAX_VALUE_FLOAT ((float)(3.3f))
#define ADC_CONV_MIN_VALUE_FLOAT ((float)(0.0f))
#define ADC_CONV_TO_FLOAT(X) ((float)(X * ADC_CONV_MAX_VALUE_FLOAT) / ADC_CONV_RESOLUTION)

// Current Sensor ----------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define ANALOG_CURRENT_SENSOR_VREF ((float)2.5f)
#define ACS712_GAIN ((float)0.1f)
#define ACS712_ANALOG_FRONTED_GAIN ((float)0.6667f)
#define AD8418_GAIN ((float)20.0f)
#define AD8418_SHUNT_RESISTANCE ((float)0.008f)
#define AD8418_ANALOG_FRONTED_GAIN ((float)0.7907f)

#define CURRENT_SENSOR_COMP_FILTER_RENEW_RATIO ((float)0.7f)
#define CURRENT_SENSOR_COMP_FILTER_MAINT_RATIO (1.0f - CURRENT_SENSOR_COMP_FILTER_RENEW_RATIO)

// Voltage Sensor ----------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define VOLTAGE_SENSOR_RESISTIVE_CONDITIONER_GAIN ((float)11.0f)
#define VOLTAGE_SENSOR_COMP_FILTER_RENEW_RATIO ((float)0.8f)
#define VOLTAGE_SENSOR_COMP_FILTER_MAINT_RATIO (1.0f - VOLTAGE_SENSOR_COMP_FILTER_RENEW_RATIO)

// VrefInt Sensor ----------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define VREF_INT_SENSOR_COMP_FILTER_RENEW_RATIO ((float)0.8f)
#define VREF_INT_SENSOR_COMP_FILTER_MAINT_RATIO (1.0f - VREF_INT_SENSOR_COMP_FILTER_RENEW_RATIO)

// Digital Actuators -------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define SJC_DIGITAL_OUTPUT_A_PIN GP_OUT_A_Pin
#define SJC_DIGITAL_OUTPUT_B_PIN GP_OUT_B_Pin

#define SJC_DIGITAL_OUTPUT_A_PORT GP_OUT_A_GPIO_Port
#define SJC_DIGITAL_OUTPUT_B_PORT GP_OUT_B_GPIO_Port

#define SJC_HEARTBEAT_OUTPUT_PIN SYS_HEARTBEAT_Pin
#define SJC_HEARTBEAT_OUTPUT_PORT SYS_HEARTBEAT_GPIO_Port

// Modbus Communication ----------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define MODBUS_SERIAL_INSTANCE USART2
#define MODBUS_SERIAL_DMA_INSTANCE DMA1
#define MODBUS_SERIAL_DMA_STREAM LL_DMA_STREAM_6
#define MODBUS_SET_RX_MODE() LL_GPIO_ResetOutputPin(RS485_DE_GPIO_Port, RS485_DE_Pin)
#define MODBUS_SET_TX_MODE() LL_GPIO_SetOutputPin(RS485_DE_GPIO_Port, RS485_DE_Pin)

// System Configs ----------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define SJC_SYSTEM_RESET HAL_NVIC_SystemReset
#define SJC_HARDWARE_ACCESS_WAIT_TIME_MS ((uint32_t)2UL)

// Debug Configs ------------------------------------------------------------------ //
// -------------------------------------------------------------------------------- //
#define SJC_DEBUG_USB_PRINTF_ENABLE
#define SJC_DEBUG_SERIAL_PRINTF_ENABLE

#ifdef SJC_DEBUG_USB_PRINTF_ENABLE
// #define SJC_DEBUG_PRINT_NET_CMDS
#define SJC_DEBUG_PRINT_NET_REGS
#endif
// -------------------------------------------------------------------------------- //
#endif /* SJC__BASE__CONFIGS_H_ */
