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
#include "tim.h"

// Firmware info ------------------------------------------------------------------ //
// -------------------------------------------------------------------------------- //
#define SJC_FW_VERSION_MAJOR										1U
#define SJC_FW_VERSION_MINOR										0U
#define SJC_FW_VERSION_PATCH										0U

#define SJC_FW_RELEASE_MONTH										10U
#define SJC_FW_RELEASE_YEAR											2023UL

#define SJC_HW_VERSION_MAJOR										1U
#define SJC_HW_VERSION_MINOR										0U


// PWM Configs -------------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define SJC_PWM_COUNT_PERIOD 								( (uint32_t) 1000 	)
#define SJC_PWM_MIN_DUTY_CYCLE 								( (float) 	 0.02	)
#define SJC_PWM_MAX_DUTY_CYCLE 								( (float) 	 0.98	)

#define SJC_PWM_INSTANCE									TIM1
#define SJC_PWM_LEFT_CHANNEL								LL_TIM_CHANNEL_CH1
#define SJC_PWM_LEFT_COMPLEMENTARY_CHANNEL					LL_TIM_CHANNEL_CH1N

#define SJC_PWM_RIGHT_CHANNEL								LL_TIM_CHANNEL_CH3
#define SJC_PWM_RIGHT_COMPLEMENTARY_CHANNEL					LL_TIM_CHANNEL_CH3N

// Runtime Configs ---------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define SJC_DATA_LOGGER_UPDATE_PERIOD_MS					( (uint32_t) 1UL)
#define SJC_SYSTEM_CONTROLLER_UPDATE_PERIOD_MS				( (uint32_t) 5UL)

// Controller Configs ------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define SJC_CONTROLLER_MIN_UPDATE_RATE_HZ					( (float) 1.0f)
#define SJC_CONTROLLER_MAX_UPDATE_RATE_HZ					( (float) 1000.0f)

#define SJC_CONTROLLER_MIN_UPDATE_PERIOD_MS					( (uint32_t) 1UL)
#define SJC_CONTROLLER_MAX_UPDATE_PERIOD_MS					( (uint32_t) 1000UL)

// Actuator Configs --------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define SJC_ACTUATOR_DRIVER_ENABLE_PORT						V_MOTOR_nEN_GPIO_Port
#define SJC_ACTUATOR_DRIVER_ENABLE_PIN						V_MOTOR_nEN_Pin

#define SJC_ACTUATOR_LIMIT_MAX_SPEED						( (float) 100.0f)
#define SJC_ACTUATOR_LIMIT_MAX_CURRENT						( (float) 36.0f)
#define SJC_ACTUATOR_LIMIT_MAX_VOLTAGE						( (float) 100.0f)

// Encoder Configs ---------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define SJC_MIN_ENCODER_RESOLUTION_PPR						( (uint16_t) 1UL)
#define GET_ENCODER_QUADRATURE_RESOLUTION_PPR(X)			( (uint32_t) (4*X) )

#define SJC_ACTUATOR_JOINT_TIMER_INSTANCE					TIM2
#define SJC_AUX_JOINT_A_TIMER_INSTANCE						TIM3
#define SJC_AUX_JOINT_B_TIMER_INSTANCE						TIM8


// Digital Sensors ---------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define SJC_JOINT_SWITCH_A_PIN								JOINT_SW_L_Pin
#define SJC_JOINT_SWITCH_B_PIN								JOINT_SW_R_Pin
#define SJC_ISOLATED_INPUT_A_PIN							GP_ISO_IN_A_Pin
#define SJC_ISOLATED_INPUT_B_PIN							GP_ISO_IN_B_Pin

#define SJC_JOINT_SWITCH_A_PORT								JOINT_SW_L_GPIO_Port
#define SJC_JOINT_SWITCH_B_PORT								JOINT_SW_R_GPIO_Port
#define SJC_ISOLATED_INPUT_A_PORT							GP_ISO_IN_A_GPIO_Port
#define SJC_ISOLATED_INPUT_B_PORT							GP_ISO_IN_B_GPIO_Port

// Analog Sensors ----------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define SJC_ADC_INSTANCE 									( ADC1 )
#define SJC_ADC_DMA_INSTANCE								DMA2
#define SJC_ADC_DMA_STREAM									LL_DMA_STREAM_0
#define ADC_CONV_TIMEOUT_MS 								( (uint32_t) 10 )
#define ADC_CONV_RESOLUTION 								( (uint32_t) 4095 )
#define ADC_CONV_MAX_VALUE_FLOAT							( (float) (3.3f) )
#define ADC_CONV_MIN_VALUE_FLOAT							( (float) (0.0f) )
#define ADC_CONV_TO_FLOAT(X)								( (float) (X*ADC_CONV_MAX_VALUE_FLOAT)/ADC_CONV_RESOLUTION )
#define ADC_CONV_DISPATCH_TIMER_INSTANCE					TIM4

// ADC ISR Callback function
void AdcGrpInjectedUnitaryConvComplete_Callback(void);


// Current Sensor ----------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define AD8418_GAIN											( (float) 20.0f		)
#define AD8418_VREF											( (float) 2.5f		)
#define AD8418_SHUNT_RESISTANCE								( (float) 0.008f	)
#define AD8418_ANALOG_FRONTED_GAIN							( (float) 0.79f		)
#define CURRENT_SENSOR_COMP_FILTER_RENEW_RATIO				( (float) 0.75f		)

// Voltage Sensor ----------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define VOLTAGE_SENSOR_COMP_FILTER_RENEW_RATIO				( (float) 0.75f		)
#define VOLTAGE_SENSOR_RESISTIVE_CONDITIONER_GAIN			( (float) 11.0f		)

// Digital Actuators -------------------------------------------------------------- //
// -------------------------------------------------------------------------------- //
#define SJC_DIGITAL_OUTPUT_A_PIN							GP_OUT_A_Pin
#define SJC_DIGITAL_OUTPUT_B_PIN							GP_OUT_B_Pin

#define SJC_DIGITAL_OUTPUT_A_PORT							GP_OUT_A_GPIO_Port
#define SJC_DIGITAL_OUTPUT_B_PORT							GP_OUT_B_GPIO_Port

#define SJC_HEARTBEAT_OUTPUT_PIN							SYS_HEARTBEAT_Pin
#define SJC_HEARTBEAT_OUTPUT_PORT							SYS_HEARTBEAT_GPIO_Port
#endif /* SJC__BASE__CONFIGS_H_ */
