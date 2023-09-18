/*
 * motor.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */


// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "adc.h"
// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "types.h"

#include "analog_sensors.h"
// ------------------------------------------------------------------- //


// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //
#define ADC_CONV_TIMEOUT_MS 	( (uint32_t) 10 )
#define ADC_DIG_TO_ANALOG_CONV	( (float) (3.3f/(4096-1)) )
// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
extern ADC_HandleTypeDef hadc1;
// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void SJC_AnalogSensorsInit(void)
{

}

float SJC_AnalogSensors_GetConversion(const eSjcAnalogSensor_t sensor)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	switch (sensor) {
	case SJC_CURRENT_SENSOR_1:
		sConfig.Channel = ADC_CHANNEL_8;
		break;
	case SJC_CURRENT_SENSOR_2:
		sConfig.Channel = ADC_CHANNEL_14;
		break;
	default:
		sConfig.Channel = ADC_CHANNEL_15;
		break;
	}
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, ADC_CONV_TIMEOUT_MS);
	HAL_ADC_Stop(&hadc1);

	float conv = ( (float) ( HAL_ADC_GetValue(&hadc1)*ADC_DIG_TO_ANALOG_CONV ) );

	return conv;

}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
