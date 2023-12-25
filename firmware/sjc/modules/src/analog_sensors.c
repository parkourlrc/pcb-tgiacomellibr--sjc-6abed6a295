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
#include "analog_sensors.h"

#include "configs.h"
// ------------------------------------------------------------------- //


// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
//extern ADC_HandleTypeDef hadc1;
// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
static uint32_t xAnalogSensorsInitiated = 0;
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t SJC_AnalogSensors_Init(xSjcAnalogSensor_t *Sensor)
{
	Sensor->Measurement = 0.0f;
	Sensor->Range.Min = ADC_CONV_MIN_VALUE_FLOAT;
	Sensor->Range.Max = ADC_CONV_MAX_VALUE_FLOAT;
	Sensor->Resolution = ADC_CONV_RESOLUTION;

	xAnalogSensorsInitiated++;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_AnalogSensors_Configure(uint32_t * buffer, const uint32_t buffer_size)
{
	if (true == LL_ADC_IsEnabled(SJC_ADC_INSTANCE))
	{
		return SJC_RET_NOK;
	}

	MX_ADC1_Init();

	/* Set DMA transfer addresses of source and destination */
	LL_DMA_ConfigAddresses(SJC_ADC_DMA_INSTANCE,
			SJC_ADC_DMA_STREAM, LL_ADC_DMA_GetRegAddr(SJC_ADC_INSTANCE,
					LL_ADC_DMA_REG_REGULAR_DATA), (uint32_t) &buffer,
					LL_DMA_DIRECTION_PERIPH_TO_MEMORY);


	/* Set DMA transfer size */
	LL_DMA_SetDataLength(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM, buffer_size);

	/* Enable DMA transfer interruption: transfer complete */
	LL_DMA_EnableIT_TC(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM);

	//	  /* Enable DMA transfer interruption: half transfer */
	//	  LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
	//
	//	  /* Enable DMA transfer interruption: transfer error */
	//	  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);

	/*## Activation of DMA #####################################################*/
	/* Enable the DMA transfer */
	LL_DMA_EnableStream(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_AnalogSensors_StartMeasurements(void)
{
//	LL_TIM_EnableCounter(TIM4);
//
//	LL_TIM_EnableIT_UPDATE(ADC_CONV_DISPATCH_TIMER_INSTANCE);

//	HAL_ADC_Start_DMA(&hadc1, buffer, xAnalogSensorsInitiated);

	LL_ADC_DisableIT_EOCS(SJC_ADC_INSTANCE);

	LL_ADC_EnableIT_JEOS(SJC_ADC_INSTANCE);


	return SJC_RET_OK;
}

eSjcReturnType_t SJC_AnalogSensors_StopMeasurements(void)
{
//	LL_TIM_DisableIT_UPDATE(ADC_CONV_DISPATCH_TIMER_INSTANCE);
//
//	LL_TIM_DisableCounter(TIM4);

//	HAL_ADC_Stop_DMA(&hadc1);

	LL_ADC_DisableIT_JEOS(SJC_ADC_INSTANCE);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_AnalogSensors_ComputeFilteredValue( const eSjcAnalogSensor_t input, const float previous, const float actual, float * value )
{
	eSjcReturnType_t ret = SJC_RET_OK;

	*value = 0;

	switch (input) {
	case SJC_CURRENT_SENSOR_STD:
		*value = (1-CURRENT_SENSOR_COMP_FILTER_RENEW_RATIO)*previous;
		*value += CURRENT_SENSOR_COMP_FILTER_RENEW_RATIO*actual;

		break;
	case SJC_VOLTAGE_SENSOR:
		*value = (1-VOLTAGE_SENSOR_COMP_FILTER_RENEW_RATIO)*previous;
		*value += VOLTAGE_SENSOR_COMP_FILTER_RENEW_RATIO*actual;
		break;

	default:
		ret = SJC_RET_NOK;
		break;
	}

	return ret;
}


eSjcReturnType_t SJC_AnalogSensors_ComputeConversionValue( const eSjcAnalogSensor_t input, const uint16_t conversion, float * value )
{
	eSjcReturnType_t ret = SJC_RET_OK;

	float conv = ADC_CONV_TO_FLOAT(conversion);
	*value = 0;

	switch (input) {
	case SJC_CURRENT_SENSOR_STD:
		*value = (conv/AD8418_ANALOG_FRONTED_GAIN)-AD8418_VREF;
		*value /= (AD8418_SHUNT_RESISTANCE*AD8418_GAIN);

		break;
	case SJC_VOLTAGE_SENSOR:
		*value = conversion*VOLTAGE_SENSOR_RESISTIVE_CONDITIONER_GAIN;
		break;

	default:
		ret = SJC_RET_NOK;
		break;
	}

	return ret;
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
