/*
 * analog_interfaces.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */


// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "adc.h"
#include "tim.h"
#include "main.h"

// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "analog_interfaces.h"
#include "configs.h"
#include "debug_output.h"
// ------------------------------------------------------------------- //


// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //
#define VDDA_APPLI                       				((uint32_t) 3300UL)
#define ADC_CONVERTED_DATA_BUFFER_SIZE   				((uint32_t) 3UL)
// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
typedef enum {
	MOTOR_CURRENT_CONV = 0,
	MOTOR_VOLTAGE_CONV
//	VREF_INT_CONV
} eSjcConversionOrder_t;


// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
/* ADC group regular conversion data */
static float xAnalogSensorsValueBuffer[ADC_CONVERTED_DATA_BUFFER_SIZE];
static uint16_t xAnalogSensorConvBuffer[ADC_CONVERTED_DATA_BUFFER_SIZE];
static uint32_t xAnalogSensorsInitiated = 0;
static uint32_t xMeasurementCounter = 0;

xSjcSampleTimeMeasurement_t xSampleTimeObj = xSjcSampleTimeEmptyObj;

extern SemaphoreHandle_t xMeasurementSemaphoreHandle;
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t SJC_AnalogInterfaces_Init(xSjcAnalogSensor_t *Sensor)
{
	/* ADC Peripheral Configuration ---------------------------------- */
	if (0Ul == xAnalogSensorsInitiated)
	{
		MX_ADC1_Init();
		SJC_ADC_CHECK_INIT();
		LL_TIM_EnableCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
	}

	Sensor->Measurement = 0.0f;
	Sensor->Range.Min = ADC_CONV_MIN_VALUE_FLOAT;
	Sensor->Range.Max = ADC_CONV_MAX_VALUE_FLOAT;
	Sensor->Resolution = ADC_CONV_RESOLUTION;
	xAnalogSensorsInitiated++;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_AnalogInterfaces_Enable(void)
{
	LL_ADC_EnableIT_OVR(SJC_ADC_INSTANCE);
	LL_ADC_EnableIT_JEOS(SJC_ADC_INSTANCE);

	LL_ADC_Enable(SJC_ADC_INSTANCE);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_AnalogInterfaces_Disable(void)
{
	LL_ADC_Disable(SJC_ADC_INSTANCE);

	LL_ADC_EnableIT_OVR(SJC_ADC_INSTANCE);
	LL_ADC_EnableIT_JEOS(SJC_ADC_INSTANCE);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_AnalogInterfaces_GetAnalogConversion(const eSjcAnalogSensorType_t sensor, float * value)
{
	eSjcReturnType_t ret = SJC_RET_OK;
	float conv;

	switch (sensor) {
		case SJC_CURRENT_SENSOR_STD:
			conv = xAnalogSensorsValueBuffer[MOTOR_CURRENT_CONV];
			*value = (conv/AD8418_ANALOG_FRONTED_GAIN)-AD8418_VREF;
			*value /= (AD8418_SHUNT_RESISTANCE*AD8418_GAIN);

			break;
		case SJC_VOLTAGE_SENSOR:
			conv = xAnalogSensorsValueBuffer[MOTOR_VOLTAGE_CONV];
			*value = conv*VOLTAGE_SENSOR_RESISTIVE_CONDITIONER_GAIN;
			break;

		default:
			ret = SJC_RET_NOK;
			break;
		}
	return ret;
}

eSjcReturnType_t SJC_AnalogInterfaces_GetMeanSampleTime(xSjcSampleTimeMeasurement_t * Ts)
{
	*Ts = xSampleTimeObj;
	xSampleTimeObj = xSjcSampleTimeEmptyObj;

	return SJC_RET_OK;
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void ComputeAdcMeasurements(void)
{
	xSampleTimeObj.actual_tick = LL_TIM_GetCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
	uint32_t dt;

	if (xSampleTimeObj.actual_tick > xSampleTimeObj.last_tick )
	{
		dt = xSampleTimeObj.actual_tick - xSampleTimeObj.last_tick;
	}
	else
	{
		dt = xSampleTimeObj.actual_tick + SJC_ADC_CHECK_TIMER_COUNT_PERIOD - xSampleTimeObj.last_tick;
	}

	xSampleTimeObj.samples++;
	xSampleTimeObj.last_tick = xSampleTimeObj.actual_tick;
	xSampleTimeObj.timesteps += ( SJC_ADC_CHECK_TIMER_RESOLUTION*dt);

//	xAnalogSensorsValueBuffer[VREF_INT_CONV] *= VREF_INT_SENSOR_COMP_FILTER_MAINT_RATIO;
	xAnalogSensorsValueBuffer[MOTOR_CURRENT_CONV] *= CURRENT_SENSOR_COMP_FILTER_MAINT_RATIO;
	xAnalogSensorsValueBuffer[MOTOR_VOLTAGE_CONV] *= VOLTAGE_SENSOR_COMP_FILTER_MAINT_RATIO;

//	xAnalogSensorConvBuffer[VREF_INT_CONV] = LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_1);
	xAnalogSensorConvBuffer[MOTOR_CURRENT_CONV] = LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_2);
	xAnalogSensorConvBuffer[MOTOR_VOLTAGE_CONV] = LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_3);

//	xAnalogSensorsValueBuffer[VREF_INT_CONV] += (__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, xAnalogSensorConvBuffer[VREF_INT_CONV], LL_ADC_RESOLUTION_12B)*VREF_INT_SENSOR_COMP_FILTER_RENEW_RATIO/1000.0f);
	xAnalogSensorsValueBuffer[MOTOR_CURRENT_CONV] += (__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, xAnalogSensorConvBuffer[MOTOR_CURRENT_CONV], LL_ADC_RESOLUTION_12B)*CURRENT_SENSOR_COMP_FILTER_RENEW_RATIO/1000.0f);
	xAnalogSensorsValueBuffer[MOTOR_VOLTAGE_CONV] += (__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, xAnalogSensorConvBuffer[MOTOR_VOLTAGE_CONV], LL_ADC_RESOLUTION_12B)*VREF_INT_SENSOR_COMP_FILTER_RENEW_RATIO/1000.0f);

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (++xMeasurementCounter >= SJC_ADC_MIN_SAMPLES_BEFORE_CONTROL_ACTION)
	{
		if (xMeasurementSemaphoreHandle != NULL)
		{
		    xHigherPriorityTaskWoken = pdFALSE;

	        xSemaphoreGiveFromISR( xMeasurementSemaphoreHandle, &xHigherPriorityTaskWoken );

	        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}
	}
}

void ADC_IRQHandler(void)
{
	/* Check whether ADC group injected end of unitary conversion caused        */
	/* the ADC interruption.                                                    */
	/* Note: On this STM32 series, there is no flag of group injected            */
	/*       end of unitary conversion. Therefore, flag of group injected       */
	/*       end of sequence conversions is used (equivalent when there is      */
	/*       only 1 rank in group injected sequencer).                          */
	if(LL_ADC_IsActiveFlag_JEOS(SJC_ADC_INSTANCE) != 0)
	{
		/* Clear flag ADC group injected end of sequence conversions */
		LL_ADC_ClearFlag_JEOS(SJC_ADC_INSTANCE);

		/* Call interruption treatment function */
		ComputeAdcMeasurements();
	}

	/* Check whether ADC group regular overrun caused the ADC interruption */
	if(LL_ADC_IsActiveFlag_OVR(SJC_ADC_INSTANCE) != 0)
	{
		/* Clear flag ADC group regular overrun */
		LL_ADC_ClearFlag_OVR(SJC_ADC_INSTANCE);

		/* Call interruption treatment function */
		SJC_DebugOutput_printf("\r\n[FAULT] ADC Overrun!\n");
		Error_Handler();
	}
}
// ------------------------------------------------------------------- //
