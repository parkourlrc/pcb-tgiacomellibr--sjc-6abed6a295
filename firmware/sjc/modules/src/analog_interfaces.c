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

#include "cmsis_os.h"
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
#define ANALOG_INTERFACE_MUTEX_DEFAULT_WAIT_TIME_MS 	((uint32_t) 3UL)
#define ANALOG_INTERFACE_MUTEX_CONVERSION_WAIT_TIME_MS 	((uint32_t) osWaitForever)
#define CONV_MILLIVOLT_TO_VOLT(X)						(X/1000.0f)
// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
typedef enum {
	MOTOR_CURRENT_CONV = 0,
	MOTOR_VOLTAGE_CONV,
	VREF_INT_CONV,
	CONVERSION_ORDER_SIZE
} eSjcConversionOrder_t;
// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
/* ADC group regular conversion data */
static uint32_t xAdcConversionBuffer[CONVERSION_ORDER_SIZE];
static float xAdcMeasurementValueBuffer[CONVERSION_ORDER_SIZE];

static float xAdcMeasurements[CONVERSION_ORDER_SIZE];

static xSjcMeanSampleTimeMeasurement_t xMeanSampleTimeObj = xSjcSampleTimeEmptyObj;

static uint32_t xIsrServiceCounter = 0;


osSemaphoreId_t xAnalogInterfaceSemprHandle = NULL;
StaticSemaphore_t AnalogInterfaceSemaphoreControlBlock;
const osSemaphoreAttr_t AnalogInterfaceSemaphore_attributes = {
		.name = "AnalogInterfaceSemaphore",
		.cb_mem = &AnalogInterfaceSemaphoreControlBlock,
		.cb_size = sizeof(AnalogInterfaceSemaphoreControlBlock),
};
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //
void ComputeFilteredMeasurements(void);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t SJC_AnalogInterfaces_Init(void)
{
	/* ADC Peripheral Configuration ---------------------------------- */
	xAnalogInterfaceSemprHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &AnalogInterfaceSemaphore_attributes);

	if (xAnalogInterfaceSemprHandle == NULL)
	{
		SJC_DebugOutput_printf("\r\n[OS FAULT] Analog interface semaphore not created...");
		return SJC_RET_NOK;
	} else
	{
		osSemaphoreRelease(xAnalogInterfaceSemprHandle);
	}

	if (false == LL_TIM_IsEnabledCounter(SJC_ADC_CHECK_TIMER_INSTANCE))
	{
		LL_TIM_EnableCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
	}

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_AnalogInterfaces_ConfigureInterface(xSjcAnalogSensor_t *Sensor)
{

	Sensor->Measurement = 0.0f;
	Sensor->Range.Min = ADC_CONV_MIN_VALUE_FLOAT;
	Sensor->Range.Max = ADC_CONV_MAX_VALUE_FLOAT;
	Sensor->Resolution = ADC_CONV_RESOLUTION;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_AnalogInterfaces_MeasurementEnable(void)
{
	LL_ADC_EnableIT_OVR(SJC_ADC_INSTANCE);
	LL_ADC_EnableIT_JEOS(SJC_ADC_INSTANCE);

	LL_ADC_Enable(SJC_ADC_INSTANCE);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_AnalogInterfaces_MeasurementDisable(void)
{
	LL_ADC_Disable(SJC_ADC_INSTANCE);

	LL_ADC_EnableIT_OVR(SJC_ADC_INSTANCE);
	LL_ADC_EnableIT_JEOS(SJC_ADC_INSTANCE);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_AnalogInterfaces_UpdateConversionValues(void)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	configASSERT( xAnalogInterfaceSemprHandle != NULL );

	if (osOK == osSemaphoreAcquire(xAnalogInterfaceSemprHandle, ANALOG_INTERFACE_MUTEX_CONVERSION_WAIT_TIME_MS))
	{
		if (xIsrServiceCounter != 20)
		{
			SJC_DebugOutput_printf("\n\n\n\r[OS FAULT] ADC Measurement check timer wrong period!");
			for (;;);
		}
		xIsrServiceCounter = 0;
		xAdcMeasurements[MOTOR_CURRENT_CONV] = xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV];
		xAdcMeasurements[MOTOR_VOLTAGE_CONV] = xAdcMeasurementValueBuffer[MOTOR_VOLTAGE_CONV];
		xAdcMeasurements[VREF_INT_CONV] = xAdcMeasurementValueBuffer[VREF_INT_CONV];

		osSemaphoreRelease(xAnalogInterfaceSemprHandle);

		ret = SJC_RET_OK;
	}

	return ret;
}

eSjcReturnType_t SJC_AnalogInterfaces_GetAnalogConversion(const eSjcAnalogSensorType_t sensor, float * value)
{
	float conv;
	eSjcReturnType_t ret = SJC_RET_NOK;

	configASSERT( xAnalogInterfaceSemprHandle != NULL );

	if (osOK == osSemaphoreAcquire(xAnalogInterfaceSemprHandle, ANALOG_INTERFACE_MUTEX_DEFAULT_WAIT_TIME_MS))
	{
		switch (sensor) {
		case SJC_CURRENT_SENSOR_STD:
			conv = xAdcMeasurements[MOTOR_CURRENT_CONV];
			*value = (conv/AD8418_ANALOG_FRONTED_GAIN)-AD8418_VREF;
			*value /= (AD8418_SHUNT_RESISTANCE*AD8418_GAIN);
			ret = SJC_RET_OK;
			break;
		case SJC_VOLTAGE_SENSOR:
			conv = xAdcMeasurements[MOTOR_VOLTAGE_CONV];
			*value = conv*VOLTAGE_SENSOR_RESISTIVE_CONDITIONER_GAIN;
			ret = SJC_RET_OK;
			break;

		default:
			ret = SJC_RET_NOK;
			break;
		}
		osSemaphoreRelease(xAnalogInterfaceSemprHandle);
	}

	return ret;
}

eSjcReturnType_t SJC_AnalogInterfaces_GetMeanSampleTime(xSjcMeanSampleTimeMeasurement_t * Ts)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	configASSERT( xAnalogInterfaceSemprHandle != NULL );

	if (osOK == osSemaphoreAcquire(xAnalogInterfaceSemprHandle, ANALOG_INTERFACE_MUTEX_DEFAULT_WAIT_TIME_MS))
	{
		if (xMeanSampleTimeObj.Samples == 0)
		{
			xMeanSampleTimeObj.Samples = 1;
		}

		*Ts = xMeanSampleTimeObj;
		xMeanSampleTimeObj = xSjcSampleTimeEmptyObj;

		osSemaphoreRelease(xAnalogInterfaceSemprHandle);
	}

	return ret;
}

void SJC_AnalogInterfaces_AdcConversionIsr(void)
{
	LL_TIM_DisableCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
	xMeanSampleTimeObj.AccumulatedTimerTicks += LL_TIM_GetCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
	LL_TIM_SetCounter(SJC_ADC_CHECK_TIMER_INSTANCE, 0);
	xMeanSampleTimeObj.Samples++;
	xIsrServiceCounter++;

	xAdcConversionBuffer[MOTOR_CURRENT_CONV] = LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_1);
	xAdcConversionBuffer[MOTOR_VOLTAGE_CONV] = LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_2);
	xAdcConversionBuffer[VREF_INT_CONV] = LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_3);

	ComputeFilteredMeasurements();

}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void ComputeFilteredMeasurements(void)
{
	xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV] *= CURRENT_SENSOR_COMP_FILTER_MAINT_RATIO;
	xAdcMeasurementValueBuffer[MOTOR_VOLTAGE_CONV] *= VOLTAGE_SENSOR_COMP_FILTER_MAINT_RATIO;
	xAdcMeasurementValueBuffer[VREF_INT_CONV] *= VREF_INT_SENSOR_COMP_FILTER_MAINT_RATIO;

	xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV] += __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, xAdcConversionBuffer[MOTOR_CURRENT_CONV], LL_ADC_RESOLUTION_12B)*CURRENT_SENSOR_COMP_FILTER_RENEW_RATIO/1000.0f;
	xAdcMeasurementValueBuffer[MOTOR_VOLTAGE_CONV] += __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, xAdcConversionBuffer[MOTOR_VOLTAGE_CONV], LL_ADC_RESOLUTION_12B)*VOLTAGE_SENSOR_COMP_FILTER_RENEW_RATIO/1000.0f;
	xAdcMeasurementValueBuffer[VREF_INT_CONV] += __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, xAdcConversionBuffer[VREF_INT_CONV], LL_ADC_RESOLUTION_12B)*VREF_INT_SENSOR_COMP_FILTER_RENEW_RATIO/1000.0f;
}
//void ComputeSampleTime(void)
//{
//
//	uint32_t dt;
//
//	if (xSampleTimeObj.ActualTick > xMeanSampleTimeObj.last_tick )
//	{
//		dt = xSampleTimeObj.ActualTick - xSampleTimeObj.last_tick;
//	}
//	else
//	{
//		dt = xSampleTimeObj.ActualTick + SJC_ADC_CHECK_TIMER_COUNT_PERIOD - xSampleTimeObj.last_tick;
//	}
//
//	xSampleTimeObj.abc++;
//	xSampleTimeObj.tick_counts++;
//	xSampleTimeObj.last_tick = xSampleTimeObj.ActualTick;
//	xSampleTimeObj.AccumulatedTimerTicks += ( SJC_ADC_CHECK_TIMER_RESOLUTION*dt);
//}


// ------------------------------------------------------------------- //
