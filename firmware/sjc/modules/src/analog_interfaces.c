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
#define ANALOG_MEASUREMENT_BUFFER_SIZE					((uint32_t) 100UL)

#define ANALOG_IFACE_ISR_FREQUENCY						(float) 20000.0f
#define ANALOG_MEASUREMENT_ISR_FREQUENCY				(float) 1000.0f
#define ANALOG_MEASUREMENT_ISR_TICKS					((uint32_t) (ANALOG_IFACE_ISR_FREQUENCY/ANALOG_MEASUREMENT_ISR_FREQUENCY))
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

volatile uint32_t xAdcConversionIndex = 0;
static uint32_t xAdcConversionBuffer[CONVERSION_ORDER_SIZE][ANALOG_MEASUREMENT_BUFFER_SIZE];
static float xAdcMeasurementValueBuffer[CONVERSION_ORDER_SIZE];

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
void IncIsrServiceCounter(void);
void ResetIsrServiceCounter(void);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void SJC_AnalogInterfaces_Init(void)
{
	/* ADC Peripheral Configuration ---------------------------------- */
	xAnalogInterfaceSemprHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &AnalogInterfaceSemaphore_attributes);

	if (xAnalogInterfaceSemprHandle == NULL)
	{
		SJC_DebugOutput_SystemPrintf("\r\n[OS FAULT] Analog interface semaphore not created...");
	} else
	{
		osSemaphoreRelease(xAnalogInterfaceSemprHandle);
	}

	if (false == LL_TIM_IsEnabledCounter(SJC_ADC_CHECK_TIMER_INSTANCE))
	{
		LL_TIM_EnableCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
	}
}

void SJC_AnalogInterfaces_MeasurementEnable(void)
{
	LL_ADC_EnableIT_OVR(SJC_ADC_INSTANCE);
	LL_ADC_EnableIT_JEOS(SJC_ADC_INSTANCE);

	LL_ADC_Enable(SJC_ADC_INSTANCE);

	ResetIsrServiceCounter();
}

void SJC_AnalogInterfaces_MeasurementDisable(void)
{
	LL_ADC_Disable(SJC_ADC_INSTANCE);

	LL_ADC_EnableIT_OVR(SJC_ADC_INSTANCE);
	LL_ADC_EnableIT_JEOS(SJC_ADC_INSTANCE);
}

eSjcReturnType_t SJC_AnalogInterfaces_UpdateConversionValues(void)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	configASSERT( xAnalogInterfaceSemprHandle != NULL );

	if (osOK == osSemaphoreAcquire(xAnalogInterfaceSemprHandle, ANALOG_INTERFACE_MUTEX_CONVERSION_WAIT_TIME_MS))
	{
		if (xIsrServiceCounter != 20)
		{
			SJC_DebugOutput_SystemPrintf("\n\n\n\r[OS FAULT] ADC Measurement check timer wrong period!");
			//			for (;;);
		}
		ResetIsrServiceCounter();

		ComputeFilteredMeasurements();

		osSemaphoreRelease(xAnalogInterfaceSemprHandle);

		ret = SJC_RET_OK;
	}

	return ret;
}

eSjcReturnType_t SJC_AnalogInterfaces_CheckUpdateConversionTime(void)
{
	eSjcReturnType_t ret = (xIsrServiceCounter >= ANALOG_MEASUREMENT_ISR_TICKS) ? SJC_RET_OK : SJC_RET_NOK;
	return ret;
}

eSjcReturnType_t SJC_AnalogInterfaces_GetAnalogConversion(const eSjcAnalogSensorType_t sensor, float * value)
{
	eSjcReturnType_t ret = SJC_RET_NOK;
	*value = ZERO_FLOAT;

	configASSERT( xAnalogInterfaceSemprHandle != NULL );

	if (osOK == osSemaphoreAcquire(xAnalogInterfaceSemprHandle, ANALOG_INTERFACE_MUTEX_DEFAULT_WAIT_TIME_MS))
	{
		switch (sensor) {
		case SJC_CURRENT_SENSOR_STD:
			*value = (xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV]/AD8418_ANALOG_FRONTED_GAIN)-ANALOG_CURRENT_SENSOR_VREF;
			*value /= (AD8418_SHUNT_RESISTANCE*AD8418_GAIN);
			ret = SJC_RET_OK;
			break;
		case SJC_CURRENT_SENSOR_AUX:
			*value = (xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV]/ACS712_ANALOG_FRONTED_GAIN)-ANALOG_CURRENT_SENSOR_VREF;
			*value /= ACS712_GAIN;
			ret = SJC_RET_OK;
			break;

		case SJC_VOLTAGE_SENSOR:
			*value = xAdcMeasurementValueBuffer[MOTOR_VOLTAGE_CONV]*VOLTAGE_SENSOR_RESISTIVE_CONDITIONER_GAIN;
			ret = SJC_RET_OK;
			break;

		default:
			*value = ZERO_FLOAT;
			break;
		}

		osSemaphoreRelease(xAnalogInterfaceSemprHandle);
	}

	return ret;
}

eSjcReturnType_t SJC_AnalogInterfaces_GetMeanConversionDelay(xSjcMeanSampleTimeMeasurement_t * Ts)
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

		ret = SJC_RET_OK;
	}

	return ret;
}

void SJC_AnalogInterfaces_AdcConversionIsr(void)
{
	LL_TIM_DisableCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
	xMeanSampleTimeObj.AccumulatedTimerTicks += LL_TIM_GetCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
	LL_TIM_SetCounter(SJC_ADC_CHECK_TIMER_INSTANCE, 0);
	xMeanSampleTimeObj.Samples++;
	IncIsrServiceCounter();

	xAdcConversionBuffer[MOTOR_CURRENT_CONV][xAdcConversionIndex] = LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_1);
	xAdcConversionBuffer[MOTOR_VOLTAGE_CONV][xAdcConversionIndex] = LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_2);
	xAdcConversionBuffer[VREF_INT_CONV][xAdcConversionIndex++] = LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_3);
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void IncIsrServiceCounter(void)
{
	xIsrServiceCounter++;
}

void ResetIsrServiceCounter(void)
{
	xIsrServiceCounter = 0;
}

void ComputeFilteredMeasurements(void)
{
	for (int i = 0; i < xAdcConversionIndex; i++)
	{
		xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV] *= CURRENT_SENSOR_COMP_FILTER_MAINT_RATIO;
		xAdcMeasurementValueBuffer[MOTOR_VOLTAGE_CONV] *= VOLTAGE_SENSOR_COMP_FILTER_MAINT_RATIO;
		xAdcMeasurementValueBuffer[VREF_INT_CONV] *= VREF_INT_SENSOR_COMP_FILTER_MAINT_RATIO;

		xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV] += __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, xAdcConversionBuffer[MOTOR_CURRENT_CONV][i], LL_ADC_RESOLUTION_12B)*CURRENT_SENSOR_COMP_FILTER_RENEW_RATIO/1000.0f;
		xAdcMeasurementValueBuffer[MOTOR_VOLTAGE_CONV] += __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, xAdcConversionBuffer[MOTOR_VOLTAGE_CONV][i], LL_ADC_RESOLUTION_12B)*VOLTAGE_SENSOR_COMP_FILTER_RENEW_RATIO/1000.0f;
		xAdcMeasurementValueBuffer[VREF_INT_CONV] += __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, xAdcConversionBuffer[VREF_INT_CONV][i], LL_ADC_RESOLUTION_12B)*VREF_INT_SENSOR_COMP_FILTER_RENEW_RATIO/1000.0f;
	}

	xAdcConversionIndex = 0;
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
