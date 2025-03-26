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
#include "base/include/configs.h"
#include "modules/include/analog_interfaces.h"
#include "modules/include/debug_output.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //
#define VDDA_APPLI ((uint32_t)3300UL)
#define ANALOG_INTERFACE_MUTEX_DEFAULT_WAIT_TIME_MS ((uint32_t)3UL)
#define ANALOG_INTERFACE_MUTEX_CONVERSION_WAIT_TIME_MS ((uint32_t)osWaitForever)
#define CONV_MILLIVOLT_TO_VOLT(X) (X / 1000.0f)
#define ANALOG_MEASUREMENT_BUFFER_SIZE ((uint32_t)100UL)

#define ANALOG_IFACE_ISR_FREQUENCY (float)20000.0f
#define ANALOG_MEASUREMENT_ISR_FREQUENCY (float)1000.0f
#define ANALOG_MEASUREMENT_ISR_TICKS ((uint32_t)(ANALOG_IFACE_ISR_FREQUENCY / ANALOG_MEASUREMENT_ISR_FREQUENCY))
// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
typedef enum
{
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
const osSemaphoreAttr_t AnalogInterfaceSemaphore_attributes = { .name = "AnalogInterfaceSemaphore", .cb_mem = &AnalogInterfaceSemaphoreControlBlock, .cb_size = sizeof(AnalogInterfaceSemaphoreControlBlock),
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
void AnalogInterfaces_init(void)
{
	/* ADC Peripheral Configuration ---------------------------------- */
	xAnalogInterfaceSemprHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &AnalogInterfaceSemaphore_attributes);

	if (xAnalogInterfaceSemprHandle == NULL)
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Analog interface semaphore not created...");
	}
	else
	{
		osSemaphoreRelease(xAnalogInterfaceSemprHandle);
	}

	if (false == LL_TIM_IsEnabledCounter(SJC_ADC_CHECK_TIMER_INSTANCE))
	{
		LL_TIM_EnableCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
	}
}

void AnalogInterfaces_enable(void)
{
	LL_ADC_EnableIT_OVR(SJC_ADC_INSTANCE);
	LL_ADC_EnableIT_JEOS(SJC_ADC_INSTANCE);

	LL_ADC_Enable(SJC_ADC_INSTANCE);

	ResetIsrServiceCounter();
}

void AnalogInterfaces_disable(void)
{
	LL_ADC_Disable(SJC_ADC_INSTANCE);

	LL_ADC_EnableIT_OVR(SJC_ADC_INSTANCE);
	LL_ADC_EnableIT_JEOS(SJC_ADC_INSTANCE);
}

eSjcReturnType_t AnalogInterfaces_updateConversionValues(void)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	configASSERT(xAnalogInterfaceSemprHandle != NULL);

	if (osOK == osSemaphoreAcquire(xAnalogInterfaceSemprHandle, ANALOG_INTERFACE_MUTEX_CONVERSION_WAIT_TIME_MS))
	{
		// LL_TIM_DisableCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
		xMeanSampleTimeObj.AccumulatedTimerTicks += LL_TIM_GetCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
		LL_TIM_SetCounter(SJC_ADC_CHECK_TIMER_INSTANCE, 0);
		xMeanSampleTimeObj.Samples++;

		if (xIsrServiceCounter != 20)
		{
			DebugOutput_systemPrintf("\n\n\n\r[OS FAULT] ADC Measurement check timer wrong period!");
			//			for (;;);
		}
		ResetIsrServiceCounter();

		ComputeFilteredMeasurements();

		osSemaphoreRelease(xAnalogInterfaceSemprHandle);

		ret = SJC_RET_OK;
	}

	return ret;
}

eSjcReturnType_t AnalogInterfaces_checkUpdateConversionTime(void)
{
	eSjcReturnType_t ret = (xIsrServiceCounter >= ANALOG_MEASUREMENT_ISR_TICKS) ? SJC_RET_OK : SJC_RET_NOK;
	return ret;
}

eSjcReturnType_t AnalogInterfaces_getAnalogConversion(const eSjcAnalogSensorType_t sensor, float *value)
{
	eSjcReturnType_t ret = SJC_RET_NOK;
	*value = ZERO_FLOAT;

	configASSERT(xAnalogInterfaceSemprHandle != NULL);

	if (osOK == osSemaphoreAcquire(xAnalogInterfaceSemprHandle, ANALOG_INTERFACE_MUTEX_DEFAULT_WAIT_TIME_MS))
	{
		switch (sensor)
		{
		case SJC_CURRENT_SENSOR_STD:
			*value = (xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV] / AD8418_ANALOG_FRONTED_GAIN) - ANALOG_CURRENT_SENSOR_VREF;
			*value /= (AD8418_SHUNT_RESISTANCE * AD8418_GAIN);
			ret = SJC_RET_OK;
			break;
		case SJC_CURRENT_SENSOR_AUX:
			*value = (xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV] / ACS712_ANALOG_FRONTED_GAIN) - ANALOG_CURRENT_SENSOR_VREF;
			*value /= ACS712_GAIN;
			ret = SJC_RET_OK;
			break;

		case SJC_VOLTAGE_SENSOR:
			*value = xAdcMeasurementValueBuffer[MOTOR_VOLTAGE_CONV] * VOLTAGE_SENSOR_RESISTIVE_CONDITIONER_GAIN;
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

eSjcReturnType_t AnalogInterfaces_getMeanConversionDelay(xSjcMeanSampleTimeMeasurement_t *Ts)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	configASSERT(xAnalogInterfaceSemprHandle != NULL);

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
	xAdcConversionBuffer[MOTOR_CURRENT_CONV][xAdcConversionIndex] = LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_1);
	xAdcConversionBuffer[MOTOR_VOLTAGE_CONV][xAdcConversionIndex] = LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_2);
	xAdcConversionBuffer[VREF_INT_CONV][xAdcConversionIndex++] = LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_3);
	IncIsrServiceCounter();
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
	float xAdcMeasurementValueBufferAux[3] = {0.0f, 0.0f, 0.0f};
	xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV] *= CURRENT_SENSOR_COMP_FILTER_MAINT_RATIO;
	xAdcMeasurementValueBuffer[MOTOR_VOLTAGE_CONV] *= VOLTAGE_SENSOR_COMP_FILTER_MAINT_RATIO;
	xAdcMeasurementValueBuffer[VREF_INT_CONV] *= VREF_INT_SENSOR_COMP_FILTER_MAINT_RATIO;


	for (uint32_t i = 0; i < xAdcConversionIndex; i++)
	{
		xAdcMeasurementValueBufferAux[MOTOR_CURRENT_CONV] += __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, xAdcConversionBuffer[MOTOR_CURRENT_CONV][i], LL_ADC_RESOLUTION_12B) / (xAdcConversionIndex*1000.0f);
		xAdcMeasurementValueBufferAux[MOTOR_VOLTAGE_CONV] += __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, xAdcConversionBuffer[MOTOR_VOLTAGE_CONV][i], LL_ADC_RESOLUTION_12B) / (xAdcConversionIndex*1000.0f);
		xAdcMeasurementValueBufferAux[VREF_INT_CONV] += __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, xAdcConversionBuffer[VREF_INT_CONV][i], LL_ADC_RESOLUTION_12B) / (xAdcConversionIndex*1000.0f);
	}

	xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV] += xAdcMeasurementValueBufferAux[MOTOR_CURRENT_CONV] * CURRENT_SENSOR_COMP_FILTER_RENEW_RATIO;
	xAdcMeasurementValueBuffer[MOTOR_VOLTAGE_CONV] += xAdcMeasurementValueBufferAux[MOTOR_VOLTAGE_CONV] * VOLTAGE_SENSOR_COMP_FILTER_RENEW_RATIO;
	xAdcMeasurementValueBuffer[VREF_INT_CONV] += xAdcMeasurementValueBuffer[VREF_INT_CONV] * VREF_INT_SENSOR_COMP_FILTER_RENEW_RATIO;

	xAdcConversionIndex = 0;
}
// ------------------------------------------------------------------- //
