/*
 * analog_interfaces.c
 *
 *  Created on: Sep 9, 2023
 * 	Updated: Mar 2025
 *      Author: tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "adc.h"
#include "cmsis_os.h"
#include "main.h"
#include "tim.h"
// ------------------------------------------------------------------- //

// RTOS includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "cmsis_os.h"
// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "base/include/configs.h"
#include "modules/include/analog_interfaces.h"
#include "modules/include/debug_output.h"
#include "system_types/include/types.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //
#define VDDA_APPLI ((uint32_t)3300UL)
#define ANALOG_INTERFACE_MUTEX_DEFAULT_WAIT_TIME_MS ((uint32_t)3UL)
#define ANALOG_INTERFACE_MUTEX_CONVERSION_WAIT_TIME_MS ((uint32_t)osWaitForever)
#define ANALOG_IFACE_ISR_FREQUENCY (float)20000.0f
#define ANALOG_MEASUREMENT_ISR_FREQUENCY (float)1000.0f
#define ANALOG_MEASUREMENT_ISR_TICKS ((uint32_t)(ANALOG_IFACE_ISR_FREQUENCY / ANALOG_MEASUREMENT_ISR_FREQUENCY))
// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
// ADC channels
typedef enum
{
	MOTOR_CURRENT_CONV = 0,
	MOTOR_VOLTAGE_CONV,
	VREF_INT_CONV,
	CONVERSION_ORDER_SIZE
} eSjcConversionOrder_t;

typedef struct
{
	float channels[CONVERSION_ORDER_SIZE];
} SjcAdcFilteredSample_t;

typedef struct
{
	ExponentialFilter_t exp;
	BoxFilter_t box;
} SjcAdcFilter_t;

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static osSemaphoreId_t xAnalogInterfaceSemprHandle = NULL;
static StaticSemaphore_t AnalogInterfaceSemaphoreControlBlock;
static TaskHandle_t xAdcUpdateTaskHandle = NULL;

static float xAdcMeasurementValueBuffer[CONVERSION_ORDER_SIZE] = {0};
static volatile SjcAdcFilteredSample_t xFilteredSampleFromIsr = {0};
static SjcAdcFilter_t channel_filters[CONVERSION_ORDER_SIZE];
static xSjcMeanSampleTimeMeasurement_t xMeanSampleTimeObj = xSjcSampleTimeEmptyObj;
static volatile uint32_t xIsrServiceCounter = 0;
static volatile uint32_t xFilterWindowTickCounter = 0;

static eSjcAdcFilterMode_t xAdcFilterMode = ADC_FILTER_EXPONENTIAL;
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
static void filter_none(const float *raw);
static void filter_exponential(const float *raw);
static void filter_box(const float *raw);
static void filter_exp_then_box(const float *raw);
static void filter_box_then_exp(const float *raw);

// Helpers
static void IncIsrServiceCounter(void) { xIsrServiceCounter++; }
static void ResetIsrServiceCounter(void) { xIsrServiceCounter = 0; }

// Function pointer table
static const void (*filter_function_table[])(const float *) = {
	[ADC_FILTER_NONE] = filter_none,
	[ADC_FILTER_EXPONENTIAL] = filter_exponential,
	[ADC_FILTER_BOX] = filter_box,
	[ADC_FILTER_EXP_THEN_BOX] = filter_exp_then_box,
	[ADC_FILTER_BOX_THEN_EXP] = filter_box_then_exp,
};
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void AnalogInterfaces_init(void)
{
	xAnalogInterfaceSemprHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &(osSemaphoreAttr_t){.name = "AnalogInterfaceSemaphore", .cb_mem = &AnalogInterfaceSemaphoreControlBlock, .cb_size = sizeof(AnalogInterfaceSemaphoreControlBlock)});

	if (xAnalogInterfaceSemprHandle == NULL)
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Analog interface semaphore not created...");
	}
	else
	{
		osSemaphoreRelease(xAnalogInterfaceSemprHandle);
	}

	for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
	{
		ExponentialFilter_init(&channel_filters[ch].exp, ANALOG_MEASUREMENT_ISR_FREQUENCY, ANALOG_IFACE_ISR_FREQUENCY);
		BoxFilter_init(&channel_filters[ch].box, (uint16_t)ANALOG_MEASUREMENT_ISR_TICKS);
	}

	if (!LL_TIM_IsEnabledCounter(SJC_ADC_CHECK_TIMER_INSTANCE))
	{
		LL_TIM_EnableCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
	}

	AnalogInterfaces_setFilterMode(ADC_FILTER_NONE);
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
	LL_ADC_DisableIT_OVR(SJC_ADC_INSTANCE);
	LL_ADC_DisableIT_JEOS(SJC_ADC_INSTANCE);
}

void AnalogInterfaces_setFilterMode(eSjcAdcFilterMode_t mode)
{
	if (mode < ADC_FILTER_MODE_SIZE)
	{
		xAdcFilterMode = mode;

		DebugOutput_systemPrintf("\r\n[OS] ADC Digital filter mode: %u", mode);

		if ((xAdcFilterMode == ADC_FILTER_EXPONENTIAL) || (xAdcFilterMode == ADC_FILTER_EXP_THEN_BOX) || (xAdcFilterMode == ADC_FILTER_BOX_THEN_EXP))
		{
			float fs = (xAdcFilterMode == ADC_FILTER_BOX_THEN_EXP) ? ANALOG_MEASUREMENT_ISR_FREQUENCY : ANALOG_IFACE_ISR_FREQUENCY;
			DebugOutput_systemPrintf("\r\n[OS] ADC Digital filter sampling frequency: %.2f Hz", fs);
			for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
			{
				ExponentialFilter_set_sampling_frequency(&channel_filters[ch].exp, fs);
			}
		}
	}
	else
	{
		xAdcFilterMode = ADC_FILTER_NONE;
	}
}

void AnalogInterfaces_setExponentialCutoffFrequency(float fc)
{
	DebugOutput_systemPrintf("\r\n[OS] ADC Digital filter cutoff frequency: %.2f Hz", fc);

	for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
	{
		ExponentialFilter_set_cutoff(&channel_filters[ch].exp, fc);
	}
}

eSjcReturnType_t AnalogInterfaces_updateConversionValues(void)
{
	if (osOK == osSemaphoreAcquire(xAnalogInterfaceSemprHandle, ANALOG_INTERFACE_MUTEX_CONVERSION_WAIT_TIME_MS))
	{
		/* Check converion times and conversion errors                           */
		/* --------------------------------------------------------------------- */
		// LL_TIM_DisableCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
		xMeanSampleTimeObj.AccumulatedTimerTicks += LL_TIM_GetCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
		LL_TIM_SetCounter(SJC_ADC_CHECK_TIMER_INSTANCE, 0);
		xMeanSampleTimeObj.Samples++;

		if ((xIsrServiceCounter < 20) || (xIsrServiceCounter > 25))
		{
			DebugOutput_systemPrintf("\n\n\n\r[OS FAULT] ADC Measurement check timer wrong period! (%d cycles)", xIsrServiceCounter);
		}

		ResetIsrServiceCounter();
		/* --------------------------------------------------------------------- */

		/* Process filtered measurements                                         */
		/* --------------------------------------------------------------------- */

		for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
		{
			xAdcMeasurementValueBuffer[ch] = xFilteredSampleFromIsr.channels[ch];
		}
		/* --------------------------------------------------------------------- */

		osSemaphoreRelease(xAnalogInterfaceSemprHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t AnalogInterfaces_getAnalogConversion(const eSjcAnalogSensorType_t sensor, float *value)
{
	if (value == NULL)
		return SJC_RET_NOK;
	*value = ZERO_FLOAT;

	if (osOK == osSemaphoreAcquire(xAnalogInterfaceSemprHandle, ANALOG_INTERFACE_MUTEX_DEFAULT_WAIT_TIME_MS))
	{
		switch (sensor)
		{
		case SJC_CURRENT_SENSOR_STD:
			*value = (xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV] / AD8418_ANALOG_FRONTED_GAIN) - ANALOG_CURRENT_SENSOR_VREF;
			*value /= (AD8418_SHUNT_RESISTANCE * AD8418_GAIN);
			break;
		case SJC_CURRENT_SENSOR_AUX:
//			*value = (xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV] / ACS712_ANALOG_FRONTED_GAIN) - ANALOG_CURRENT_SENSOR_VREF;
//			*value /= ACS712_GAIN;
			*value = xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV]-(1.6566);
			*value /= (0.066265);

			break;
		case SJC_VOLTAGE_SENSOR:
			*value = xAdcMeasurementValueBuffer[MOTOR_VOLTAGE_CONV] * VOLTAGE_SENSOR_RESISTIVE_CONDITIONER_GAIN;
			break;
		default:
			*value = ZERO_FLOAT;
			break;
		}
		osSemaphoreRelease(xAnalogInterfaceSemprHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t AnalogInterfaces_getMeanConversionDelay(xSjcMeanSampleTimeMeasurement_t *Ts)
{
	if (Ts == NULL)
		return SJC_RET_NOK;

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
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

void AnalogInterfaces_registerUpdateTaskHandle(TaskHandle_t *handle)
{
	configASSERT(handle != NULL); // Safety check for null pointer
	xAdcUpdateTaskHandle = *handle;
}

void SJC_AnalogInterfaces_AdcConversionIsr(void)
{
	float raw[CONVERSION_ORDER_SIZE];
	raw[MOTOR_CURRENT_CONV] = (float)__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_1), LL_ADC_RESOLUTION_12B) / 1000.0f;
	raw[MOTOR_VOLTAGE_CONV] = (float)__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_2), LL_ADC_RESOLUTION_12B) / 1000.0f;
	raw[VREF_INT_CONV] = (float)__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_3), LL_ADC_RESOLUTION_12B) / 1000.0f;

	IncIsrServiceCounter();

	if (xAdcFilterMode < ADC_FILTER_MODE_SIZE)
	{
		filter_function_table[xAdcFilterMode](raw);
	}
	else
	{
		filter_none(raw);
	}

	if (++xFilterWindowTickCounter >= ANALOG_MEASUREMENT_ISR_TICKS)
	{
		xFilterWindowTickCounter = 0;

		if (xAdcUpdateTaskHandle != NULL)
		{
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			vTaskNotifyGiveFromISR(xAdcUpdateTaskHandle, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		else
		{
			DebugOutput_systemPrintf("\n\n\n\r[OS FAULT] ADC Measurement task update handle is NULL!");
		}
	}
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void ADC_IRQHandler(void)
{
	if (LL_ADC_IsActiveFlag_JEOS(SJC_ADC_INSTANCE) != 0)
	{
		/* Clear flag ADC group injected end of sequence conversions */
		LL_ADC_ClearFlag_JEOS(SJC_ADC_INSTANCE);

		SJC_AnalogInterfaces_AdcConversionIsr();
	}

	/* Check whether ADC group regular overrun caused the ADC interruption */
	if (LL_ADC_IsActiveFlag_OVR(SJC_ADC_INSTANCE) != 0)
	{
		/* Clear flag ADC group regular overrun */
		LL_ADC_ClearFlag_OVR(SJC_ADC_INSTANCE);

		/* Call interruption treatment function */
		DebugOutput_systemPrintf("\r\n[HW FAULT] ADC Overrun!\n");
		Error_Handler();
	}
}

// Filter implementations
static void filter_none(const float *raw)
{
	for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
	{
		xFilteredSampleFromIsr.channels[ch] = raw[ch];
	}
}

static void filter_exponential(const float *raw)
{
	for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
	{
		xFilteredSampleFromIsr.channels[ch] = ExponentialFilter_update(&channel_filters[ch].exp, raw[ch]);
	}
}

static void filter_box(const float *raw)
{
	for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
	{
		BoxFilter_add_sample(&channel_filters[ch].box, raw[ch]);
	}

	if (!BoxFilter_ready(&channel_filters[0].box))
		return;

	for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
	{
		xFilteredSampleFromIsr.channels[ch] = BoxFilter_get_average_and_reset(&channel_filters[ch].box);
	}
}

static void filter_exp_then_box(const float *raw)
{
	for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
	{
		float exp_filtered = ExponentialFilter_update(&channel_filters[ch].exp, raw[ch]);
		BoxFilter_add_sample(&channel_filters[ch].box, exp_filtered);
	}

	if (!BoxFilter_ready(&channel_filters[0].box))
		return;

	for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
	{
		xFilteredSampleFromIsr.channels[ch] = BoxFilter_get_average_and_reset(&channel_filters[ch].box);
	}
}

static void filter_box_then_exp(const float *raw)
{
	for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
	{
		BoxFilter_add_sample(&channel_filters[ch].box, raw[ch]);
	}

	if (!BoxFilter_ready(&channel_filters[0].box))
		return;

	for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
	{
		float box_output = BoxFilter_get_average_and_reset(&channel_filters[ch].box);
		xFilteredSampleFromIsr.channels[ch] = ExponentialFilter_update(&channel_filters[ch].exp, box_output);
	}
}
// ------------------------------------------------------------------- //
