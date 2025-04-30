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

static eSjcAdcFilterMode_t xAdcFilterMode = ADC_FILTER_EXP_THEN_BOX;
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
static void filter_none(uint8_t ch);
static void filter_exponential(uint8_t ch);
static void filter_box(uint8_t ch);
static void filter_exp_then_box(uint8_t ch);
static void filter_box_then_exp(uint8_t ch);

// Helpers
static void IncIsrServiceCounter(void) { xIsrServiceCounter++; }
static void ResetIsrServiceCounter(void) { xIsrServiceCounter = 0; }

// Function pointer table
static void (*const filter_function_table[])(uint8_t) = {
	[ADC_FILTER_NONE] = filter_none,
	[ADC_FILTER_EXPONENTIAL] = filter_exponential,
	[ADC_FILTER_BOX] = filter_box,
	[ADC_FILTER_EXP_THEN_BOX] = filter_exp_then_box,
	[ADC_FILTER_BOX_THEN_EXP] = filter_box_then_exp,
};

float raw[CONVERSION_ORDER_SIZE] = {0.0f, 0.0f, 0.0f};
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

	AnalogInterfaces_setFilterMode(ADC_FILTER_EXP_THEN_BOX, 100);
}

void AnalogInterfaces_enable(void)
{
	LL_ADC_EnableIT_OVR(SJC_ADC_CURRENT_INSTANCE);
	LL_ADC_EnableIT_OVR(SJC_ADC_VOLTAGE_INSTANCE);
	LL_ADC_EnableIT_JEOS(SJC_ADC_CURRENT_INSTANCE);
	LL_ADC_EnableIT_JEOS(SJC_ADC_VOLTAGE_INSTANCE);
	LL_ADC_Enable(SJC_ADC_CURRENT_INSTANCE);
	LL_ADC_Enable(SJC_ADC_VOLTAGE_INSTANCE);
	ResetIsrServiceCounter();
}

void AnalogInterfaces_disable(void)
{
	LL_ADC_Disable(SJC_ADC_CURRENT_INSTANCE);
	LL_ADC_Disable(SJC_ADC_VOLTAGE_INSTANCE);
	LL_ADC_DisableIT_OVR(SJC_ADC_CURRENT_INSTANCE);
	LL_ADC_DisableIT_OVR(SJC_ADC_VOLTAGE_INSTANCE);
	LL_ADC_DisableIT_JEOS(SJC_ADC_CURRENT_INSTANCE);
	LL_ADC_DisableIT_JEOS(SJC_ADC_VOLTAGE_INSTANCE);
}

void AnalogInterfaces_setFilterMode(const eSjcAdcFilterMode_t mode, const float fc)
{
	if (mode < ADC_FILTER_MODE_SIZE)
	{
		xAdcFilterMode = mode;

		DebugOutput_systemPrintf("\r\n[SJC] ADC Digital filter mode: %u", mode);

		if ((xAdcFilterMode == ADC_FILTER_EXPONENTIAL) || (xAdcFilterMode == ADC_FILTER_EXP_THEN_BOX) || (xAdcFilterMode == ADC_FILTER_BOX_THEN_EXP))
		{
			float fs = (xAdcFilterMode == ADC_FILTER_BOX_THEN_EXP) ? ANALOG_MEASUREMENT_ISR_FREQUENCY : ANALOG_IFACE_ISR_FREQUENCY;
			DebugOutput_systemPrintf("\r\n[SJC] ADC Digital filter sampling frequency: %.2f Hz", fs);
			for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
			{
				ExponentialFilter_set_sampling_frequency(&channel_filters[ch].exp, fs);
				ExponentialFilter_set_cutoff(&channel_filters[ch].exp, fc);
			}
			DebugOutput_systemPrintf("\r\n[SJC] ADC Digital filter cutoff frequency: %.2f Hz", fc);
		}
	}
	else
	{
		xAdcFilterMode = ADC_FILTER_NONE;
	}
}

eSjcReturnType_t AnalogInterfaces_updateConversionValues(void)
{
	configASSERT(xAnalogInterfaceSemprHandle != NULL);

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
	configASSERT(xAnalogInterfaceSemprHandle != NULL);

	if (value == NULL)
	{
		return SJC_RET_NOK;
	}

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
			*value = xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV] - (1.6566);
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
	configASSERT(xAnalogInterfaceSemprHandle != NULL);

	if (Ts == NULL)
	{
		return SJC_RET_NOK;
	}

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
	if ((handle == NULL) || (*handle == NULL))
	{
		DebugOutput_systemPrintf("[SJC] Fault: Invalid ADC update task handle!");
		return;
	}

	xAdcUpdateTaskHandle = *handle;
}

void SJC_AnalogInterfaces_Adc1ConversionIsr(void)
{
	raw[VREF_INT_CONV] = (float)__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_INJ_ReadConversionData12(SJC_ADC_VOLTAGE_INSTANCE, LL_ADC_INJ_RANK_1), LL_ADC_RESOLUTION_12B) / 1000.0f;
	raw[VREF_INT_CONV] += (float)__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_INJ_ReadConversionData12(SJC_ADC_VOLTAGE_INSTANCE, LL_ADC_INJ_RANK_2), LL_ADC_RESOLUTION_12B) / 1000.0f;

	raw[MOTOR_VOLTAGE_CONV] = (float)__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_INJ_ReadConversionData12(SJC_ADC_VOLTAGE_INSTANCE, LL_ADC_INJ_RANK_3), LL_ADC_RESOLUTION_12B) / 1000.0f;
	raw[MOTOR_VOLTAGE_CONV] += (float)__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_INJ_ReadConversionData12(SJC_ADC_VOLTAGE_INSTANCE, LL_ADC_INJ_RANK_4), LL_ADC_RESOLUTION_12B) / 1000.0f;

	raw[VREF_INT_CONV] /= 2.0f;
	raw[MOTOR_VOLTAGE_CONV] /= 2.0f;

	filter_function_table[xAdcFilterMode](VREF_INT_CONV);
	filter_function_table[xAdcFilterMode](MOTOR_VOLTAGE_CONV);
}

void SJC_AnalogInterfaces_Adc2ConversionIsr(void)
{
	raw[MOTOR_CURRENT_CONV] = (float)__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_INJ_ReadConversionData12(SJC_ADC_CURRENT_INSTANCE, LL_ADC_INJ_RANK_1), LL_ADC_RESOLUTION_12B) / 1000.0f;
	raw[MOTOR_CURRENT_CONV] += (float)__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_INJ_ReadConversionData12(SJC_ADC_CURRENT_INSTANCE, LL_ADC_INJ_RANK_2), LL_ADC_RESOLUTION_12B) / 1000.0f;
	raw[MOTOR_CURRENT_CONV] += (float)__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_INJ_ReadConversionData12(SJC_ADC_CURRENT_INSTANCE, LL_ADC_INJ_RANK_3), LL_ADC_RESOLUTION_12B) / 1000.0f;
	raw[MOTOR_CURRENT_CONV] += (float)__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_INJ_ReadConversionData12(SJC_ADC_CURRENT_INSTANCE, LL_ADC_INJ_RANK_4), LL_ADC_RESOLUTION_12B) / 1000.0f;
	raw[MOTOR_CURRENT_CONV] /= 4.0f;

	IncIsrServiceCounter();

	filter_function_table[xAdcFilterMode](MOTOR_CURRENT_CONV);

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
	if (LL_ADC_IsActiveFlag_JEOS(SJC_ADC_CURRENT_INSTANCE) != 0)
	{
		/* Clear flag ADC group injected end of sequence conversions */
		LL_ADC_ClearFlag_JEOS(SJC_ADC_CURRENT_INSTANCE);

		SJC_AnalogInterfaces_Adc2ConversionIsr();
	}

	if (LL_ADC_IsActiveFlag_JEOS(SJC_ADC_VOLTAGE_INSTANCE) != 0)
	{
		/* Clear flag ADC group injected end of sequence conversions */
		LL_ADC_ClearFlag_JEOS(SJC_ADC_VOLTAGE_INSTANCE);

		SJC_AnalogInterfaces_Adc1ConversionIsr();
	}

	/* Check whether ADC group regular overrun caused the ADC interruption */
	if (LL_ADC_IsActiveFlag_OVR(SJC_ADC_CURRENT_INSTANCE) != 0)
	{
		/* Clear flag ADC group regular overrun */
		LL_ADC_ClearFlag_OVR(SJC_ADC_CURRENT_INSTANCE);

		/* Call interruption treatment function */
		//		DebugOutput_systemPrintf("\r\n[HW FAULT] ADC2 Overrun!\n");
		Error_Handler();
	}

	if (LL_ADC_IsActiveFlag_OVR(SJC_ADC_VOLTAGE_INSTANCE) != 0)
	{
		/* Clear flag ADC group regular overrun */
		LL_ADC_ClearFlag_OVR(SJC_ADC_VOLTAGE_INSTANCE);

		/* Call interruption treatment function */
		//			DebugOutput_systemPrintf("\r\n[HW FAULT] ADC1 Overrun!\n");
		Error_Handler();
	}
}

// Filter implementations
static void filter_none(uint8_t ch)
{
	xFilteredSampleFromIsr.channels[ch] = raw[ch];
}

static void filter_exponential(uint8_t ch)
{
	xFilteredSampleFromIsr.channels[ch] = ExponentialFilter_update(&channel_filters[ch].exp, raw[ch]);
}

static void filter_box(uint8_t ch)
{
	BoxFilter_add_sample(&channel_filters[ch].box, raw[ch]);
	if (BoxFilter_ready(&channel_filters[ch].box))
		xFilteredSampleFromIsr.channels[ch] = BoxFilter_get_average_and_reset(&channel_filters[ch].box);
}

static void filter_exp_then_box(uint8_t ch)
{
	float exp_val = ExponentialFilter_update(&channel_filters[ch].exp, raw[ch]);
	BoxFilter_add_sample(&channel_filters[ch].box, exp_val);
	if (BoxFilter_ready(&channel_filters[ch].box))
		xFilteredSampleFromIsr.channels[ch] = BoxFilter_get_average_and_reset(&channel_filters[ch].box);
}

static void filter_box_then_exp(uint8_t ch)
{
	BoxFilter_add_sample(&channel_filters[ch].box, raw[ch]);
	if (BoxFilter_ready(&channel_filters[ch].box))
	{
		float avg = BoxFilter_get_average_and_reset(&channel_filters[ch].box);
		xFilteredSampleFromIsr.channels[ch] = ExponentialFilter_update(&channel_filters[ch].exp, avg);
	}
}

// ------------------------------------------------------------------- //
