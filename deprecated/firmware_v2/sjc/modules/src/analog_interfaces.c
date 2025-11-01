/*
 * analog_interfaces.c
 *
 *  Created on: Sep 9, 2023
 *  Updated: Mar 2025
 *      Author: tiago
 *
 *  This module handles analog interfaces (currents, voltages, VrefInt)
 *  using:
 *   - ADC1 (voltage side, injected mode, SW/trigger start, ISR based)
 *   - ADC2 (current side, regular mode, timer-triggered with DMA)
 *
 *  It supports exponential and box filtering, RTOS task notification,
 *  and provides thread-safe API for measurement access.
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
#include "modules/include/digital_interfaces.h"
#include "modules/include/motor_drive.h"
#include "system_types/include/types.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //
// Supply and mutex config
#define VDDA_APPLI                              ((uint32_t)3300UL)     ///< Supply voltage in mV
#define ANALOG_INTERFACE_MUTEX_DEFAULT_WAIT_TIME_MS    ((uint32_t)3UL)
#define ANALOG_INTERFACE_MUTEX_CONVERSION_WAIT_TIME_MS ((uint32_t)osWaitForever)

// Measurement timing
#define ANALOG_MEASUREMENT_PWM_TIMER_FREQ     (20000.0f)   ///< PWM frequency (Hz) = 20 kHz
#define ANALOG_MEASUREMENT_HIGH_RES_TIMER_FREQ (500000.0f) ///< High-resolution timer frequency (Hz)
#define ANALOG_MEASUREMENT_DEFAULT_FREQ       (1000.0f)   ///< Default filter cutoff frequency (Hz)
#define ANALOG_MEASUREMENT_ISR_TICKS \
    ((uint32_t)(ANALOG_MEASUREMENT_PWM_TIMER_FREQ / ANALOG_MEASUREMENT_DEFAULT_FREQ))

// PWM cycle timing
#define PWM_CYCLE_PERIOD_SEC   (1.0f / (ANALOG_MEASUREMENT_PWM_TIMER_FREQ)) ///< PWM period = 50 µs @ 20 kHz

// ADC sampling config
#define ADC_SAMPLE_PERIOD_SEC  (1.0e-3f)     ///< Processing window = 1 ms
#define ADC_SAMPLES_PER_PWM_CYCLE (25u)      ///< 25 ADC samples per PWM cycle (2 µs spacing)

// Derived ADC frequency
#define ADC_DMA_CONVERSION_FREQ_HZ  ((float)(ADC_SAMPLES_PER_PWM_CYCLE) / (PWM_CYCLE_PERIOD_SEC)) ///< Effective ADC conversion rate (Hz)

// ADC DMA buffer config
#define ADC_DMA_BANKS          (2u)
#define ADC_SAMPLES_PER_BANK  ((uint32_t)(ADC_DMA_CONVERSION_FREQ_HZ * ADC_SAMPLE_PERIOD_SEC)) ///< Samples per DMA bank (e.g. 500 @ 1 ms window)


// ------------------------------------------------------------------- //
// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
static uint16_t adc_buf[ADC_DMA_BANKS * ADC_SAMPLES_PER_BANK];  // ADC2 buffer
static volatile uint32_t bank_current = 0;                      // DMA bank currently in use
static volatile uint32_t bank_ready   = 0;                      // DMA bank ready to be proces

typedef enum
{
    MOTOR_CURRENT_CONV = 0,
    DRIVE_SUPPLY_VOLTAGE_CONV,
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
static volatile float raw[CONVERSION_ORDER_SIZE] = {0.0f, 0.0f, 0.0f};
static volatile SjcAdcFilteredSample_t xFilteredSampleFromIsr = {0};

static SjcAdcFilter_t channel_filters[CONVERSION_ORDER_SIZE];

static xSjcMeanSampleTimeMeasurement_t xMeanSampleTimeObj = xSjcSampleTimeEmptyObj;
static volatile uint32_t xIsrServiceCounter = 0;
static volatile uint32_t xFilterWindowTickCounter = 0;

static eSjcAdcFilterMode_t xAdcFilterMode = ADC_FILTER_BOX_THEN_EXP;
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //

/**
 * @brief No filtering applied. Raw ADC values are used directly.
 */
static void filter_none(uint8_t ch);

/**
 * @brief Exponential IIR filtering applied to each ADC sample.
 *
 * Provides fast response with high-frequency ripple attenuation.
 */
static void filter_exponential(uint8_t ch);

/**
 * @brief Box (moving average) filtering across a fixed window of samples.
 *
 * Smooths out short-term noise at the cost of increased delay.
 */
static void filter_box(uint8_t ch);

/**
 * @brief Exponential filtering on every ADC sample (≈ 2 µs),
 *        then accumulation over one PWM cycle (25 samples),
 *        followed by a Box filter on cycle averages (20 cycles ≈ 1 ms).
 *
 * This approach combines:
 * - Fast ripple attenuation (exponential per sample)
 * - Cycle-based smoothing (BoxFilter over cycles)
 *
 * Widely used in motor current sensing for robust noise rejection
 * with near real-time response.
 */
static void filter_exp_then_box_per_cycle(uint8_t ch);

/**
 * @brief Box filter first, then exponential filter on the averaged result.
 *
 * Useful when strong cycle averaging is required before exponential smoothing.
 */
static void filter_box_then_exp(uint8_t ch);

/**
 * @brief Exponential filter first, then Box filter on the exponential outputs.
 *
 * A simpler variant compared to the per-cycle method.
 */
static void filter_exp_then_box(uint8_t ch);

/* ------------------------------------------------------------------------- */
/*                       ISR Service Counter Utilities                       */
/* ------------------------------------------------------------------------- */

/**
 * @brief Increment the ISR service counter.
 */
static inline void IncIsrServiceCounter(void) { xIsrServiceCounter++; }

/**
 * @brief Reset the ISR service counter to zero.
 */
static inline void ResetIsrServiceCounter(void) { xIsrServiceCounter = 0; }

/**
 * @brief Get the current ISR service counter value.
 *
 * @return Current counter as unsigned 32-bit.
 */
static inline uint32_t GetIsrServiceCounter(void) { return xIsrServiceCounter; }

// ------------------------------------------------------------------- //

// Function pointer table -------------------------------------------- //
// ------------------------------------------------------------------- //
/**
 * @brief Lookup table mapping filter mode enumeration to filter functions.
 *
 * Each entry corresponds to one filtering strategy for ADC signals.
 * The table allows selecting the active filter dynamically at runtime.
 *
 * @note The index must match ::eSjcAdcFilterMode_t.
 */
static void (*const filter_function_table[])(uint8_t) = {
    [ADC_FILTER_NONE]                 = filter_none,                 ///< No filtering
    [ADC_FILTER_EXPONENTIAL]          = filter_exponential,          ///< Exponential IIR per sample
    [ADC_FILTER_BOX]                  = filter_box,                  ///< Box (moving average) filter
    [ADC_FILTER_EXP_THEN_BOX_PER_CYCLE] = filter_exp_then_box_per_cycle, ///< Exp per sample → per-cycle average → Box
    [ADC_FILTER_BOX_THEN_EXP]         = filter_box_then_exp,         ///< Box followed by Exponential
    [ADC_FILTER_EXP_THEN_BOX]         = filter_exp_then_box          ///< Exponential followed by Box
};
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void AnalogInterfaces_init(void)
{
    xAnalogInterfaceSemprHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE,
                                                 BINARY_SEMPHR_INITIAL_COUNT,
                                                 &(osSemaphoreAttr_t){.name = "AnalogInterfaceSemaphore",
                                                                      .cb_mem = &AnalogInterfaceSemaphoreControlBlock,
                                                                      .cb_size = sizeof(AnalogInterfaceSemaphoreControlBlock)});

    if (xAnalogInterfaceSemprHandle == NULL)
    {
        DebugOutput_systemPrintf("\r\n[OS FAULT] Analog interface semaphore not created...");
    }
    else
    {
        osSemaphoreRelease(xAnalogInterfaceSemprHandle);
    }

    // Initialize filters for each channel
    for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
    {
    	float fs;
    	float fc;
    	uint32_t box_size;

    	switch (xAdcFilterMode) {
    	case ADC_FILTER_BOX:
    		box_size = ADC_SAMPLES_PER_BANK;
    		fs = ADC_DMA_CONVERSION_FREQ_HZ;
    		fc = ANALOG_MEASUREMENT_DEFAULT_FREQ;
    		break;
    	case ADC_FILTER_BOX_THEN_EXP:
    		box_size = ADC_SAMPLES_PER_PWM_CYCLE;
    		fs = ANALOG_MEASUREMENT_PWM_TIMER_FREQ;
    		fc = 200.0f;
    		break;
    	case ADC_FILTER_EXP_THEN_BOX:
    		box_size = ADC_SAMPLES_PER_BANK;
    		fs = ADC_DMA_CONVERSION_FREQ_HZ;
    		fc = 5.0e3f;
    		break;
    	case ADC_FILTER_EXP_THEN_BOX_PER_CYCLE:
    		box_size = (uint32_t)(ADC_SAMPLE_PERIOD_SEC / PWM_CYCLE_PERIOD_SEC);
    		fs = ADC_DMA_CONVERSION_FREQ_HZ;
    		fc = 5.0e3f;
//    		fs = 20e3;
//    		fc = 1.0e3f;
    		break;
    	default: // ADC_FILTER_EXPONENTIAL:
    		box_size = ADC_SAMPLES_PER_BANK;
    		fs = ADC_DMA_CONVERSION_FREQ_HZ;
    		fc = 5.0e3f;
    		break;
    	}

        ExponentialFilter_init(&channel_filters[ch].exp, fc, fs);
        BoxFilter_init(&channel_filters[ch].box, box_size);
    }

    AnalogInterfaces_setVoltageFilterConfig(ANALOG_MEASUREMENT_PWM_TIMER_FREQ, ANALOG_MEASUREMENT_DEFAULT_FREQ/5.0f);


    // Configure DMA for ADC2 (currents)
    LL_DMA_DisableStream(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM);
    LL_DMA_SetChannelSelection(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM, LL_DMA_CHANNEL_1);
    LL_DMA_SetDataTransferDirection(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM, LL_DMA_PRIORITY_VERYHIGH);
    LL_DMA_SetMode(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM, LL_DMA_MDATAALIGN_HALFWORD);
    LL_DMA_DisableFifoMode(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM);

    LL_DMA_SetPeriphAddress(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM, (uint32_t)&ADC2->DR);
    LL_DMA_SetMemoryAddress(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM, (uint32_t)adc_buf);
    LL_DMA_SetDataLength(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM, ADC_SAMPLES_PER_BANK * ADC_DMA_BANKS);
}

void AnalogInterfaces_enable(void)
{
    // Enable ADC1 (voltage) in injected mode ------------------------ //
    LL_ADC_Enable(SJC_ADC_VOLTAGE_INSTANCE);
    while (!LL_ADC_IsEnabled(SJC_ADC_VOLTAGE_INSTANCE)) {}
    LL_ADC_EnableIT_JEOS(SJC_ADC_VOLTAGE_INSTANCE);

    // Enable ADC2 (current) with DMA -------------------------------- //
    LL_ADC_Enable(SJC_ADC_CURRENT_INSTANCE);
    while (!LL_ADC_IsEnabled(SJC_ADC_CURRENT_INSTANCE)) {}
    LL_ADC_REG_SetDMATransfer(SJC_ADC_CURRENT_INSTANCE, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    LL_ADC_REG_SetTriggerSource(SJC_ADC_CURRENT_INSTANCE, LL_ADC_REG_TRIG_EXT_TIM4_CH4);
    LL_ADC_REG_StartConversionExtTrig(SJC_ADC_CURRENT_INSTANCE, LL_ADC_REG_TRIG_EXT_RISINGFALLING);


    // Configure DMA interrupts -------------------------------------- //
    LL_DMA_ClearFlag_TC2(SJC_ADC_DMA_INSTANCE);
    LL_DMA_ClearFlag_HT2(SJC_ADC_DMA_INSTANCE);
    LL_DMA_ClearFlag_TE2(SJC_ADC_DMA_INSTANCE);
    LL_DMA_EnableIT_TC(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM);
    LL_DMA_EnableIT_HT(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM);
    LL_DMA_EnableIT_TE(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM);
    LL_DMA_EnableStream(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM);

    // --- Enable TIM10 (measurement check timer) -------------------- //
    if (!LL_TIM_IsEnabledCounter(SJC_ADC_CHECK_TIMER_INSTANCE))
    {
    	LL_TIM_SetCounter(SJC_ADC_CHECK_TIMER_INSTANCE, 0);
    	LL_TIM_ClearFlag_UPDATE(SJC_ADC_CHECK_TIMER_INSTANCE);
    	LL_TIM_EnableCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
    }

    // --- Enable TIM4 (trigger timer for ADC2) ---------------------- //
    if (!LL_TIM_IsEnabledCounter(SJC_ADC_TRIG_TIMER_INSTANCE))
    {
    	LL_TIM_SetCounter(SJC_ADC_TRIG_TIMER_INSTANCE, 0);
    	LL_TIM_ClearFlag_CC4(SJC_ADC_TRIG_TIMER_INSTANCE);
//    	LL_TIM_EnableIT_CC4(SJC_ADC_TRIG_TIMER_INSTANCE);   // Enable CC4 interrupt
    	LL_TIM_CC_EnableChannel(SJC_ADC_TRIG_TIMER_INSTANCE, LL_TIM_CHANNEL_CH4);
    	LL_TIM_EnableCounter(SJC_ADC_TRIG_TIMER_INSTANCE);
    }

    ResetIsrServiceCounter();
}

void AnalogInterfaces_disable(void)
{
    // --- Disable TIM4 (trigger timer) ------------------------------ //
    if (LL_TIM_IsEnabledCounter(SJC_ADC_TRIG_TIMER_INSTANCE))
    {
        LL_TIM_SetCounter(SJC_ADC_TRIG_TIMER_INSTANCE, 0);
        LL_TIM_ClearFlag_CC4(SJC_ADC_TRIG_TIMER_INSTANCE);
//        LL_TIM_DisableIT_CC4(SJC_ADC_TRIG_TIMER_INSTANCE);
        LL_TIM_CC_DisableChannel(SJC_ADC_TRIG_TIMER_INSTANCE, LL_TIM_CHANNEL_CH4);
        LL_TIM_DisableCounter(SJC_ADC_TRIG_TIMER_INSTANCE);
    }

    // --- Disable TIM10 (measurement check timer) ------------------- //
    if (LL_TIM_IsEnabledCounter(SJC_ADC_CHECK_TIMER_INSTANCE))
    {
        LL_TIM_SetCounter(SJC_ADC_CHECK_TIMER_INSTANCE, 0);
        LL_TIM_ClearFlag_UPDATE(SJC_ADC_CHECK_TIMER_INSTANCE);
        LL_TIM_DisableCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
    }

    // --- Disable ADCs ---------------------------------------------- //
    LL_ADC_Disable(SJC_ADC_CURRENT_INSTANCE);
    LL_ADC_Disable(SJC_ADC_VOLTAGE_INSTANCE);

    // --- Disable DMA ----------------------------------------------- //
    LL_DMA_DisableStream(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM);
    LL_DMA_DisableIT_TE(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM);
    LL_DMA_DisableIT_HT(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM);
    LL_DMA_DisableIT_TC(SJC_ADC_DMA_INSTANCE, SJC_ADC_DMA_STREAM);
}

eSjcReturnType_t AnalogInterfaces_updateConversionValues(void)
{
    configASSERT(xAnalogInterfaceSemprHandle != NULL);

    if (osOK == osSemaphoreAcquire(xAnalogInterfaceSemprHandle,
                                   ANALOG_INTERFACE_MUTEX_CONVERSION_WAIT_TIME_MS))
    {
        // Timing check
        xMeanSampleTimeObj.AccumulatedTimerTicks += LL_TIM_GetCounter(SJC_ADC_CHECK_TIMER_INSTANCE);
        LL_TIM_SetCounter(SJC_ADC_CHECK_TIMER_INSTANCE, 0);
        xMeanSampleTimeObj.Samples++;

        ResetIsrServiceCounter();

        // Select DMA bank to process
        uint16_t *bank_ptr = (bank_ready == 0) ? &adc_buf[0] : &adc_buf[ADC_SAMPLES_PER_BANK];

        // Process current samples from ADC2
        for (uint32_t i = 0; i < ADC_SAMPLES_PER_BANK; i++)
        {
            raw[MOTOR_CURRENT_CONV] =
                (float)__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, bank_ptr[i], LL_ADC_RESOLUTION_12B) / 1000.0f;
            filter_function_table[xAdcFilterMode](MOTOR_CURRENT_CONV);
        }

        // Copy filtered values
        for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
        {
            xAdcMeasurementValueBuffer[ch] = xFilteredSampleFromIsr.channels[ch];
        }

        osSemaphoreRelease(xAnalogInterfaceSemprHandle);
        return SJC_RET_OK;
    }
    return SJC_RET_NOK;
}

eSjcReturnType_t AnalogInterfaces_getAnalogConversion(const eSjcAnalogSensorType_t sensor, float *value)
{
	configASSERT(xAnalogInterfaceSemprHandle != NULL);

	if (value == NULL) return SJC_RET_NOK;
	*value = ZERO_FLOAT;

	if (osOK == osSemaphoreAcquire(xAnalogInterfaceSemprHandle,
			ANALOG_INTERFACE_MUTEX_DEFAULT_WAIT_TIME_MS))
	{
		switch (sensor)
		{
		case SJC_CURRENT_SENSOR_STD:
			*value = (xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV] / AD8418_ANALOG_FRONTED_GAIN) - ANALOG_CURRENT_SENSOR_VREF;
			*value /= (AD8418_SHUNT_RESISTANCE * AD8418_GAIN);
			break;
		case SJC_CURRENT_SENSOR_AUX:
			*value = (xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV] / ACS712_ANALOG_FRONTED_GAIN) - ANALOG_CURRENT_SENSOR_VREF;
			*value /= ACS712_GAIN;
			break;
#if SJC_HW_VERSION_MAJOR > 1
		case SJC_CURRENT_SENSOR_INA240_16A:
			// I_adc_meas = (([min(Vadc) max(Vadc)]./ Av_input_3v3) - Vref) / (RshuntValues(2) * Av_INA240);
			*value = (xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV] / INA240_ANALOG_FRONTED_GAIN) - ANALOG_CURRENT_SENSOR_VREF;
			*value /= INA240_16A_SENSITIVITY;
			break;
		case SJC_CURRENT_SENSOR_INA240_25A:
			*value = (xAdcMeasurementValueBuffer[MOTOR_CURRENT_CONV] / INA240_ANALOG_FRONTED_GAIN) - ANALOG_CURRENT_SENSOR_VREF;
			*value /= INA240_25A_SENSITIVITY;
			break;
		case SJC_VREF_SENSOR:
			*value = xAdcMeasurementValueBuffer[VREF_INT_CONV];
			break;
#endif
		case SJC_VOLTAGE_SENSOR:
			*value = xAdcMeasurementValueBuffer[DRIVE_SUPPLY_VOLTAGE_CONV] * VOLTAGE_SENSOR_RESISTIVE_CONDITIONER_GAIN;
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

    if (Ts == NULL) return SJC_RET_NOK;

    if (osOK == osSemaphoreAcquire(xAnalogInterfaceSemprHandle,
                                   ANALOG_INTERFACE_MUTEX_DEFAULT_WAIT_TIME_MS))
    {
        if (xMeanSampleTimeObj.Samples == 0)
            xMeanSampleTimeObj.Samples = 1;

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

void AnalogInterfaces_setCurrentFilterConfig(const eSjcAdcFilterMode_t mode, const float fc)
{
    if (mode < ADC_FILTER_MODE_SIZE)
    {
        xAdcFilterMode = mode;
        float fs = ANALOG_MEASUREMENT_DEFAULT_FREQ;

        DebugOutput_systemPrintf("\r\n[SJC] Current filter mode: %u", mode);

        if ((xAdcFilterMode == ADC_FILTER_EXPONENTIAL) ||
            (xAdcFilterMode == ADC_FILTER_EXP_THEN_BOX) ||
            (xAdcFilterMode == ADC_FILTER_BOX_THEN_EXP))
        {
            // For current, use high-resolution timer (500 kHz)
            fs = ANALOG_MEASUREMENT_HIGH_RES_TIMER_FREQ;

            for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
            {
                ExponentialFilter_set_sampling_frequency(&channel_filters[ch].exp, fs);
                ExponentialFilter_set_cutoff(&channel_filters[ch].exp, fc);
            }

            DebugOutput_systemPrintf("\r\n[SJC] Current filter fs: %.2f Hz, fc: %.2f Hz", fs, fc);
        }
    }
    else
    {
        xAdcFilterMode = ADC_FILTER_NONE;
    }
}

void AnalogInterfaces_setVoltageFilterConfig(const float fs, const float fc)
{
	for (uint8_t ch = 0; ch < CONVERSION_ORDER_SIZE; ch++)
	{
		ExponentialFilter_set_sampling_frequency(&channel_filters[ch].exp, fs);
		ExponentialFilter_set_cutoff(&channel_filters[ch].exp, fc);
	}

	DebugOutput_systemPrintf("\r\n[SJC] Voltage filter fs: %.2f Hz, fc: %.2f Hz", fs, fc);
}

// ------------------------------------------------------------------- //

// ISRs -------------------------------------------------------------- //
// ------------------------------------------------------------------- //
void DMA2_Stream2_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (LL_DMA_IsActiveFlag_HT2(SJC_ADC_DMA_INSTANCE))
    {
        LL_DMA_ClearFlag_HT2(SJC_ADC_DMA_INSTANCE);
        IncIsrServiceCounter();
        bank_ready = 0;
        vTaskNotifyGiveFromISR(xAdcUpdateTaskHandle, &xHigherPriorityTaskWoken);
    }

    if (LL_DMA_IsActiveFlag_TC2(SJC_ADC_DMA_INSTANCE))
    {
        LL_DMA_ClearFlag_TC2(SJC_ADC_DMA_INSTANCE);
        IncIsrServiceCounter();
        bank_ready = 1;
        vTaskNotifyGiveFromISR(xAdcUpdateTaskHandle, &xHigherPriorityTaskWoken);
    }

    if (LL_DMA_IsActiveFlag_TE2(SJC_ADC_DMA_INSTANCE))
    {
        LL_DMA_ClearFlag_TE2(SJC_ADC_DMA_INSTANCE);
        DebugOutput_systemPrintf("\r\n[HW FAULT] ADC DMA Transfer Error!");
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ADC_IRQHandler(void)
{
    if (LL_ADC_IsActiveFlag_JEOS(SJC_ADC_VOLTAGE_INSTANCE))
    {
        LL_ADC_ClearFlag_JEOS(SJC_ADC_VOLTAGE_INSTANCE);

    	raw[VREF_INT_CONV] = (float)__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_INJ_ReadConversionData12(SJC_ADC_VOLTAGE_INSTANCE, LL_ADC_INJ_RANK_1), LL_ADC_RESOLUTION_12B) / 1000.0f;
    	raw[VREF_INT_CONV] += (float)__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_INJ_ReadConversionData12(SJC_ADC_VOLTAGE_INSTANCE, LL_ADC_INJ_RANK_2), LL_ADC_RESOLUTION_12B) / 1000.0f;

    	raw[DRIVE_SUPPLY_VOLTAGE_CONV] = (float)__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_INJ_ReadConversionData12(SJC_ADC_VOLTAGE_INSTANCE, LL_ADC_INJ_RANK_3), LL_ADC_RESOLUTION_12B) / 1000.0f;
    	raw[DRIVE_SUPPLY_VOLTAGE_CONV] += (float)__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, LL_ADC_INJ_ReadConversionData12(SJC_ADC_VOLTAGE_INSTANCE, LL_ADC_INJ_RANK_4), LL_ADC_RESOLUTION_12B) / 1000.0f;

    	xFilteredSampleFromIsr.channels[VREF_INT_CONV] = ExponentialFilter_update(&channel_filters[VREF_INT_CONV].exp, raw[VREF_INT_CONV]/2.0f);
    	xFilteredSampleFromIsr.channels[DRIVE_SUPPLY_VOLTAGE_CONV] = ExponentialFilter_update(&channel_filters[DRIVE_SUPPLY_VOLTAGE_CONV].exp, raw[DRIVE_SUPPLY_VOLTAGE_CONV]/2.0f);
    }

    if (LL_ADC_IsActiveFlag_OVR(SJC_ADC_VOLTAGE_INSTANCE))
    {
        LL_ADC_ClearFlag_OVR(SJC_ADC_VOLTAGE_INSTANCE);
        Error_Handler();
    }
}

//void TIM4_IRQHandler(void)
//{
//    if (LL_TIM_IsActiveFlag_CC4(SJC_ADC_TRIG_TIMER_INSTANCE))
//    {
//        LL_TIM_ClearFlag_CC4(SJC_ADC_TRIG_TIMER_INSTANCE);
//
//        // Optionally: trigger ADC by SW if configured that way
//        LL_ADC_REG_StartConversionSWStart(SJC_ADC_CURRENT_INSTANCE);
//    }
//}
// ------------------------------------------------------------------- //

// Private function implementations ---------------------------------- //
// ------------------------------------------------------------------- //
static void filter_none(uint8_t ch)
{
    xFilteredSampleFromIsr.channels[ch] = raw[ch];
}

static void filter_exponential(uint8_t ch)
{
    xFilteredSampleFromIsr.channels[ch] =
        ExponentialFilter_update(&channel_filters[ch].exp, raw[ch]);
}

static void filter_box(uint8_t ch)
{
    BoxFilter_add_sample(&channel_filters[ch].box, raw[ch]);
    if (BoxFilter_ready(&channel_filters[ch].box))
        xFilteredSampleFromIsr.channels[ch] =
            BoxFilter_get_average_and_reset(&channel_filters[ch].box);
}

static void filter_exp_then_box_per_cycle(uint8_t ch)
{
    // 1. Exponential smoothing of each raw sample
    float exp_val = ExponentialFilter_update(&channel_filters[ch].exp, raw[ch]);

    // 2. Accumulate inside the current PWM cycle
    static float cycle_accum[CONVERSION_ORDER_SIZE] = {0};
    static uint16_t cycle_count[CONVERSION_ORDER_SIZE] = {0};

    cycle_accum[ch] += exp_val;
    cycle_count[ch]++;

    // 3. End of PWM cycle? (25 samples @ 20 kHz = 50 µs)
    if (cycle_count[ch] >= ADC_SAMPLES_PER_PWM_CYCLE)
    {
        // Average value for this PWM cycle
        float cycle_avg = cycle_accum[ch] / (float)ADC_SAMPLES_PER_PWM_CYCLE;

        // Reset accumulators
        cycle_accum[ch] = 0.0f;
        cycle_count[ch] = 0;

        // 4. Feed the cycle average into BoxFilter (20 values ≈ 1 ms window)
        BoxFilter_add_sample(&channel_filters[ch].box, cycle_avg);

        if (BoxFilter_ready(&channel_filters[ch].box))
        {
            xFilteredSampleFromIsr.channels[ch] = BoxFilter_get_average_and_reset(&channel_filters[ch].box);
        }
    }
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

static void filter_exp_then_box(uint8_t ch)
{
    float exp_val = ExponentialFilter_update(&channel_filters[ch].exp, raw[ch]);
    BoxFilter_add_sample(&channel_filters[ch].box, exp_val);
    if (BoxFilter_ready(&channel_filters[ch].box))
    {
    	xFilteredSampleFromIsr.channels[ch] = BoxFilter_get_average_and_reset(&channel_filters[ch].box);
    	ExponentialFilter_force_set(&channel_filters[ch].exp, xFilteredSampleFromIsr.channels[ch]);
    }
}
// ------------------------------------------------------------------- //
