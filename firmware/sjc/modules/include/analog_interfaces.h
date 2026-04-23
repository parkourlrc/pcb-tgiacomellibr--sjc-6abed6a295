/*
 * analog_interfaces.h
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  Analog measurement interface for current, voltage, and VrefInt sensing.
 *  Handles ADC1 (voltage, injected mode) and ADC2 (current, DMA mode) with
 *  configurable filtering (exponential, box, combined). Provides thread-safe
 *  measurement access via mutex-protected API.
 */

#ifndef SJC__MODULES__ANALOG_INTERFACES_H_
#define SJC__MODULES__ANALOG_INTERFACES_H_

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enumeration of available ADC filter modes.
 *
 * Defines the possible filtering strategies for analog signal conditioning.
 */
typedef enum
{
    ADC_FILTER_NONE = 0x00,                   ///< No filtering, raw ADC values are used directly
    ADC_FILTER_EXPONENTIAL = 0x01,            ///< Exponential IIR filtering applied to each sample
    ADC_FILTER_BOX = 0x02,                    ///< Batch average (box) filter across a window
    ADC_FILTER_EXP_THEN_BOX_PER_CYCLE = 0x03, ///< Exponential filter on each sample (≈2 µs),
                                              ///< then averaging per PWM cycle (25 samples),
                                              ///< then Box filter on cycle averages (20 cycles ≈ 1 ms)
    ADC_FILTER_BOX_THEN_EXP = 0x04,           ///< Box filter applied first, followed by exponential filter
    ADC_FILTER_EXP_THEN_BOX = 0x05,           ///< Exponential filter applied first, followed by Box filter
    ADC_FILTER_MODE_SIZE                      ///< Number of available filter modes
} eSjcAdcFilterMode_t;

/**
 * @brief Initialize the analog interface module and reset filter states.
 */
void AnalogInterfaces_init(void);

/**
 * @brief Enable the ADC peripherals and their interrupts.
 */
void AnalogInterfaces_enable(void);

/**
 * @brief Disable the ADC peripherals and their interrupts.
 */
void AnalogInterfaces_disable(void);

/**
 * @brief Update the filtered measurement buffer using the last ISR results.
 *
 * This function should be called from the application task context.
 *
 * @return SJC_RET_OK if values were updated successfully,
 *         SJC_RET_NOK otherwise.
 */
eSjcReturnType_t AnalogInterfaces_updateConversionValues(void);

/**
 * @brief Retrieve mean ADC conversion delay statistics.
 *
 * @param[out] Ts Pointer to a measurement structure that will be filled with
 *                the computed mean sample timing data.
 *
 * @return SJC_RET_OK if data is valid,
 *         SJC_RET_NOK otherwise.
 */
eSjcReturnType_t AnalogInterfaces_getMeanConversionDelay(
		xSjcMeanSampleTimeMeasurement_t *Ts);

/**
 * @brief Get the filtered analog conversion result for a specific sensor.
 *
 * @param[in]  sensor The analog sensor to read.
 * @param[out] value  Pointer to store the retrieved floating-point result.
 *
 * @return SJC_RET_OK if successful,
 *         SJC_RET_NOK otherwise.
 */
eSjcReturnType_t AnalogInterfaces_getAnalogConversion(
		const eSjcAnalogSensorType_t sensor, float *value);

/**
 * @brief Get multiple filtered analog conversions in a single mutex-protected call.
 *
 * @param[in]  sensors Array of sensor types to read.
 * @param[out] values  Array to store the results (same order as sensors).
 * @param[in]  count   Number of sensors to read.
 *
 * @return SJC_RET_OK if successful,
 *         SJC_RET_NOK otherwise.
 */
eSjcReturnType_t AnalogInterfaces_getMultipleConversions(
		const eSjcAnalogSensorType_t sensors[], float values[], uint8_t count);

/**
 * @brief Register a task handle to be notified from the ISR.
 *
 * When a full set of filtered ADC data is available, the ISR will notify
 * the registered task.
 *
 * @param[in] handle Pointer to the task handle (FreeRTOS).
 */
void AnalogInterfaces_registerUpdateTaskHandle(TaskHandle_t *handle);

/**
 * @brief Configure the filter for current measurements at runtime.
 *
 * @param[in] mode Filter mode to apply.
 * @param[in] fc   Cutoff frequency in Hz.
 */
void AnalogInterfaces_setCurrentFilterConfig(const eSjcAdcFilterMode_t mode, const float fc);

/**
 * @brief Configure the voltage filter parameters at runtime.
 *
 * Voltage channels use a fixed filter type. This function adjusts only
 * the cutoff and sampling frequencies.
 *
 * @param[in] fs Sampling frequency in Hz.
 * @param[in] fc Cutoff frequency in Hz.
 */
void AnalogInterfaces_setVoltageFilterConfig(const float fs, const float fc);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__ANALOG_INTERFACES_H_ */
