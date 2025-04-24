/*
 * analog_interfaces.h
 *
 *  Created on: Sep 9, 2023
 *  Updated on: Mar 2025
 *      Author: tiago
 */

#ifndef SJC__MODULES__ANALOG_INTERFACES_H_
#define SJC__MODULES__ANALOG_INTERFACES_H_

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Enumeration for selecting the ADC filtering mode.
     */
    typedef enum
    {
        ADC_FILTER_NONE = 0x00,         ///< No filtering, raw values used directly
        ADC_FILTER_EXPONENTIAL = 0x01,  ///< Exponential filtering on each sample
        ADC_FILTER_BOX = 0x02,          ///< Box filtering over a window
        ADC_FILTER_EXP_THEN_BOX = 0x03, ///< Exponential filter then box filter
        ADC_FILTER_BOX_THEN_EXP = 0x04, ///< Box filter then exponential filter
        ADC_FILTER_MODE_SIZE
    } eSjcAdcFilterMode_t;

    /**
     * @brief Initializes the analog interface module and internal filter states.
     */
    void AnalogInterfaces_init(void);

    /**
     * @brief Enables the ADC peripheral and its interrupt sources.
     */
    void AnalogInterfaces_enable(void);

    /**
     * @brief Disables the ADC peripheral and its interrupt sources.
     */
    void AnalogInterfaces_disable(void);

    /**
     * @brief Updates the filtered measurement buffer from the last ISR result.
     * @return SJC_RET_OK on success, SJC_RET_NOK on failure.
     */
    eSjcReturnType_t AnalogInterfaces_updateConversionValues(void);

    /**
     * @brief Retrieves mean conversion delay information.
     * @param[out] Ts Pointer to sample timing structure to receive stats.
     * @return SJC_RET_OK if data is available, otherwise SJC_RET_NOK.
     */
    eSjcReturnType_t AnalogInterfaces_getMeanConversionDelay(xSjcMeanSampleTimeMeasurement_t *Ts);

    /**
     * @brief Gets the filtered analog conversion value for a specific sensor.
     * @param[in] sensor The sensor type to query.
     * @param[out] value Pointer to store the converted result.
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise.
     */
    eSjcReturnType_t AnalogInterfaces_getAnalogConversion(const eSjcAnalogSensorType_t sensor, float *value);

    /**
     * @brief ISR handler to process ADC1 injected conversion results.
     *        Must be called from ADC_IRQHandler when JEOS is triggered.
     */
    void SJC_AnalogInterfaces_Adc1ConversionIsr(void);

    /**
         * @brief ISR handler to process ADC2 injected conversion results.
         *        Must be called from ADC_IRQHandler when JEOS is triggered.
         */
    void SJC_AnalogInterfaces_Adc2ConversionIsr(void);

    /**
     * @brief Registers a task handle that should be notified from the ISR
     *        when a complete set of filtered data is available.
     * @param[in] handle Pointer to the task handle.
     */
    void AnalogInterfaces_registerUpdateTaskHandle(TaskHandle_t *handle);

    /**
     * @brief Sets the ADC filtering mode dynamically at runtime.
     * @param[in] mode Filtering mode to be applied.
     * @param[in] fc Cutoff frequency in Hz.
     */
    void AnalogInterfaces_setFilterMode(const eSjcAdcFilterMode_t mode, const float fc);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__ANALOG_INTERFACES_H_ */
