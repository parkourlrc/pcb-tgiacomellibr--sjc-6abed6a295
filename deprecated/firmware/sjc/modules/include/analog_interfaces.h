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
     * @brief Initializes the analog interface module and filters.
     */
    void AnalogInterfaces_init(void);

    /**
     * @brief Enables ADC and related interrupts.
     */
    void AnalogInterfaces_enable(void);

    /**
     * @brief Disables ADC and related interrupts.
     */
    void AnalogInterfaces_disable(void);

    /**
     * @brief Updates the filtered conversion values from the ISR results.
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise.
     */
    eSjcReturnType_t AnalogInterfaces_updateConversionValues(void);

    /**
     * @brief Gets the mean ADC sampling time information.
     * @param[out] Ts Struct containing accumulated ticks and sample count.
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise.
     */
    eSjcReturnType_t AnalogInterfaces_getMeanConversionDelay(xSjcMeanSampleTimeMeasurement_t *Ts);

    /**
     * @brief Retrieves the processed analog value for the selected sensor.
     * @param[in] sensor Selected analog sensor.
     * @param[out] value Filtered and converted value.
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise.
     */
    eSjcReturnType_t AnalogInterfaces_getAnalogConversion(const eSjcAnalogSensorType_t sensor, float *value);

    /**
     * @brief ADC injected conversion ISR handler. Should be called from ADC_IRQHandler().
     */
    void SJC_AnalogInterfaces_AdcConversionIsr(void);

    /**
     * @brief Registers a task handle to be notified when filtered ADC data is ready.
     * @param handle Pointer to the task handle to notify.
     */
    void AnalogInterfaces_registerUpdateTaskHandle(TaskHandle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__ANALOG_INTERFACES_H_ */
