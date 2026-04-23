/*
 * pwm_signal.h
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  PWM signal control interface for actuator drive module.
 *  Provides configuration, control, and access to PWM-based signal generation,
 *  including duty cycle management and frequency selection.
 */

#ifndef SJC__MODULES__PWM_SIGNAL_H
#define SJC__MODULES__PWM_SIGNAL_H

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initializes the PWM module with a given base frequency.
     *
     * @param BaseFrequency Desired base frequency enum (e.g. 10 kHz, 20 kHz).
     */
    void PwmSignal_init(const eSjcPwmBaseFreq_t BaseFrequency);

    /**
     * @brief Enables the PWM output channels and starts the timer.
     */
    void PwmSignal_enable(void);

    /**
     * @brief Disables the PWM output channels and stops the timer.
     */
    void PwmSignal_disable(void);

    /**
     * @brief Retrieves the current duty cycle (0.0 to 1.0).
     *
     * @return Current duty cycle.
     */
    float PwmSignal_getDutyCycle(void);

    /**
     * @brief Returns the minimum allowed duty cycle (typically 0.0).
     *
     * @return Minimum duty cycle.
     */
    float PwmSignal_getMinDutyCycle(void);

    /**
     * @brief Returns the maximum allowed duty cycle (typically 1.0).
     *
     * @return Maximum duty cycle.
     */
    float PwmSignal_getMaxDutyCycle(void);

    /**
     * @brief Sets the duty cycle of the PWM signal.
     *
     * @param duty Desired duty cycle (clamped between min and max).
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise.
     */
    eSjcReturnType_t PwmSignal_setDutyCycle(const float duty);

    /**
     * @brief Retrieves the current configured PWM frequency in Hz.
     *
     * @return Frequency in Hz.
     */
    uint32_t PwmSignal_getFrequency(void);

    /**
     * @brief Sets the base frequency of the PWM using predefined options.
     *
     * @param BaseFrequency Enum corresponding to the target frequency.
     * @return Final frequency in Hz after configuration.
     */
    uint32_t PwmSignal_setFrequency(const eSjcPwmBaseFreq_t BaseFrequency);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__PWM_SIGNAL_H */
