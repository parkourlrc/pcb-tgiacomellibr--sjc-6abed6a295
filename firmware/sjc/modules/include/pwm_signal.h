/*
 * pwm_signal.h
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago
 */

#ifndef SJC__MODULES__PWM_SIGNAL_H
#define SJC__MODULES__PWM_SIGNAL_H

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initializes the PWM signal with a given base frequency.
     */
    void PwmSignal_init(const eSjcPwmBaseFreq_t BaseFrequency);

    /**
     * @brief Enables the PWM signal generation.
     */
    void PwmSignal_enable(void);

    /**
     * @brief Disables the PWM signal generation.
     */
    void PwmSignal_disable(void);

    /**
     * @brief Returns the current duty cycle (0.0 - 1.0).
     */
    float PwmSignal_getDutyCycle(void);

    /**
     * @brief Returns the minimum allowed duty cycle.
     */
    float PwmSignal_getMinDutyCycle(void);

    /**
     * @brief Returns the maximum allowed duty cycle.
     */
    float PwmSignal_getMaxDutyCycle(void);

    /**
     * @brief Sets the duty cycle for the PWM signal.
     * @param duty Normalized duty cycle (0.0 to 1.0).
     * @return SJC_RET_OK if successful.
     */
    eSjcReturnType_t PwmSignal_setDutyCycle(const float duty);

    /**
     * @brief Returns the current PWM frequency in Hz.
     */
    uint32_t PwmSignal_getFrequency(void);

    /**
     * @brief Sets the PWM frequency.
     * @param BaseFrequency Frequency enum.
     * @return Actual frequency in Hz.
     */
    uint32_t PwmSignal_setFrequency(const eSjcPwmBaseFreq_t BaseFrequency);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__PWM_SIGNAL_H */
