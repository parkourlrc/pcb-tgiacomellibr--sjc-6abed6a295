/*
 * discrete_controller.h
 *
 *  Created on: Apr 11, 2025
 *      Author: Tiago Giacomelli
 *
 *  Discrete controller module header. This module implements a second-order
 *  discrete-time controller using Direct Form II, with output saturation,
 *  anti-windup (back-calculation), and optional exponential filtering on the
 *  error signal. It is designed for real-time embedded control applications
 *  where deterministic execution and bounded memory are required.
 */

#ifndef SJC__MODULES__DISCRETE_CONTROLLER_H
#define SJC__MODULES__DISCRETE_CONTROLLER_H

#include "system_types/include/data_structures.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initializes the discrete controller and its internal filter.
     *
     * Clears errors, control action history, parameters, and filter states.
     * Must be called before using compute().
     */
    void DiscreteController_init(void);

    /**
     * @brief Computes the next control effort based on reference and measurement.
     *
     * Implements a Direct Form II second-order difference equation:
     * @f[
     *     Cd(z) = \frac{B_0 + B_1 z^{-1} + B_2 z^{-2}}{1 + A_1 z^{-1} + A_2 z^{-2}}
     * @f]
     *
     * Includes output saturation, anti-windup, and optional exponential error filtering.
     *
     * @param reference   Desired setpoint value.
     * @param measurement Current measured process variable.
     * @return Saturated control output.
     */
    float DiscreteController_compute(float reference, float measurement);

    /**
     * @brief Retrieves the most recent control output (saturated).
     *
     * @return Last computed saturated control value.
     */
    float DiscreteController_getOutput(void);

    /**
     * @brief Retrieves the most recent error value.
     *
     * @return Last computed error (reference - measurement).
     */
    float DiscreteController_get_error(void);

    /**
     * @brief Retrieves the most recent reference value.
     *
     * @return Last commanded reference.
     */
    float DiscreteController_get_reference(void);

    /**
     * @brief Resets the controller state.
     *
     * Clears past errors, control output history, and filter state.
     * Should be called after faults or reinitialization.
     */
    void DiscreteController_reset(void);

    /**
     * @brief Configures output saturation limits.
     *
     * @param min Minimum allowed output.
     * @param max Maximum allowed output.
     */
    void DiscreteController_setSaturation(float min, float max);

    /**
     * @brief Retrieves currently configured output saturation limits.
     *
     * @param min Pointer to store minimum limit (can be NULL).
     * @param max Pointer to store maximum limit (can be NULL).
     */
    void DiscreteController_getOutputLimits(float *min, float *max);

    /**
     * @brief Sets the anti-windup gain used in back-calculation.
     *
     * @param k_aw Anti-windup gain (positive scalar).
     */
    void DiscreteController_setAntiWindupGain(float k_aw);

    /**
     * @brief Sets the cutoff frequency for the exponential filter applied to the error signal.
     *
     * @param cutoff_freq Cutoff frequency in Hz.
     */
    void DiscreteController_setFilterCutoffFrequency(float cutoff_freq);

    /**
     * @brief Retrieves current controller parameters (coefficients and limits).
     *
     * @param params Pointer to parameter struct (output).
     */
    void DiscreteController_get_parameters(xSjcDiscreteControllerParameters_t *params);

    /**
     * @brief Dynamically updates controller parameters.
     *
     * Copies coefficients, saturation limits, anti-windup gain, and filter settings.
     * Safe to call at runtime. Parameters are passed by value for encapsulation.
     *
     * @param params Parameter struct (copied by value).
     */
    void DiscreteController_set_parameters(const xSjcDiscreteControllerParameters_t params);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__DISCRETE_CONTROLLER_H */
