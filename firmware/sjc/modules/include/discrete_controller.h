/**
 * @file controller.h
 * @brief Discrete controller module header.
 *
 * This module implements a second-order discrete-time controller using Direct Form II,
 * with output saturation, anti-windup (back-calculation), and optional exponential filtering.
 *
 * Designed for real-time embedded control applications.
 *
 * @author Tiago
 * @date Apr 11, 2025
 */

#ifndef SJC__MODULES__DISCRETE_CONTROLLER_H
#define SJC__MODULES__DISCRETE_CONTROLLER_H

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initializes the discrete controller and its internal filter.
     */
    void DiscreteController_init(void);

    /**
     * @brief Computes the next control effort based on reference and measurement.
     *
     * @param reference Desired target value.
     * @param measurement Current measured value.
     * @return Saturated control output.
     */
    float DiscreteController_compute(const float reference, const float measurement);

    /**
     * @brief Retrieves the latest control output.
     *
     * @return Last saturated control value.
     */
    float DiscreteController_getOutput(void);

    /**
     * @brief Resets internal controller states (errors and output history).
     */
    void DiscreteController_reset(void);

    /**
     * @brief Configures output saturation limits.
     *
     * @param min Minimum output allowed.
     * @param max Maximum output allowed.
     */
    void DiscreteController_setOutputLimits(float min, float max);

    /**
     * @brief Retrieves currently configured output saturation limits.
     *
     * @param min Pointer to receive minimum limit (can be NULL).
     * @param max Pointer to receive maximum limit (can be NULL).
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
     * @brief (Optional) Dynamically updates controller coefficients if runtime reconfiguration is supported.
     */
    void DiscreteController_updateParameters(void);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__DISCRETE_CONTROLLER_H */
