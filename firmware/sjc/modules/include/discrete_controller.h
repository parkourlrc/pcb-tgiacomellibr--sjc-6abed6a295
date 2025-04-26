/**
 * @file controller.h
 * @brief Discrete controller module header.
 *
 * This module implements a second-order discrete-time controller with configurable output limits.
 * Designed for real-time embedded control applications.
 *
 * @author Tiago
 * @date Apr 11, 2025
 */

#ifndef SJC__MODULES__DISCRETE_CONTROLLER_H
#define SJC__MODULES__DISCRETE_CONTROLLER_H

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the internal state of the discrete controller.
 */
void DiscreteController_init(void);

/**
 * @brief Computes the controller output based on reference and measurement.
 *
 * @param reference Desired setpoint.
 * @param measurement Measured process variable.
 * @return Control effort (output) limited by configured bounds.
 */
float DiscreteController_compute(const float reference, const float measurement);

/**
 * @brief Retrieves the most recent controller output.
 *
 * @return Last control output.
 */
float DiscreteController_getOutput(void);

/**
 * @brief Resets the internal state of the controller (errors and output history).
 */
void DiscreteController_reset(void);

/**
 * @brief Sets the minimum and maximum output limits.
 *
 * @param min Minimum output value.
 * @param max Maximum output value.
 */
void DiscreteController_setOutputLimits(float min, float max);

/**
 * @brief Retrieves the currently configured output limits.
 *
 * @param min Pointer to store minimum output limit.
 * @param max Pointer to store maximum output limit.
 */
void DiscreteController_getOutputLimits(float *min, float *max);

void DiscreteController_setAntiWindupGain(float k_aw);

void DiscreteController_setFilterCutoffFrequency(float cutoff_freq);
/**
 * @brief Updates the controller parameters dynamically (optional).
 */
void DiscreteController_updateParameters(void);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__DISCRETE_CONTROLLER_H */
