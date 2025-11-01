/**
 * @file lqr_controller.h
 * @brief Discrete-time LQR/LQI controller with integral action, anti-windup and error filtering.
 *
 * This module provides a discrete-time Linear Quadratic Regulator (LQR),
 * extended with:
 *  - Integral action (LQI)
 *  - Anti-windup via back-calculation
 *  - Exponential filtering of the error signal
 *
 * Supported configurations:
 *  - Order = 1 → single state + integral action
 *  - Order = 2 → two states + integral action, with optional observer feedback
 *
 * Author: Tiago
 * Date: Apr 23, 2025
 */

#ifndef SJC__MODULES__LQR_CONTROLLER_H_
#define SJC__MODULES__LQR_CONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "system_types/include/data_structures.h"
#include "modules/include/lqr_observer.h"

/**
 * @brief Initialize the LQR/LQI controller.
 *
 * - Resets controller states, integrator, and telemetry variables.
 * - Configures and resets the exponential error filter.
 * - Initializes the observer (active only if order = 2 and enabled).
 */
void LQRController_init(void);

/**
 * @brief Compute the control effort based on reference and measurement.
 *
 * Control law:
 *   u = -Kx·x - Ki·∫e
 *
 * - For order = 1: uses internal model propagation.
 * - For order = 2: uses observer states if enabled, otherwise direct model.
 * - Applies saturation, error filtering, and anti-windup correction.
 *
 * @param ref    Desired reference value.
 * @param y_meas Measured process variable (e.g., current [A]).
 * @return Saturated control effort (within configured limits).
 */
float LQRController_compute(float ref, float y_meas);

/**
 * @brief Get the most recent filtered error signal.
 *
 * @return Current error value (reference - measurement).
 */
float LQRController_get_error(void);

/**
 * @brief Get the most recent reference value.
 *
 * @return Last applied reference.
 */
float LQRController_get_reference(void);

/**
 * @brief Reset the controller state.
 *
 * - Clears state variables, integrator, and telemetry.
 * - Resets the exponential error filter.
 */
void LQRController_reset(void);

/**
 * @brief Get current controller and observer parameters.
 *
 * @param control_params  Pointer to store controller parameters.
 * @param observer_params Pointer to store observer parameters.
 */
void LQRController_get_parameters(xSjcLQRControllerParameters_t *control_params,
                                  xSjcLQRObserverParameters_t *observer_params);

/**
 * @brief Set controller and observer parameters.
 *
 * - Copies parameter structures by value.
 * - Updates error filter cutoff and resets the filter.
 *
 * @param control_params  Controller parameters.
 * @param observer_params Observer parameters.
 */
void LQRController_set_parameters(const xSjcLQRControllerParameters_t control_params,
                                  const xSjcLQRObserverParameters_t observer_params);

/**
 * @brief Configure output saturation limits.
 *
 * @param u_min Minimum allowed control effort.
 * @param u_max Maximum allowed control effort.
 */
void LQRController_setSaturation(float u_min, float u_max);

/**
 * @brief Check whether the controller has been initialized.
 *
 * @return true if initialized, false otherwise.
 */
bool LQRController_isInitialized(void);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__LQR_CONTROLLER_H_ */
