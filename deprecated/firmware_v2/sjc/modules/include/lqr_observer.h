/**
 * @file lqr_observer.h
 * @brief LQR observer module for state estimation.
 *
 * Provides a discrete-time observer for current control loops.
 * Supports up to second-order models (2 states: current and derivative).
 *
 * Features:
 *  - Configurable observer matrices (Ao, Bo, Co, L)
 *  - Optional cutoff frequency parameter
 *  - Provides estimated current and current derivative
 *  - Reset and reconfiguration support
 *
 * Designed for real-time embedded control applications.
 */

#ifndef SJC__MODULES__LQR_OBSERVER_H_
#define SJC__MODULES__LQR_OBSERVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "system_types/include/data_structures.h"
#include <stdbool.h>

/**
 * @brief Initializes the LQR observer.
 *
 * Clears state estimates and loads default parameters.
 */
void LQRObserver_init(void);

/**
 * @brief Updates the internal state estimate using input and measured output.
 *
 * Observer update equation:
 *   x_hat[k+1] = Ao * x_hat[k] + Bo * u[k] + L * (y_meas[k] - Co * x_hat[k])
 *
 * @param u      Control input [V].
 * @param y_meas Measured output (current) [A].
 */
void LQRObserver_update(float u, float y_meas);

/**
 * @brief Retrieves the current observer parameters.
 *
 * @param params Pointer to store observer parameter struct.
 */
void LQRObserver_getParameters(xSjcLQRObserverParameters_t *params);

/**
 * @brief Sets new observer parameters.
 *
 * @param params Observer parameter struct (copied by value).
 */
void LQRObserver_setParameters(const xSjcLQRObserverParameters_t params);

/**
 * @brief Returns the estimated current.
 *
 * @return Estimated current [A] (x_hat[0]).
 */
float LQRObserver_get_state(void);

/**
 * @brief Returns the estimated current derivative.
 *
 * @return Estimated derivative [A/s] (x_hat[1]).
 */
float LQRObserver_get_state_derivative(void);

/**
 * @brief Resets the observer.
 *
 * Clears internal states and parameters.
 * Should be called after fault recovery or system reinitialization.
 */
void LQRObserver_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__LQR_OBSERVER_H_ */
