/**
 * @file pid_controller.h
 * @brief Discrete PID controller with conditional integration (anti-windup).
 *
 * This module implements a classical discrete-time PID controller with:
 *  - Proportional, Integral, and Derivative terms
 *  - Conditional integration to mitigate integrator windup
 *  - Configurable actuator effort limits (saturation)
 *
 * Features:
 *  - Designed for real-time embedded control (STM32, FreeRTOS, etc.)
 *  - Maintains an internal controller state (singleton pattern)
 *  - Provides API functions for initialization, computation, parameter tuning,
 *    saturation limits, and reset of the internal state
 */

#ifndef SJC__MODULES__PID_CONTROLLER_H
#define SJC__MODULES__PID_CONTROLLER_H

#include "system_types/include/data_structures.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // ---------------------------------------------------------------------------
    // Public API
    // ---------------------------------------------------------------------------

    /**
     * @brief Initialize the PID controller.
     *
     * Clears error history, resets reference and control effort,
     * and sets default parameters and saturation limits to zero.
     *
     * Must be called once before using the controller.
     */
    void PidController_init(void);

    /**
     * @brief Compute the PID control effort for the current step.
     *
     * Implements:
     *   u[k] = Kp * e[k] + Kd * (e[k] - e[k-1]) / dt + Ki * ∫ e[k] dt
     *
     * - The integral term is conditionally updated (anti-windup):
     *   the integral accumulator is only advanced if the tentative
     *   control effort lies within the configured saturation limits.
     * - The final control effort is always clamped to the configured limits.
     *
     * @param reference   Desired setpoint.
     * @param measurement Current measured process variable.
     * @param dt          Sampling period in seconds (must be > 0 and finite).
     * @return Saturated control effort (actuator command).
     */
    float PidController_compute(const float reference,
                                const float measurement,
                                const float dt);

    /**
     * @brief Get the most recent control error.
     *
     * @return Last computed error value (reference - measurement).
     */
    float PidController_get_error(void);

    /**
     * @brief Get the actuator effort saturation limits.
     *
     * @return Effort range {Min, Max}.
     */
    xSjcRange_t PidController_get_saturation(void);

    /**
     * @brief Set the actuator effort saturation limits.
     *
     * The output of @ref PidController_compute will be clamped
     * within these bounds.
     *
     * @param limits Range struct with Min and Max values.
     */
    void PidController_set_saturation(const xSjcRange_t limits);

    /**
     * @brief Get the current PID parameters.
     *
     * @return Current parameter struct (Kp, Ki, Kd).
     */
    xSjcPidParameters_t PidController_get_parameters(void);

    /**
     * @brief Copy the current PID parameters into a user-provided struct.
     *
     * @param[out] parameters Pointer to destination struct.
     */
    void PidController_copy_parameters(xSjcPidParameters_t *parameters);

    /**
     * @brief Set the PID controller parameters.
     *
     * Overwrites the internal parameter struct with the given values.
     *
     * @param parameters Parameter struct (copied by value).
     */
    void PidController_set_parameters(const xSjcPidParameters_t parameters);

    /**
     * @brief Reset the PID controller state.
     *
     * Clears error history, integral accumulator, reference,
     * and control effort. Parameters and saturation limits remain unchanged.
     */
    void PidController_reset(void);

    // ---------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__PID_CONTROLLER_H */
