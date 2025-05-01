/**
 * @file pid_controller.h
 * @brief PID controller module with anti-windup (conditional integration).
 */

#ifndef SJC__MODULES__PID_CONTROLLER_H
#define SJC__MODULES__PID_CONTROLLER_H

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initializes the internal PID controller state.
     */
    void PidController_init(void);

    /**
     * @brief Computes the PID control action based on the error and time step.
     *
     * @param ControllerError Error signal.
     * @param dt Time step (s).
     * @return Control effort.
     */
    float PidController_compute(const float ControllerError, const float dt);

    /**
     * @brief Gets the most recent controller error.
     */
    float PidController_getError(void);

    /**
     * @brief Gets the current reference setpoint.
     */
    float PidController_getReference(void);

    /**
     * @brief Sets the reference setpoint for the controller.
     */
    void PidController_setReference(const float reference);

    /**
     * @brief Gets the configured actuator effort (output) limits.
     */
    xSjcRange_t PidController_getActuatorEffortLimits(void);

    /**
     * @brief Sets the actuator effort limits.
     */
    void PidController_setActuatorEffortLimits(const xSjcRange_t limits);

    /**
     * @brief Gets the current PID controller gains.
     */
    xSjcServoPidParameters_t PidController_getControlParameters(void);

    /**
     * @brief Sets the PID controller gains.
     */
    void PidController_setControlParameters(const xSjcServoPidParameters_t Parameters);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__PID_CONTROLLER_H */
