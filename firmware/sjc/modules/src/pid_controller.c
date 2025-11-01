/*
 * pid_controller.c
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago
 *
 *  @brief PID controller implementation for actuator control.
 */

#include "modules/include/pid_controller.h"
#include "base/include/configs.h"
#include <math.h> // for isfinite()

// ------------------------------------------------------------------- //
// Module variables
// ------------------------------------------------------------------- //
static xSjcPidController_t xPidController = xPidControllerEmptyObj;
static xSjcRange_t xPidControllerEffortLimits = xSjcRangeEmptyObj;

// ------------------------------------------------------------------- //
// Private function prototypes
// ------------------------------------------------------------------- //
static void reset_errors(void);

// ------------------------------------------------------------------- //
// Public functions
// ------------------------------------------------------------------- //

void PidController_init(void)
{
    xPidController.Reference = 0.0f;
    xPidController.ControlAction = 0.0f;

    reset_errors();
    xPidController.Parameters = xSjcPidParametersEmptyObj;
    xPidControllerEffortLimits = xSjcRangeEmptyObj;
}

float PidController_compute(const float reference, const float measurement, const float dt)
{
    if (dt <= 0.0f || !isfinite(dt))
    {
        xPidController.ControlAction = 0.0f;
        return 0.0f;
    }

    // Error update
    float error = reference - measurement;
    xPidController.Errors.PrevError = xPidController.Errors.ActualError;
    xPidController.Errors.ActualError = error;
    xPidController.Errors.Derivative =
        (xPidController.Errors.ActualError - xPidController.Errors.PrevError) / dt;

    // P + D
    float control_pd =
        (xPidController.Errors.ActualError * xPidController.Parameters.Kp) +
        (xPidController.Errors.Derivative * xPidController.Parameters.Kd);

    // Candidate integral
    float tempIntegral = xPidController.Errors.Integral + (error * dt);
    float tempEffort = control_pd + (tempIntegral * xPidController.Parameters.Ki);

    // Conditional integration
    if (tempEffort > xPidControllerEffortLimits.Min &&
        tempEffort < xPidControllerEffortLimits.Max)
    {
        xPidController.Errors.Integral = tempIntegral;
    }

    // Final unsaturated effort
    float effort = control_pd + (xPidController.Errors.Integral * xPidController.Parameters.Ki);

    // Saturate
    if (effort > xPidControllerEffortLimits.Max)
        effort = xPidControllerEffortLimits.Max;
    if (effort < xPidControllerEffortLimits.Min)
        effort = xPidControllerEffortLimits.Min;

    xPidController.ControlAction = effort;
    return effort;
}

float PidController_get_error(void)
{
    return xPidController.Errors.ActualError;
}

xSjcRange_t PidController_get_saturation(void)
{
    return xPidControllerEffortLimits;
}

void PidController_set_saturation(const xSjcRange_t limits)
{
    xPidControllerEffortLimits = limits;
}

xSjcPidParameters_t PidController_get_parameters(void)
{
    return xPidController.Parameters;
}

void PidController_copy_parameters(xSjcPidParameters_t *parameters)
{
    if (parameters != NULL)
    {
        *parameters = xPidController.Parameters;
    }
}

void PidController_set_parameters(const xSjcPidParameters_t parameters)
{
    xPidController.Parameters = parameters;
}

void PidController_reset(void)
{
    reset_errors();
    xPidController.Reference = 0.0f;
    xPidController.ControlAction = 0.0f;
}

// ------------------------------------------------------------------- //
// Private functions
// ------------------------------------------------------------------- //

static void reset_errors(void)
{
    xPidController.Errors = xControllerErrorsEmptyObj;
}
