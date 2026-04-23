/*
 * discrete_controller.c
 *
 *  Created on: Apr 23, 2025
 *      Author: Tiago Giacomelli
 *
 *  Discrete controller implementation with exponential filtering and anti-windup.
 *  Implements a second-order direct-form discrete-time controller:
 *      Cd(z) = (B0 + B1*z^-1 + B2*z^-2) / (1 + A1*z^-1 + A2*z^-2)
 *
 *  Features:
 *   - Output saturation
 *   - Anti-windup using back-calculation
 *   - Optional exponential filtering on the error signal
 *
 *  Designed for real-time embedded control applications.
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "modules/include/discrete_controller.h"
#include "system_types/include/filters.h"
#include "system_types/include/helpers.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static xSjcDiscreteController_t xDiscreteController;
static ExponentialFilter_t xDiscreteControllerFilter;
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //
static void reset_errors(void);
static void reset_filter(void);
static void reset_parameters(void);
static void reset_control_action(void);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //

void DiscreteController_init(void)
{
    // Initialize exponential filter (default: 1 kHz sample, 1 kHz cutoff)
    ExponentialFilter_init(&xDiscreteControllerFilter, 1000.0f, 1000.0f);
    ExponentialFilter_reset(&xDiscreteControllerFilter);

    reset_errors();
    reset_filter();
    reset_parameters();
    reset_control_action();
}

float DiscreteController_compute(const float reference, const float measurement)
{
    // Compute error and apply exponential filter
    float error = reference - measurement;
    error = ExponentialFilter_update(&xDiscreteControllerFilter, error);

    // Store reference for monitoring
    xDiscreteController.Reference = reference;

    // Direct Form II, second-order difference equation
    float control =
        xDiscreteController.parameters.B[0] * error +
        xDiscreteController.parameters.B[1] * xDiscreteController.Error[0] +
        xDiscreteController.parameters.B[2] * xDiscreteController.Error[1] -
        xDiscreteController.parameters.A[1] * xDiscreteController.ControlAction[0] -
        xDiscreteController.parameters.A[2] * xDiscreteController.ControlAction[1];

    // Output saturation
    float saturated_control = control;
    if (control > xDiscreteController.parameters.u_max)
    {
        saturated_control = xDiscreteController.parameters.u_max;
    }
    else if (control < xDiscreteController.parameters.u_min)
    {
        saturated_control = xDiscreteController.parameters.u_min;
    }

    // Update control action history (most recent at index [0])
    xDiscreteController.ControlAction[2] = xDiscreteController.ControlAction[1];
    xDiscreteController.ControlAction[1] = xDiscreteController.ControlAction[0];
    xDiscreteController.ControlAction[0] = saturated_control;

    // Anti-windup via back-calculation
    float backcalc = xDiscreteController.parameters.K_aw * (saturated_control - control);
    float corrected_error = error + backcalc;

    // Update error history (most recent at index [0])
    xDiscreteController.Error[2] = xDiscreteController.Error[1];
    xDiscreteController.Error[1] = xDiscreteController.Error[0];
    xDiscreteController.Error[0] = corrected_error;

    return saturated_control;
}

float DiscreteController_getOutput(void)
{
    return xDiscreteController.ControlAction[0];
}

float DiscreteController_get_error(void)
{
    return xDiscreteController.Error[0];
}

float DiscreteController_get_reference(void)
{
    return xDiscreteController.Reference;
}

void DiscreteController_reset(void)
{
    reset_filter();
    reset_errors();
    reset_control_action();
    xDiscreteController.Reference = 0.0f;
}

void DiscreteController_setSaturation(float min, float max)
{
    xDiscreteController.parameters.u_min = min;
    xDiscreteController.parameters.u_max = max;
}

void DiscreteController_getOutputLimits(float *min, float *max)
{
    if (min != NULL)
    {
        *min = xDiscreteController.parameters.u_min;
    }
    if (max != NULL)
    {
        *max = xDiscreteController.parameters.u_max;
    }
}

void DiscreteController_setAntiWindupGain(float k_aw)
{
    xDiscreteController.parameters.K_aw = k_aw;
}

void DiscreteController_setFilterCutoffFrequency(float cutoff_freq)
{
    ExponentialFilter_set_cutoff(&xDiscreteControllerFilter, cutoff_freq);
}

void DiscreteController_get_parameters(xSjcDiscreteControllerParameters_t *params)
{
    SJC_RETURN_IF_NULL_LOG_VOID(params, "DiscreteController_get_parameters: params");
    *params = xDiscreteController.parameters;
}

void DiscreteController_set_parameters(const xSjcDiscreteControllerParameters_t params)
{
    // Copy by value ensures encapsulation and avoids dangling references
    xDiscreteController.parameters = params;

    // Reconfigure filter with updated cutoff frequency
    ExponentialFilter_reset(&xDiscreteControllerFilter);
    ExponentialFilter_set_cutoff(&xDiscreteControllerFilter, xDiscreteController.parameters.cutoff_freq);
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

static void reset_errors(void)
{
    for (int i = 0; i < 3; ++i)
    {
        xDiscreteController.Error[i] = 0.0f;
    }
}

static void reset_filter(void)
{
    // Reset filter state, preserves cutoff frequency
    ExponentialFilter_reset(&xDiscreteControllerFilter);
}

static void reset_parameters(void)
{
    xDiscreteController.parameters = xSjcDiscreteControllerParametersEmptyObj;
}

static void reset_control_action(void)
{
    for (int i = 0; i < 3; ++i)
    {
        xDiscreteController.ControlAction[i] = 0.0f;
    }
}
// ------------------------------------------------------------------- //
