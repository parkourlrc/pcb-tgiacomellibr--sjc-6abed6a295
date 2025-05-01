/**
 * @file controller.c
 * @brief Discrete controller implementation with exponential filtering and anti-windup.
 *
 * Implements a second-order direct-form discrete-time controller:
 * Cd(z) = (B0 + B1*z^-1 + B2*z^-2) / (1 + A1*z^-1 + A2*z^-2)
 * Includes saturation and anti-windup using back-calculation, and optional exponential filtering.
 *
 * Author: Tiago
 * Date: Apr 23, 2025
 */

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "modules/include/discrete_controller.h"
#include "system_types/include/filters.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// Controller coefficients (Direct Form II)
#define B0 1.200000000000001f
#define B1 -2.100000000000002f
#define B2 0.918000000000001f
#define A1 -1.970000000000000f
#define A2 0.970000000000000f

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// Controller internal state
static struct
{
    float e[3];  ///< Error history
    float u[3];  ///< Output history
    float u_min; ///< Minimum output limit
    float u_max; ///< Maximum output limit
    float k_aw;  ///< Anti-windup gain
} xDiscController = {
    .e = {0.0f},
    .u = {0.0f},
    .u_min = -24.0f,
    .u_max = 24.0f,
    .k_aw = 0.01f};

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static ExponentialFilter_t filter;
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void DiscreteController_init(void)
{
    for (int i = 0; i < 3; ++i)
    {
        xDiscController.e[i] = 0.0f;
        xDiscController.u[i] = 0.0f;
    }

    ExponentialFilter_init(&filter, 1000.0f, 1000.0f);
    ExponentialFilter_reset(&filter);
}

float DiscreteController_compute(const float reference, const float measurement)
{
    float error = reference - measurement;
    error = ExponentialFilter_update(&filter, error);

    float control =
        B0 * error +
        B1 * xDiscController.e[0] +
        B2 * xDiscController.e[1] -
        A1 * xDiscController.u[0] -
        A2 * xDiscController.u[1];

    float saturated_control = control;
    bool is_saturated = false;

    if (control > xDiscController.u_max)
    {
        saturated_control = xDiscController.u_max;
        is_saturated = true;
    }
    else if (control < xDiscController.u_min)
    {
        saturated_control = xDiscController.u_min;
        is_saturated = true;
    }

    // Update histories
    xDiscController.u[1] = xDiscController.u[0];
    xDiscController.u[0] = saturated_control;

    float backcalc = xDiscController.k_aw * (saturated_control - control);
    float corrected_error = error + backcalc;

    xDiscController.e[2] = xDiscController.e[1];
    xDiscController.e[1] = xDiscController.e[0];
    xDiscController.e[0] = corrected_error;

    return saturated_control;
}

float DiscreteController_getOutput(void)
{
    return xDiscController.u[0];
}

void DiscreteController_reset(void)
{
    for (int i = 0; i < 3; ++i)
    {
        xDiscController.e[i] = 0.0f;
        xDiscController.u[i] = 0.0f;
    }
    ExponentialFilter_reset(&filter);
}

void DiscreteController_setOutputLimits(float min, float max)
{
    xDiscController.u_min = min;
    xDiscController.u_max = max;
}

void DiscreteController_getOutputLimits(float *min, float *max)
{
    if (min != NULL)
        *min = xDiscController.u_min;
    if (max != NULL)
        *max = xDiscController.u_max;
}

void DiscreteController_setAntiWindupGain(float k_aw)
{
    xDiscController.k_aw = k_aw;
}

void DiscreteController_setFilterCutoffFrequency(float cutoff_freq)
{
    ExponentialFilter_set_cutoff(&filter, cutoff_freq);
}

void DiscreteController_updateParameters(void)
{
    // Not implemented in static version.
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
