/**
 * @file controller.c
 * @brief Discrete controller implementation with anti-windup.
 *
 * Implements a second-order discrete-time controller using Direct Form II structure.
 * Includes anti-windup strategy that halts integration when output saturates.
 *
 * Discrete-time controller:
 * Cd(z) = 0.0050725 * (z - 0.9093)(z + 1) / ((z - 0.9952)(z - 1))
 *
 * Derived via Tustin method from:
 * C(s) = 9.7082 * (s + 95) / (s * (s + 4.8))
 *
 * Author: Tiago
 * Date: Apr 11, 2025
 */

#include "modules/include/discrete_controller.h"
#include <stdbool.h>

// === Controller coefficients (Direct Form II) ===
#define B0  0.0050725f
#define B1  0.0005424f
#define B2 -0.0045301f
#define A1 -1.9952f
#define A2  0.9952f

// === Controller state structure ===
static struct {
    float e[3];           ///< Error history
    float u[3];           ///< Output history
    float u_min;          ///< Minimum control effort
    float u_max;          ///< Maximum control effort
} xDiscController = {
    .e = {0.0f, 0.0f, 0.0f},
    .u = {0.0f, 0.0f, 0.0f},
    .u_min = -24.0f,
    .u_max =  24.0f
};

void DiscreteController_init(void)
{
    for (int i = 0; i < 3; ++i) {
        xDiscController.e[i] = 0.0f;
        xDiscController.u[i] = 0.0f;
    }
}

float DiscreteController_compute(const float reference, const float measurement)
{
    float error = reference - measurement;

    float control =
        B0 * error +
        B1 * xDiscController.e[0] +
        B2 * xDiscController.e[1] -
        A1 * xDiscController.u[0] -
        A2 * xDiscController.u[1];

    // Saturate control
    float saturated_control = control;
    bool saturated = false;

    if (control > xDiscController.u_max) {
        saturated_control = xDiscController.u_max;
        saturated = true;
    } else if (control < xDiscController.u_min) {
        saturated_control = xDiscController.u_min;
        saturated = true;
    }

    // Update histories
    xDiscController.u[1] = xDiscController.u[0];
    xDiscController.u[0] = saturated_control;

    if (!saturated) {
        xDiscController.e[2] = xDiscController.e[1];
        xDiscController.e[1] = xDiscController.e[0];
        xDiscController.e[0] = error;
    }

    return xDiscController.u[0];
}

float DiscreteController_getOutput(void)
{
    return xDiscController.u[0];
}

void DiscreteController_reset(void)
{
    for (int i = 0; i < 3; ++i) {
        xDiscController.e[i] = 0.0f;
        xDiscController.u[i] = 0.0f;
    }
}

void DiscreteController_setOutputLimits(float min, float max)
{
    xDiscController.u_min = min;
    xDiscController.u_max = max;
}

void DiscreteController_getOutputLimits(float *min, float *max)
{
    if (min) *min = xDiscController.u_min;
    if (max) *max = xDiscController.u_max;
}

void DiscreteController_updateParameters(void)
{
    // Optional: Implement runtime update of coefficients if needed
}
