/**
 * @file lqr_observer.c
 * @brief Robust second-order state observer for current and its derivative.
 *
 * Implements a discrete-time Luenberger observer for current and dI/dt.
 * Provides:
 *  - Numerical stability checks
 *  - Output saturation
 *  - Fault detection and recovery
 *
 * Author: Tiago
 * Date: Apr 2025
 */

#include "modules/include/lqr_observer.h"
#include "modules/include/debug_output.h"

#include <math.h>   // isfinite(), fminf(), fmaxf()
#include <stddef.h>

#define OBSERVER_CURRENT_LIMIT       70.0f   ///< Max expected current [A] with margin
#define OBSERVER_CURRENT_DERIV_LIMIT 1e5f    ///< Max expected current derivative [A/s]

// ------------------------------------------------------------------- //
// Internal state
// ------------------------------------------------------------------- //
static xSjcLQRObserver_t xCurrentObserver;

// ------------------------------------------------------------------- //
// Private function prototypes
// ------------------------------------------------------------------- //
static void reset_states(void);
static void reset_parameters(void);

// ------------------------------------------------------------------- //
// Public API
// ------------------------------------------------------------------- //

void LQRObserver_init(void)
{
    xCurrentObserver = xSjcLQRObserverEmptyObj;
}

void LQRObserver_update(float u, float y_meas)
{
    if (!isfinite(u) || !isfinite(y_meas))
    {
        DebugOutput_systemPrintf("[FAULT] LQRObserver: invalid input u=%.3f, y=%.3f\n", u, y_meas);
        return;
    }

    // Estimated output
    float y_hat = xCurrentObserver.parameters.Co[0] * xCurrentObserver.x_hat[0] +
                  xCurrentObserver.parameters.Co[1] * xCurrentObserver.x_hat[1];

    float e = y_meas - y_hat;

    // Next state prediction
    float x_next[2];
    x_next[0] = xCurrentObserver.parameters.Ao[0][0] * xCurrentObserver.x_hat[0] +
                xCurrentObserver.parameters.Ao[0][1] * xCurrentObserver.x_hat[1] +
                xCurrentObserver.parameters.Bo[0] * u +
                xCurrentObserver.parameters.L[0]  * e;

    x_next[1] = xCurrentObserver.parameters.Ao[1][0] * xCurrentObserver.x_hat[0] +
                xCurrentObserver.parameters.Ao[1][1] * xCurrentObserver.x_hat[1] +
                xCurrentObserver.parameters.Bo[1] * u +
                xCurrentObserver.parameters.L[1]  * e;

    // Check numerical validity
    if (!isfinite(x_next[0]) || !isfinite(x_next[1]))
    {
        DebugOutput_systemPrintf("[FAULT] LQRObserver: NaN/Inf detected\n");
        DebugOutput_systemPrintf("  u=%.3f, y=%.3f, y_hat=%.3f, e=%.3f\n", u, y_meas, y_hat, e);
        DebugOutput_systemPrintf("  x_hat[0]=%.3f, x_hat[1]=%.3f\n",
                                 xCurrentObserver.x_hat[0], xCurrentObserver.x_hat[1]);

        LQRObserver_reset(); // Reset observer on fault
        return;
    }

    // Apply saturation
    x_next[0] = fminf(fmaxf(x_next[0], -OBSERVER_CURRENT_LIMIT), OBSERVER_CURRENT_LIMIT);
    x_next[1] = fminf(fmaxf(x_next[1], -OBSERVER_CURRENT_DERIV_LIMIT), OBSERVER_CURRENT_DERIV_LIMIT);

    // Update internal states (avoid residuals close to zero)
    xCurrentObserver.x_hat[0] = (fabsf(x_next[0]) < 1e-9f) ? 0.0f : x_next[0];
    xCurrentObserver.x_hat[1] = (fabsf(x_next[1]) < 1e-9f) ? 0.0f : x_next[1];
}

void LQRObserver_getParameters(xSjcLQRObserverParameters_t *params)
{
    if (params != NULL)
    {
        *params = xCurrentObserver.parameters;
    }
}

void LQRObserver_setParameters(const xSjcLQRObserverParameters_t params)
{
    xCurrentObserver.parameters = params;

     DebugOutput_systemPrintf("\r\n[NetCmd][LQR] Ao = [[%.4f, %.4f], [%.4f, %.4f]]",
                        params.Ao[0][0], params.Ao[0][1],
                        params.Ao[1][0], params.Ao[1][1]);

     DebugOutput_systemPrintf("\r\n[NetCmd][LQR] Bo = [%.4f, %.4f]",
                        params.Bo[0], params.Bo[1]);

     DebugOutput_systemPrintf("\r\n[NetCmd][LQR] Co = [%.4f, %.4f]",
                        params.Co[0], params.Co[1]);

     DebugOutput_systemPrintf("\r\n[NetCmd][LQR] L  = [%.4f, %.4f]",
                        params.L[0], params.L[1]);

     DebugOutput_systemPrintf("\r\n[NetCmd][LQR] Obs Cutoff: %.4f Hz",
                        params.obs_cutoff_freq);
}

float LQRObserver_get_state(void)
{
    return xCurrentObserver.x_hat[0];
}

float LQRObserver_get_state_derivative(void)
{
    return xCurrentObserver.x_hat[1];
}

void LQRObserver_reset(void)
{
    reset_states();
    reset_parameters();
}

// ------------------------------------------------------------------- //
// Private helpers
// ------------------------------------------------------------------- //
static void reset_states(void)
{
    xCurrentObserver.x_hat[0] = 0.0f;
    xCurrentObserver.x_hat[1] = 0.0f;
}

static void reset_parameters(void)
{
    xCurrentObserver.parameters = xSjcLQRObserverParametersEmptyObj;
}
