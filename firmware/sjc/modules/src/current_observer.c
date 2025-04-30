/**
 * @file current_observer.c
 * @brief Robust second-order state observer for current and current derivative.
 *
 * Provides numerical stability, saturation, and error detection.
 *
 * Author: Tiago
 * Date: Apr 2025
 */

#include "modules/include/current_observer.h"
#include "modules/include/debug_output.h"
#include <math.h> // for isfinite(), fabsf(), fminf(), fmaxf()
#include <stddef.h>

#define OBSERVER_CURRENT_LIMIT 70.0f      ///< Maximum expected current [A] with safety margin
#define OBSERVER_CURRENT_DERIV_LIMIT 1e5f ///< Maximum expected current derivative [A/s]

/**
 * @brief Initializes the current observer structure.
 *
 * @param observer Pointer to the observer instance
 * @param A State transition matrix
 * @param B Input matrix
 * @param C Output matrix
 * @param L Observer gain vector
 */
void CurrentObserver_init(CurrentObserver_t *observer,
                          const float A[2][2],
                          const float B[2],
                          const float C[2],
                          const float L[2])
{
    if (observer == NULL)
        return;

    for (int i = 0; i < 2; ++i)
    {
        observer->B[i] = B[i];
        observer->C[i] = C[i];
        observer->L[i] = L[i];
        observer->x_hat[i] = 0.0f;

        for (int j = 0; j < 2; ++j)
        {
            observer->A[i][j] = A[i][j];
        }
    }

    observer->initialized = true;
}

/**
 * @brief Updates the state estimates using control input and measured output.
 *
 * Includes error detection for NaN/Inf values and enforces output saturation.
 *
 * @param observer Pointer to the observer
 * @param u Control input (e.g., voltage)
 * @param y_meas Measured output (e.g., measured current)
 */
void CurrentObserver_update(CurrentObserver_t *observer, float u, float y_meas)
{
    if (observer == NULL || !observer->initialized)
        return;

    if (!isfinite(u) || !isfinite(y_meas))
    {
        DebugOutput_systemPrintf("[FAULT] CurrentObserver: invalid input: u=%.3f, y_meas=%.3f\n", u, y_meas);
        return;
    }

    // Compute estimated output
    float y_hat = observer->C[0] * observer->x_hat[0] + observer->C[1] * observer->x_hat[1];
    float e = y_meas - y_hat;

    // Predict next state
    float x_next[2];
    x_next[0] = observer->A[0][0] * observer->x_hat[0] + observer->A[0][1] * observer->x_hat[1] + observer->B[0] * u + observer->L[0] * e;
    x_next[1] = observer->A[1][0] * observer->x_hat[0] + observer->A[1][1] * observer->x_hat[1] + observer->B[1] * u + observer->L[1] * e;

    // Check for numerical instability
    if (!isfinite(x_next[0]) || !isfinite(x_next[1]))
    {
        DebugOutput_systemPrintf("[FAULT] CurrentObserver: NaN/Inf detected\n");
        DebugOutput_systemPrintf("  u=%.3f, y_meas=%.3f, y_hat=%.3f, e=%.3f\n", u, y_meas, y_hat, e);
        DebugOutput_systemPrintf("  x_hat[0]=%.3f, x_hat[1]=%.3f\n", observer->x_hat[0], observer->x_hat[1]);
        DebugOutput_systemPrintf("  x_next[0]=%.3f, x_next[1]=%.3f\n", x_next[0], x_next[1]);

        CurrentObserver_reset(observer); // Reset observer to prevent error propagation
        return;
    }

    // Apply saturation to avoid unbounded growth
    x_next[0] = fminf(fmaxf(x_next[0], -OBSERVER_CURRENT_LIMIT), OBSERVER_CURRENT_LIMIT);
    x_next[1] = fminf(fmaxf(x_next[1], -OBSERVER_CURRENT_DERIV_LIMIT), OBSERVER_CURRENT_DERIV_LIMIT);

    // Update internal state
    observer->x_hat[0] = x_next[0];
    observer->x_hat[1] = x_next[1];
}

/**
 * @brief Resets the observer state and re-enables it.
 *
 * Should be called after fault recovery or system reinitialization.
 *
 * @param observer Pointer to the observer
 */
void CurrentObserver_reset(CurrentObserver_t *observer)
{
    if (observer)
    {
        observer->x_hat[0] = 0.0f;
        observer->x_hat[1] = 0.0f;
        observer->initialized = true;
    }
}
