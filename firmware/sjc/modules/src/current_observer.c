/*
 * data_logger.c
 *
 *  Created on: Sep 31, 2024
 *      Author: tiago
 */

#include "modules/include/current_observer.h"
#include "modules/include/debug_output.h"
#include <stddef.h>

void CurrentObserver_init(CurrentObserver_t *observer,
                          const float A[2][2],
                          const float B[2],
                          const float C[2],
                          const float L[2])
{
    if (observer == NULL) return;

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

void CurrentObserver_update(CurrentObserver_t *observer, float u, float y_meas)
{
    if (observer == NULL || !observer->initialized) return;

    float y_hat = observer->C[0] * observer->x_hat[0] + observer->C[1] * observer->x_hat[1];
    float e = y_meas - y_hat;

    float x_next[2];
    x_next[0] = observer->A[0][0] * observer->x_hat[0] + observer->A[0][1] * observer->x_hat[1] + observer->B[0] * u + observer->L[0] * e;
    x_next[1] = observer->A[1][0] * observer->x_hat[0] + observer->A[1][1] * observer->x_hat[1] + observer->B[1] * u + observer->L[1] * e;

    // Verificação de estabilidade numérica
    if (!isfinite(x_next[0]) || !isfinite(x_next[1]))
    {
    	DebugOutput_systemPrintf("[FAULT] CurrentObserver: NaN/Inf detected\n");
    	DebugOutput_systemPrintf("  u=%.3f, y_meas=%.3f, y_hat=%.3f, e=%.3f\n", u, y_meas, y_hat, e);
    	DebugOutput_systemPrintf("  x_hat[0]=%.3f, x_hat[1]=%.3f\n", observer->x_hat[0], observer->x_hat[1]);
    	DebugOutput_systemPrintf("  x_next[0]=%.3f, x_next[1]=%.3f\n", x_next[0], x_next[1]);
    	DebugOutput_systemPrintf;  // Ou você pode optar por fazer observer->initialized = false
    }

    observer->x_hat[0] = x_next[0];
    observer->x_hat[1] = x_next[1];
}
