/**
 * @file lqi_controller.c
 * @brief LQI controller implementation with ZOH-discretized internal model and anti-windup.
 *
 * Implements a discrete-time LQI controller using a ZOH-discretized model,
 * exponential filtering on the error signal, and anti-windup via back-calculation.
 *
 * Author: Tiago
 * Date: Apr 23, 2025
 */

#include "modules/include/lqi_controller.h"
#include "modules/include/debug_output.h"
#include <math.h>
#include <stddef.h>

#define E_INT_CLAMP 100.0f

static struct
{
    float x[LQI_STATE_SIZE];
    float e_int;
    float err_prev;
    float u_min;
    float u_max;

    float Kx[LQI_STATE_SIZE];
    float Ki;
    float Kw;
    float alpha;

    float Ad[LQI_STATE_SIZE][LQI_STATE_SIZE];
    float Bd[LQI_STATE_SIZE];
    float Cd[LQI_STATE_SIZE];

    bool use_observer;
    CurrentObserver_t *observer;

    bool initialized;
} lqi;

void LqiController_init(const LqiControllerParams_t *params)
{
    for (int i = 0; i < LQI_STATE_SIZE; ++i)
    {
        for (int j = 0; j < LQI_STATE_SIZE; ++j)
        {
            lqi.Ad[i][j] = params->Ad[i][j];
        }
        lqi.Bd[i] = params->Bd[i];
        lqi.Cd[i] = params->Cd[i];
        lqi.Kx[i] = params->Kx[i];
        lqi.x[i] = 0.0f;
    }

    lqi.Ki = params->Ki;
    lqi.Kw = params->Kw;
    lqi.alpha = params->alpha;
    lqi.u_min = params->u_min;
    lqi.u_max = params->u_max;

    lqi.e_int = 0.0f;
    lqi.err_prev = 0.0f;
    lqi.use_observer = params->use_observer;
    lqi.observer = params->observer;
    lqi.initialized = false;
}

float LqiController_compute(float ref, float y_meas, float u_prev)
{
    if (!lqi.initialized)
    {
        for (int i = 0; i < LQI_STATE_SIZE; ++i)
            lqi.x[i] = 0.0f;
        lqi.e_int = 0.0f;
        lqi.err_prev = ref - y_meas;
        lqi.initialized = true;
        return 0.0f;
    }

    float raw_err = ref - y_meas;
    float err = lqi.alpha * raw_err + (1.0f - lqi.alpha) * lqi.err_prev;
    lqi.err_prev = err;

    if (!isfinite(err) || !isfinite(y_meas) || !isfinite(ref))
    {
        DebugOutput_systemPrintf("\r\n[FAULT] NaN/Inf in input: ref=%.3f, meas=%.3f, err=%.3f", ref, y_meas, err);
        return 0.0f;
    }

    float x_control[LQI_STATE_SIZE];
    if (lqi.use_observer && lqi.observer != NULL)
    {
        CurrentObserver_update(lqi.observer, u_prev, y_meas);
        x_control[0] = CurrentObserver_get_current(lqi.observer);
        x_control[1] = CurrentObserver_get_current_derivative(lqi.observer);
    }
    else
    {
        x_control[0] = lqi.x[0];
        x_control[1] = lqi.x[1];
    }

    float u_raw = -lqi.Kx[0] * x_control[0] - lqi.Kx[1] * x_control[1] - lqi.Ki * lqi.e_int;
    float u_sat = fminf(fmaxf(u_raw, lqi.u_min), lqi.u_max);

    if (!isfinite(u_raw) || !isfinite(u_sat))
    {
        DebugOutput_systemPrintf("\r\n[FAULT] NaN/Inf in output: u_raw=%.3f, u_sat=%.3f", u_raw, u_sat);
        return 0.0f;
    }

    if (fabsf(err) > 1e-2f)
    {
        lqi.e_int += err + (u_sat - u_raw) * lqi.Kw;
        lqi.e_int = fminf(fmaxf(lqi.e_int, -E_INT_CLAMP), E_INT_CLAMP);
    }

    if (!lqi.use_observer)
    {
        float x_next[LQI_STATE_SIZE];
        for (int i = 0; i < LQI_STATE_SIZE; ++i)
        {
            x_next[i] = 0.0f;
            for (int j = 0; j < LQI_STATE_SIZE; ++j)
            {
                x_next[i] += lqi.Ad[i][j] * lqi.x[j];
            }
            x_next[i] += lqi.Bd[i] * u_sat;
        }
        lqi.x[0] = x_next[0];
        lqi.x[1] = x_next[1];
    }

    return (fabsf(u_sat) < 0.01f) ? 0.0f : u_sat;
}

void LqiController_reset(void)
{
    for (int i = 0; i < LQI_STATE_SIZE; ++i)
    {
        lqi.x[i] = 0.0f;
    }
    lqi.e_int = 0.0f;
    lqi.err_prev = 0.0f;
    lqi.initialized = false;
}

void LqiController_setSaturation(float u_min, float u_max)
{
    lqi.u_min = u_min;
    lqi.u_max = u_max;
}

