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
    float u_last;
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
} lqi_ctx;

static float saturate_control(float u_raw, float *correction)
{
    float u_sat = fminf(fmaxf(u_raw, lqi_ctx.u_min), lqi_ctx.u_max);
    *correction = u_sat - u_raw;

    if (u_sat != u_raw)
    {
        DebugOutput_systemPrintf("\r\n[LQI] Output saturated: raw=%.3f, sat=%.3f", u_raw, u_sat);
    }
    return u_sat;
}

void LqiController_init(const LqiControllerParams_t *params)
{
    if (params == NULL)
    {
        DebugOutput_systemPrintf("\r\n[FAULT] LQI controller init failed: params is NULL.");
        return;
    }

    for (int i = 0; i < LQI_STATE_SIZE; ++i)
    {
        for (int j = 0; j < LQI_STATE_SIZE; ++j)
        {
            lqi_ctx.Ad[i][j] = params->Ad[i][j];
        }
        lqi_ctx.Bd[i] = params->Bd[i];
        lqi_ctx.Cd[i] = params->Cd[i];
        lqi_ctx.Kx[i] = params->Kx[i];
        lqi_ctx.x[i] = 0.0f;
    }

    lqi_ctx.Ki = params->Ki;
    lqi_ctx.Kw = params->Kw;
    lqi_ctx.alpha = params->alpha;
    lqi_ctx.u_min = params->u_min;
    lqi_ctx.u_max = params->u_max;

    lqi_ctx.e_int = 0.0f;
    lqi_ctx.err_prev = 0.0f;
    lqi_ctx.u_last = 0.0f;
    lqi_ctx.use_observer = params->use_observer;
    lqi_ctx.observer = params->use_observer ? params->observer : NULL;
    lqi_ctx.initialized = false;
}

float LqiController_compute(float ref, float y_meas)
{
    if (!lqi_ctx.initialized)
    {
        for (int i = 0; i < LQI_STATE_SIZE; ++i)
            lqi_ctx.x[i] = 0.0f;

        lqi_ctx.e_int = 0.0f;
        lqi_ctx.err_prev = ref - y_meas;
        lqi_ctx.u_last = 0.0f;
        lqi_ctx.initialized = true;
        return 0.0f;
    }

    float raw_err = ref - y_meas;
    float err = lqi_ctx.alpha * raw_err + (1.0f - lqi_ctx.alpha) * lqi_ctx.err_prev;
    lqi_ctx.err_prev = err;

    if (!isfinite(err) || !isfinite(y_meas) || !isfinite(ref))
    {
        DebugOutput_systemPrintf("\r\n[FAULT] NaN/Inf in input: ref=%.3f, meas=%.3f, err=%.3f", ref, y_meas, err);
        return 0.0f;
    }

    float x_control[LQI_STATE_SIZE];
    if (lqi_ctx.use_observer && lqi_ctx.observer != NULL)
    {
        CurrentObserver_update(lqi_ctx.observer, lqi_ctx.u_last, y_meas);
        x_control[0] = CurrentObserver_get_current(lqi_ctx.observer);
        x_control[1] = CurrentObserver_get_current_derivative(lqi_ctx.observer);
    }
    else
    {
        x_control[0] = lqi_ctx.x[0];
        x_control[1] = lqi_ctx.x[1];
    }

    float u_raw = -lqi_ctx.Kx[0] * x_control[0] - lqi_ctx.Kx[1] * x_control[1] - lqi_ctx.Ki * lqi_ctx.e_int;
    float correction = 0.0f;
    float u_sat = saturate_control(u_raw, &correction);

    if (!isfinite(u_raw) || !isfinite(u_sat))
    {
        DebugOutput_systemPrintf("\r\n[FAULT] NaN/Inf in output: u_raw=%.3f, u_sat=%.3f", u_raw, u_sat);
        return 0.0f;
    }

    if (fabsf(err) > 1e-2f)
    {
        lqi_ctx.e_int += err + correction * lqi_ctx.Kw;
        lqi_ctx.e_int = fminf(fmaxf(lqi_ctx.e_int, -E_INT_CLAMP), E_INT_CLAMP);
    }

    if (!lqi_ctx.use_observer)
    {
        float x_next[LQI_STATE_SIZE];
        for (int i = 0; i < LQI_STATE_SIZE; ++i)
        {
            x_next[i] = 0.0f;
            for (int j = 0; j < LQI_STATE_SIZE; ++j)
            {
                x_next[i] += lqi_ctx.Ad[i][j] * lqi_ctx.x[j];
            }
            x_next[i] += lqi_ctx.Bd[i] * u_sat;
        }
        lqi_ctx.x[0] = x_next[0];
        lqi_ctx.x[1] = x_next[1];
    }

    lqi_ctx.u_last = u_sat;
    return u_sat;
}

void LqiController_reset(void)
{
    for (int i = 0; i < LQI_STATE_SIZE; ++i)
    {
        lqi_ctx.x[i] = 0.0f;
    }
    lqi_ctx.e_int = 0.0f;
    lqi_ctx.err_prev = 0.0f;
    lqi_ctx.u_last = 0.0f;
    lqi_ctx.initialized = false;
}

void LqiController_setSaturation(float u_min, float u_max)
{
    lqi_ctx.u_min = u_min;
    lqi_ctx.u_max = u_max;
}

bool LqiController_isInitialized(void)
{
    return lqi_ctx.initialized;
}
