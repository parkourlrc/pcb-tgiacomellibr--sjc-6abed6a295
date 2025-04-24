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
#include <stdbool.h>
#include <stddef.h>

#define STATE_SIZE 2
#define E_INT_CLAMP 24.0f

static struct
{
    float x[STATE_SIZE]; ///< Internal plant state
    float e_int;         ///< Integrated error
    float err_prev;      ///< Previous filtered error
    float u_min;         ///< Saturation lower limit
    float u_max;         ///< Saturation upper limit

    float Kx[STATE_SIZE]; ///< LQI gain vector (states)
    float Ki;             ///< LQI gain (integral)
    float Kw;             ///< Back-calculation gain
    float alpha;          ///< Exponential filter coefficient

    float Ad[STATE_SIZE][STATE_SIZE];
    float Bd[STATE_SIZE];
    float Cd[STATE_SIZE];

    bool initialized;
} lqi;

void LqiController_init(const LqiControllerParams_t *params)
{
    for (int i = 0; i < STATE_SIZE; ++i)
    {
        for (int j = 0; j < STATE_SIZE; ++j)
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
    lqi.initialized = false;
}

float LqiController_compute(float ref, float y_meas)
{
    if (!lqi.initialized)
    {
        for (int i = 0; i < STATE_SIZE; ++i)
            lqi.x[i] = 0.0f;
        lqi.e_int = 0.0f;
        lqi.err_prev = ref - y_meas;
        lqi.initialized = true;
        return 0.0f;
    }

    float raw_err = ref - y_meas;
    float err = lqi.alpha * raw_err + (1.0f - lqi.alpha) * lqi.err_prev;
    lqi.err_prev = err;

    if (!isfinite(err) || !isfinite(y_meas) || !isfinite(ref)) {
        DebugOutput_systemPrintf("\r\n[FAULT] Non-finite value detected: ref=%.3f, meas=%.3f, err=%.3f", ref, y_meas, err);
        return 0.0f;
    }

    float u_raw = -lqi.Kx[0] * lqi.x[0] - lqi.Kx[1] * lqi.x[1] - lqi.Ki * lqi.e_int;
    float u_sat = fminf(fmaxf(u_raw, lqi.u_min), lqi.u_max);

    if (!isfinite(u_raw) || !isfinite(u_sat)) {
        DebugOutput_systemPrintf("\r\n[FAULT] Non-finite control output: u_raw=%.3f, u_sat=%.3f", u_raw, u_sat);
        return 0.0f;
    }

    if (fabsf(err) > 1e-2f)
    {
        lqi.e_int += err + (u_sat - u_raw) * lqi.Kw;

        // Clamp integrador
        if (lqi.e_int > E_INT_CLAMP)
            lqi.e_int = E_INT_CLAMP;
        else if (lqi.e_int < -E_INT_CLAMP)
            lqi.e_int = -E_INT_CLAMP;
    }

    float x_next[STATE_SIZE];
    x_next[0] = lqi.Ad[0][0] * lqi.x[0] + lqi.Ad[0][1] * lqi.x[1] + lqi.Bd[0] * u_sat;
    x_next[1] = lqi.Ad[1][0] * lqi.x[0] + lqi.Ad[1][1] * lqi.x[1] + lqi.Bd[1] * u_sat;

    lqi.x[0] = x_next[0];
    lqi.x[1] = x_next[1];

    DebugOutput_systemPrintf("\r\n{\"ref\":%.3f,\"meas\":%.3f,\"err\":%.4f,\"u_raw\":%.3f,\"u_sat\":%.3f,\"e_int\":%.4f}",
        ref, y_meas, err, u_raw, u_sat, lqi.e_int);

    return (fabsf(u_sat) < 0.01f) ? 0.0f : u_sat;
}

void LqiController_reset(void)
{
    for (int i = 0; i < STATE_SIZE; ++i)
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
