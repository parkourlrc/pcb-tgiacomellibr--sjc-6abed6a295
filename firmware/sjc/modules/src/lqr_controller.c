/*
 * lqr_controller.c
 *
 *  Created on: Apr 23, 2025
 *      Author: Tiago Giacomelli
 *
 *  Discrete-time LQR/LQI controller with integral action, anti-windup, and
 *  optional observer. This module implements a discrete-time Linear Quadratic
 *  Regulator (LQR), extended with integral action (LQI), back-calculation
 *  anti-windup, and exponential filtering on the error signal.
 *
 *   - Supports order = 1 (single-state model + integral)
 *   - Supports order = 2 (two-state model + integral, optional observer)
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "modules/include/lqr_controller.h"
#include "modules/include/lqr_observer.h"
#include "modules/include/debug_output.h"
#include "system_types/include/filters.h"
#include "system_types/include/helpers.h"
#include <math.h>
#include <stddef.h>
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //
#define E_INT_CLAMP 100.0f
// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static xSjcLQRController_t xLqr;
static ExponentialFilter_t filter;
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //
static float saturate_control(float u_raw, float *correction);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //

void LQRController_init(void)
{
	xLqr = xSjcLQRControllerEmptyObj;

	ExponentialFilter_init(&filter, 1000.0f, 1000.0f);
	ExponentialFilter_reset(&filter);

	LQRObserver_init();
	xLqr.initialized = true;
}

float LQRController_compute(float ref, float y_meas)
{
	if (!xLqr.initialized)
	{
		LQRController_reset();
	}

	// Error calculation with exponential filter
	float raw_err = ref - y_meas;
	float err = ExponentialFilter_update(&filter, raw_err);

	if (!isfinite(err) || !isfinite(y_meas) || !isfinite(ref))
	{
		DebugOutput_systemPrintf(
				"\r\n[FAULT] LQRController: NaN/Inf detected (ref=%.3f, meas=%.3f, err=%.3f)",
				ref, y_meas, err);
		return 0.0f;
	}

	float x_control[LQR_CONTROLLER_STATE_SIZE] = {0.0f};

	// State source: observer (if order=2 + enabled) or internal model
	if (xLqr.parameters.order == 2 && xLqr.parameters.use_observer)
	{
		LQRObserver_update(xLqr.u_last, y_meas);
		x_control[0] = LQRObserver_get_state();
		x_control[1] = LQRObserver_get_state_derivative();
	}
	else
	{
		for (int i = 0; i < xLqr.parameters.order; ++i)
		{
			x_control[i] = xLqr.x[i];
		}
	}

	// Control law: u = -Kx·x - Ki·∫e
	float u_raw = 0.0f;
	for (int i = 0; i < xLqr.parameters.order; ++i)
	{
		u_raw -= xLqr.parameters.Kx[i] * x_control[i];
	}
	u_raw -= xLqr.parameters.Ki * xLqr.e_int;

	// Apply saturation + back-calculation
	float correction = 0.0f;
	float u_sat = saturate_control(u_raw, &correction);

	if (!isfinite(u_raw) || !isfinite(u_sat))
	{
		DebugOutput_systemPrintf("\r\n[FAULT] LQRController: NaN/Inf in control effort (u_raw=%.3f, u_sat=%.3f)",
				u_raw, u_sat);
		return 0.0f;
	}

	// Integrator update with anti-windup
	if (fabsf(err) > 1e-3f)
	{
		xLqr.e_int += err;                             // integrate error
		xLqr.e_int += correction * xLqr.parameters.Kw; // back-calculation
		xLqr.e_int = fminf(fmaxf(xLqr.e_int, -E_INT_CLAMP), E_INT_CLAMP);
	}

	// Internal state propagation (if not using observer)
	if (!(xLqr.parameters.order == 2 && xLqr.parameters.use_observer))
	{
		float x_next[LQR_CONTROLLER_STATE_SIZE] = {0.0f};
		for (int i = 0; i < xLqr.parameters.order; ++i)
		{
			for (int j = 0; j < xLqr.parameters.order; ++j)
			{
				x_next[i] += xLqr.parameters.Ad[i][j] * xLqr.x[j];
			}
			x_next[i] += xLqr.parameters.Bd[i] * u_sat;
		}
		for (int i = 0; i < xLqr.parameters.order; ++i)
		{
			xLqr.x[i] = x_next[i];
		}
	}

	// Telemetry update
	xLqr.u_last        = u_sat;
	xLqr.Error         = err;
	xLqr.Reference     = ref;
	xLqr.ControlAction = u_sat;

	return u_sat;
}

float LQRController_get_error(void)
{
	return xLqr.Error;
}

float LQRController_get_reference(void)
{
	return xLqr.Reference;
}

void LQRController_reset(void)
{
	for (int i = 0; i < LQR_CONTROLLER_STATE_SIZE; ++i)
	{
		xLqr.x[i] = 0.0f;
	}
	xLqr.e_int = 0.0f;
	xLqr.u_last = 0.0f;

	xLqr.Error         = 0.0f;
	xLqr.Reference     = 0.0f;
	xLqr.ControlAction = 0.0f;

	ExponentialFilter_reset(&filter);
	xLqr.initialized = true;
}

void LQRController_setSaturation(float u_min, float u_max)
{
	xLqr.parameters.u_min = u_min;
	xLqr.parameters.u_max = u_max;
}

bool LQRController_isInitialized(void)
{
	return xLqr.initialized;
}

void LQRController_get_parameters(xSjcLQRControllerParameters_t *control_params,
		xSjcLQRObserverParameters_t *observer_params)
{
	SJC_RETURN_IF_NULL_LOG_VOID(control_params, "LQRController_get_parameters: control_params");
	SJC_RETURN_IF_NULL_LOG_VOID(observer_params, "LQRController_get_parameters: observer_params");

	*control_params = xLqr.parameters;
	LQRObserver_getParameters(observer_params);
}

void LQRController_set_parameters(const xSjcLQRControllerParameters_t control_params,
		const xSjcLQRObserverParameters_t observer_params)
{
	xLqr.parameters = control_params;
	LQRObserver_setParameters(observer_params);

	ExponentialFilter_set_cutoff(&filter, xLqr.parameters.cutoff_freq);
	DebugOutput_systemPrintf("\r\n[INFO] LQRController Filter: %.3f (%.2f)",
			filter.alpha, xLqr.parameters.cutoff_freq);
	ExponentialFilter_reset(&filter);

	DebugOutput_systemPrintf("\r\n[NetCmd][LQR] --- Controller Parameters ---");
	DebugOutput_systemPrintf("\r\n[NetCmd][LQR] Order         : %u",   control_params.order);
	DebugOutput_systemPrintf("\r\n[NetCmd][LQR] Use Observer  : %u",   control_params.use_observer);

	DebugOutput_systemPrintf("\r\n[NetCmd][LQR] Kx = [%.4f, %.4f]",   control_params.Kx[0], control_params.Kx[1]);
	DebugOutput_systemPrintf("\r\n[NetCmd][LQR] Ki = %.4f",           control_params.Ki);
	DebugOutput_systemPrintf("\r\n[NetCmd][LQR] Kw = %.4f",           control_params.Kw);
	DebugOutput_systemPrintf("\r\n[NetCmd][LQR] Cutoff freq = %.4f Hz", control_params.cutoff_freq);

	DebugOutput_systemPrintf("\r\n[NetCmd][LQR] u_min = %.4f | u_max = %.4f",
			control_params.u_min, control_params.u_max);

	DebugOutput_systemPrintf("\r\n[NetCmd][LQR] Ad = [[%.4f, %.4f], [%.4f, %.4f]]",
			control_params.Ad[0][0], control_params.Ad[0][1],
			control_params.Ad[1][0], control_params.Ad[1][1]);

	DebugOutput_systemPrintf("\r\n[NetCmd][LQR] Bd = [%.4f, %.4f]",
			control_params.Bd[0], control_params.Bd[1]);

	DebugOutput_systemPrintf("\r\n[NetCmd][LQR] Cd = [%.4f, %.4f]",
			control_params.Cd[0], control_params.Cd[1]);
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

/**
 * @brief Apply saturation and compute anti-windup correction.
 *
 * @param u_raw Raw control effort.
 * @param correction Output: saturated - raw difference.
 * @return Saturated control effort.
 */
static float saturate_control(float u_raw, float *correction)
{
	float u_sat = fminf(fmaxf(u_raw, xLqr.parameters.u_min), xLqr.parameters.u_max);
	*correction = u_sat - u_raw;

#ifdef SJC_DEBUG_PRINT_NET_CMDS
	if (u_sat != u_raw)
	{
		DebugOutput_systemPrintf("\r\n[LQI] Saturation: raw=%.3f, sat=%.3f", u_raw, u_sat);
	}
#endif
	return u_sat;
}
// ------------------------------------------------------------------- //
