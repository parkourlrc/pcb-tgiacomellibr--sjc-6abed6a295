/*
 * motor.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "pid_controller.h"

// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static xSjcPidController_t xPidController = xPidControllerEmptyObj;
static xSjcRange_t xPidControllerEffortLimits = xSjcRangeEmptyObj;
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //
void reset_errors(void);
void reset_parameters(void);

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void PidController_Init(void)
{

	xPidController.Reference = 0;

	reset_errors();
	reset_parameters();
}

float PidController_compute(const float ControllerError, const float dt)
{
	xPidController.Errors.PrevError = xPidController.Errors.ActualError;
	xPidController.Errors.ActualError = ControllerError;
	xPidController.Errors.Derivative = (xPidController.Errors.ActualError - xPidController.Errors.PrevError) / dt;

	xPidController.ControlAction = xPidController.Errors.ActualError * xPidController.Parameters.Kp;
	xPidController.ControlAction += xPidController.Errors.Derivative * xPidController.Parameters.Kd;

	float TempIntegralError = xPidController.Errors.Integral + (xPidController.Errors.ActualError * dt);
	float TempIntegralAction = TempIntegralError*xPidController.Parameters.Ki;

	float TempControlAction = xPidController.ControlAction + (TempIntegralError*xPidController.Parameters.Ki);

	bool bErrorConditionalIntegration = !( ( TempControlAction < xPidControllerEffortLimits.Min ) || ( TempControlAction > xPidControllerEffortLimits.Max ) );

	if (true == bErrorConditionalIntegration)
	{
		xPidController.Errors.Integral = TempIntegralError;
	}

	xPidController.ControlAction += TempIntegralAction;


	return xPidController.ControlAction;
}

float PidController_getError(void)
{
	return xPidController.Errors.ActualError;
}

float PidController_getReference(void)
{
	return xPidController.Reference;
}

void PidController_setReference(const float reference)
{
	xPidController.Reference = reference;
}

xSjcRange_t PidController_getActuatorEffortLimits(void)
{
	return xPidControllerEffortLimits;
}

void PidController_setActuatorEffortLimits(const xSjcRange_t limits)
{
	xPidControllerEffortLimits.Min = limits.Min;
	xPidControllerEffortLimits.Max = limits.Max;
}

xSjcServoPidParameters_t PidController_getControlParameters(void)
{
	return xPidController.Parameters;
}

void PidController_setControlParameters(const xSjcServoPidParameters_t Parameters)
{
	xPidController.Parameters = Parameters;
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void reset_errors(void)
{
	xPidController.Errors = xControllerErrorsEmptyObj;
}

void reset_parameters(void)
{
	xPidController.Parameters = xSjcServoPidParametersEmptyObj;
}

// ------------------------------------------------------------------- //
