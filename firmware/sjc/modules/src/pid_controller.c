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

#include "configs.h"
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
void SJC_PidController_Init(void)
{

	xPidController.Reference = 0;

	reset_errors();
	reset_parameters();
}

float SJC_PidController_Compute(const float ControllerError, const float dt)
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

float SJC_PidController_GetError(void)
{
	return xPidController.Errors.ActualError;
}

float SJC_PidController_GetReference(void)
{
	return xPidController.Reference;
}

void SJC_PidController_SetReference(const float reference)
{
	xPidController.Reference = reference;
}

xSjcRange_t SJC_PidController_GetActuatorEffortLimits(void)
{
	return xPidControllerEffortLimits;
}

void SJC_PidController_SetActuatorEffortLimits(const xSjcRange_t limits)
{
	xPidControllerEffortLimits.Min = limits.Min;
	xPidControllerEffortLimits.Max = limits.Max;
}

xPidControllerParameters_t SJC_PidController_GetControlParameters(void)
{
	return xPidController.Parameters;
}

void SJC_PidController_SetControlParameters(const xPidControllerParameters_t Parameters)
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
	xPidController.Parameters = xControllerParametersEmptyObj;
}

// ------------------------------------------------------------------- //
