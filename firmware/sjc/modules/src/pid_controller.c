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
	xPidController.Errors.ActualError = ControllerError;

	xPidController.Errors.Integral = xPidController.Errors.ActualError + (xPidController.Errors.PrevError*dt);
	xPidController.Errors.Derivative = (xPidController.Errors.ActualError - xPidController.Errors.PrevError)/dt;
	float limit = SJC_ACTUATOR_LIMIT_MAX_VOLTAGE*xPidController.Parameters.Ki;
	//	xPidController.Errors.Integral = SJC_Helpers_RealClamp(xPidController.Errors.Integral, -limit, limit);
	xPidController.Errors.PrevError = xPidController.Errors.ActualError;

	xPidController.ControlAction = xPidController.Errors.ActualError*xPidController.Parameters.Kp;
	xPidController.ControlAction += xPidController.Errors.Integral*xPidController.Parameters.Ki;
	xPidController.ControlAction += xPidController.Errors.Derivative*xPidController.Parameters.Kd;

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
