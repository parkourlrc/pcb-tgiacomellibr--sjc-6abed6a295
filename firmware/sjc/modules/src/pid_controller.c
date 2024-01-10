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
static xSjcController_t * xPidController;
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //
void reset_errors(void);
void reset_parameters(void);

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t SJC_PidController_Init(xSjcController_t * Controller)
{
	if (Controller == NULL)
	{
		return SJC_RET_NOK;
	}

	xPidController = Controller;

	reset_errors();
	reset_parameters();

	return SJC_RET_OK;
}

float SJC_PidController_Compute(const float ControlError, const float dt)
{
	xPidController->Errors.ActualError = ControlError;

	xPidController->Errors.Integral = xPidController->Errors.ActualError + (xPidController->Errors.PrevError*dt);
	xPidController->Errors.Derivative = (xPidController->Errors.ActualError - xPidController->Errors.PrevError)/dt;
	float limit = SJC_ACTUATOR_LIMIT_MAX_VOLTAGE*xPidController->Parameters.Ki;
//	xPidController->Errors.Integral = SJC_Helpers_RealClamp(xPidController->Errors.Integral, -limit, limit);
	xPidController->Errors.PrevError = xPidController->Errors.ActualError;

	xPidController->ControlAction = xPidController->Errors.ActualError*xPidController->Parameters.Kp;
	xPidController->ControlAction += xPidController->Errors.Integral*xPidController->Parameters.Ki;
	xPidController->ControlAction += xPidController->Errors.Derivative*xPidController->Parameters.Kd;

	return xPidController->ControlAction;
}

float SJC_PidController_GetError(void)
{
	return xPidController->Errors.ActualError;
}

float SJC_PidController_GetReference(void)
{
	return xPidController->Reference;
}

void SJC_PidController_SetReference(const float reference)
{
	xPidController->Reference = reference;
}

eSjcReturnType_t SJC_PidController_GetControlParameters(xControllerParameters_t * Parameters)
{
	*Parameters = xPidController->Parameters;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_PidController_SetControlParameters(const xControllerParameters_t Parameters)
{
	xPidController->Parameters = Parameters;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_PidController_GetControlUpdateRateHz(float * config)
{
	*config = xPidController->Rate;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_PidController_SetControlUpdateRateHz(const float config)
{
	xPidController->Rate = SJC_Helpers_RealClamp(config, SJC_CONTROLLER_MIN_UPDATE_RATE_HZ, SJC_CONTROLLER_MAX_UPDATE_RATE_HZ);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_PidController_GetControlUpdatePeriodMs(uint32_t * config)
{
	*config = ((uint32_t) (1000/xPidController->Rate));

	return SJC_RET_OK;
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void reset_errors(void)
{
	xPidController->Errors = xControllerErrorsEmptyObj;
}

void reset_parameters(void)
{
	xPidController->Parameters = xControllerParametersEmptyObj;
}

// ------------------------------------------------------------------- //
