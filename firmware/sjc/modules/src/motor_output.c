/*
 * motor_output.c
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  Power output enable/disable GPIO control for motor drivers.
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "main.h"
// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "base/include/configs.h"
#include "modules/include/motor_output.h"
#include "system_types/include/helpers.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static xSjcMotorOutput_t MotorOutput;

// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void MotorOutput_init(void)
{
#if SJC_HW_VERSION_MAJOR > 1
	MotorOutput.OutputEnablePort.Pin = SJC_ACTUATOR_POWER_OUTPUT_ENABLE_PIN;
	MotorOutput.OutputEnablePort.Port = SJC_ACTUATOR_POWER_OUTPUT_ENABLE_PORT;
#endif

	MotorOutput_disable();
}

void MotorOutput_enable(void)
{
#if SJC_HW_VERSION_MAJOR > 1
	LL_GPIO_ResetOutputPin(MotorOutput.OutputEnablePort.Port, MotorOutput.OutputEnablePort.Pin);
#endif

	MotorOutput.State = true;
}

void MotorOutput_disable(void)
{
#if SJC_HW_VERSION_MAJOR > 1
	LL_GPIO_SetOutputPin(MotorOutput.OutputEnablePort.Port, MotorOutput.OutputEnablePort.Pin);
#endif

	MotorOutput.State = false;
}

eSjcReturnType_t MotorOutput_getState(bool *state)
{
	SJC_RETURN_IF_NULL_LOG_RET(state, "MotorOutput_getState: state");

	*state = MotorOutput.State;
	return SJC_RET_OK;
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
