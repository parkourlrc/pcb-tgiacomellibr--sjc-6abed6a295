/*
 * digital_interfaces.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */


// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "adc.h"
// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "types.h"

#include "digital_interfaces.h"
// ------------------------------------------------------------------- //


// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static bool xInputGpioState[SJC_INPUTS_SIZE];
static bool xOutputGpioState[SJC_OUTPUTS_SIZE];
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void SJC_DigitalInterfaces_init(void)
{
	for (size_t i = 0; i < SJC_OUTPUTS_SIZE; i++)
	{
		xOutputGpioState[i] = false;
		SJC_DigitalInterfaces_DisableOutput(i);
	}

	xInputGpioState[SJC_ISOLATED_INPUT_A] = LL_GPIO_IsInputPinSet(GP_ISO_IN_A_GPIO_Port, GP_ISO_IN_A_Pin);
	xInputGpioState[SJC_ISOLATED_INPUT_B] = LL_GPIO_IsInputPinSet(GP_ISO_IN_B_GPIO_Port, GP_ISO_IN_B_Pin);
	xInputGpioState[SJC_JOINT_SWITCH_L] = LL_GPIO_IsInputPinSet(JOINT_SW_L_GPIO_Port, JOINT_SW_L_Pin);
	xInputGpioState[SJC_JOINT_SWITCH_R] = LL_GPIO_IsInputPinSet(JOINT_SW_R_GPIO_Port, JOINT_SW_R_Pin);
}

bool SJC_DigitalInterfaces_GetInputState(eSjcDigitalInputs_t Port)
{
	bool ret = false;

	switch (Port) {
	case SJC_ISOLATED_INPUT_A:
		ret = xInputGpioState[SJC_ISOLATED_INPUT_A];
		break;
	case SJC_ISOLATED_INPUT_B:
		ret = xInputGpioState[SJC_ISOLATED_INPUT_B];
		break;
	case SJC_JOINT_SWITCH_L:
		ret = xInputGpioState[SJC_JOINT_SWITCH_L];
		break;
	default: // SJC_JOINT_SWITCH_R
		ret = xInputGpioState[SJC_JOINT_SWITCH_R];
		break;
	}
	return ret;
}

void SJC_DigitalInterfaces_EnableOutput(eSjcDigitalOutputs_t Port)
{
	switch (Port) {
	case SJC_HEARTBEAT_OUTPUT:
		LL_GPIO_SetOutputPin(SYS_HEARTBEAT_GPIO_Port, SYS_HEARTBEAT_Pin);
		xOutputGpioState[SJC_HEARTBEAT_OUTPUT] = true;
		break;
	case SJC_DIGITAL_OUTPUT_B:
		LL_GPIO_SetOutputPin(GP_OUT_B_GPIO_Port, GP_OUT_B_Pin);
		xOutputGpioState[SJC_DIGITAL_OUTPUT_B] = true;
		break;
	default: // SJC_DIGITAL_OUTPUT_A:
		LL_GPIO_SetOutputPin(GP_OUT_A_GPIO_Port, GP_OUT_A_Pin);
		xOutputGpioState[SJC_DIGITAL_OUTPUT_A] = true;
		break;
	}
}

void SJC_DigitalInterfaces_DisableOutput(eSjcDigitalOutputs_t Port)
{
	switch (Port) {
	case SJC_HEARTBEAT_OUTPUT:
		LL_GPIO_ResetOutputPin(SYS_HEARTBEAT_GPIO_Port, SYS_HEARTBEAT_Pin);
		xOutputGpioState[SJC_HEARTBEAT_OUTPUT] = false;
		break;
	case SJC_DIGITAL_OUTPUT_B:
		LL_GPIO_ResetOutputPin(GP_OUT_B_GPIO_Port, GP_OUT_B_Pin);
		xOutputGpioState[SJC_DIGITAL_OUTPUT_B] = false;
		break;
	default: // SJC_DIGITAL_OUTPUT_A:
		LL_GPIO_ResetOutputPin(GP_OUT_A_GPIO_Port, GP_OUT_A_Pin);
		xOutputGpioState[SJC_DIGITAL_OUTPUT_A] = false;
		break;
	}
}

void SJC_DigitalInterfaces_ToggleOutputState(eSjcDigitalOutputs_t Port)
{
	switch (Port) {
	case SJC_HEARTBEAT_OUTPUT:
		LL_GPIO_TogglePin(SYS_HEARTBEAT_GPIO_Port, SYS_HEARTBEAT_Pin);
		xOutputGpioState[SJC_HEARTBEAT_OUTPUT] ^= true;
		break;
	case SJC_DIGITAL_OUTPUT_B:
		LL_GPIO_TogglePin(GP_OUT_B_GPIO_Port, GP_OUT_B_Pin);
		xOutputGpioState[SJC_DIGITAL_OUTPUT_B] ^= true;
		break;
	default: // SJC_DIGITAL_OUTPUT_A:
		LL_GPIO_TogglePin(GP_OUT_A_GPIO_Port, GP_OUT_A_Pin);
		xOutputGpioState[SJC_DIGITAL_OUTPUT_A] ^= true;
		break;
	}
}

bool SJC_DigitalInterfaces_GetOutputState(eSjcDigitalOutputs_t Port)
{
	bool ret = false;
	switch (Port) {
	case SJC_HEARTBEAT_OUTPUT:
		ret = xOutputGpioState[SJC_HEARTBEAT_OUTPUT];
		break;
	case SJC_DIGITAL_OUTPUT_B:
		ret = xOutputGpioState[SJC_DIGITAL_OUTPUT_B];
		break;
	default: // SJC_DIGITAL_OUTPUT_A:
		ret = xOutputGpioState[SJC_DIGITAL_OUTPUT_A];
		break;
	}
	return ret;
}


// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
