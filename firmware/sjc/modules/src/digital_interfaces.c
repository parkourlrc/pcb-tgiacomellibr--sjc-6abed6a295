/*
 * digital_interfaces.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */


// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "digital_interfaces.h"

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

// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t SJC_DigitalInterfaces_init(xSjcDigitalPort_t *Interfaces, const eSjcDigitalPortType_t type)
{
	eSjcReturnType_t ret = SJC_RET_OK;

	switch (type) {
	case SJC_DIGITAL_INPUT_PORT:
		Interfaces[SJC_JOINT_SWITCH_A].State = false;
		Interfaces[SJC_JOINT_SWITCH_A].Type = SJC_DIGITAL_INPUT_PORT;
		Interfaces[SJC_JOINT_SWITCH_A].Pin = SJC_JOINT_SWITCH_A_PIN;
		Interfaces[SJC_JOINT_SWITCH_A].Port = SJC_JOINT_SWITCH_A_PORT;

		Interfaces[SJC_JOINT_SWITCH_B].State = false;
		Interfaces[SJC_JOINT_SWITCH_B].Type = SJC_DIGITAL_INPUT_PORT;
		Interfaces[SJC_JOINT_SWITCH_B].Pin = SJC_JOINT_SWITCH_B_PIN;
		Interfaces[SJC_JOINT_SWITCH_B].Port = SJC_JOINT_SWITCH_B_PORT;

		Interfaces[SJC_ISOLATED_INPUT_A].State = false;
		Interfaces[SJC_ISOLATED_INPUT_A].Type = SJC_DIGITAL_INPUT_PORT;
		Interfaces[SJC_ISOLATED_INPUT_A].Pin = SJC_ISOLATED_INPUT_A_PIN;
		Interfaces[SJC_ISOLATED_INPUT_A].Port = SJC_ISOLATED_INPUT_A_PORT;

		Interfaces[SJC_ISOLATED_INPUT_B].State = false;
		Interfaces[SJC_ISOLATED_INPUT_B].Type = SJC_DIGITAL_INPUT_PORT;
		Interfaces[SJC_ISOLATED_INPUT_B].Pin = SJC_ISOLATED_INPUT_B_PIN;
		Interfaces[SJC_ISOLATED_INPUT_B].Port = SJC_ISOLATED_INPUT_B_PORT;
		break;
	case SJC_DIGITAL_OUTPUT_PORT:
		Interfaces[SJC_DIGITAL_OUTPUT_A].State = false;
		Interfaces[SJC_DIGITAL_OUTPUT_A].Type = SJC_DIGITAL_OUTPUT_PORT;
		Interfaces[SJC_DIGITAL_OUTPUT_A].Pin = SJC_DIGITAL_OUTPUT_A_PIN;
		Interfaces[SJC_DIGITAL_OUTPUT_A].Port = SJC_DIGITAL_OUTPUT_A_PORT;

		Interfaces[SJC_DIGITAL_OUTPUT_B].State = false;
		Interfaces[SJC_DIGITAL_OUTPUT_B].Type = SJC_DIGITAL_OUTPUT_PORT;
		Interfaces[SJC_DIGITAL_OUTPUT_B].Pin = SJC_DIGITAL_OUTPUT_B_PIN;
		Interfaces[SJC_DIGITAL_OUTPUT_B].Port = SJC_DIGITAL_OUTPUT_B_PORT;

		Interfaces[SJC_HEARTBEAT_OUTPUT].State = false;
		Interfaces[SJC_HEARTBEAT_OUTPUT].Type = SJC_DIGITAL_OUTPUT_PORT;
		Interfaces[SJC_HEARTBEAT_OUTPUT].Pin = SJC_HEARTBEAT_OUTPUT_PIN;
		Interfaces[SJC_HEARTBEAT_OUTPUT].Port = SJC_HEARTBEAT_OUTPUT_PORT;
		break;
	default:
		ret = SJC_RET_NOK;
		break;
	}

	return ret;
}

eSjcReturnType_t SJC_DigitalInterfaces_ReadInputState(xSjcDigitalPort_t *Interface)
{
	Interface->State = LL_GPIO_IsInputPinSet(Interface->Port, Interface->Pin);
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_DigitalInterfaces_ReadOutputState(xSjcDigitalPort_t *Interface)
{
	Interface->State = LL_GPIO_IsOutputPinSet(Interface->Port, Interface->Pin);
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_DigitalInterfaces_ToggleOutputState(xSjcDigitalPort_t *Interface)
{
	LL_GPIO_TogglePin(Interface->Port, Interface->Pin);
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_DigitalInterfaces_WriteOutputState(xSjcDigitalPort_t *Interface, const bool state)
{
	if (state == true)
	{
		LL_GPIO_SetOutputPin(Interface->Port, Interface->Pin);
		Interface->State = true;
	}
	else
	{
		LL_GPIO_ResetOutputPin(Interface->Port, Interface->Pin);
		Interface->State = false;
	}
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_DigitalInterfaces_ReadInputStates(xSjcDigitalPort_t *Interfaces, uint16_t * states)
{
	*states = 0;

	bool pin_state = LL_GPIO_IsInputPinSet(Interfaces[SJC_ISOLATED_INPUT_A].Port, Interfaces[SJC_ISOLATED_INPUT_A].Pin);
	Interfaces[SJC_ISOLATED_INPUT_A].State = pin_state;
	*states |= ( pin_state <<  SJC_ISOLATED_INPUT_A );

	pin_state = LL_GPIO_IsInputPinSet(Interfaces[SJC_ISOLATED_INPUT_B].Port, Interfaces[SJC_ISOLATED_INPUT_B].Pin);
	Interfaces[SJC_ISOLATED_INPUT_B].State = pin_state;
	*states |= ( pin_state <<  SJC_ISOLATED_INPUT_B );

	pin_state = LL_GPIO_IsInputPinSet(Interfaces[SJC_JOINT_SWITCH_A].Port, Interfaces[SJC_JOINT_SWITCH_A].Pin);
	Interfaces[SJC_JOINT_SWITCH_A].State = pin_state;
	*states |= ( pin_state <<  SJC_JOINT_SWITCH_A );

	pin_state = LL_GPIO_IsInputPinSet(Interfaces[SJC_JOINT_SWITCH_B].Port, Interfaces[SJC_JOINT_SWITCH_B].Pin);
	Interfaces[SJC_JOINT_SWITCH_B].State = pin_state;
	*states |= ( pin_state <<  SJC_JOINT_SWITCH_B );

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_DigitalInterfaces_ReadOutputStates(xSjcDigitalPort_t *Interfaces)
{
	Interfaces[SJC_DIGITAL_OUTPUT_A].State = LL_GPIO_IsInputPinSet(Interfaces[SJC_DIGITAL_OUTPUT_A].Port, Interfaces[SJC_DIGITAL_OUTPUT_A].Pin);
	Interfaces[SJC_DIGITAL_OUTPUT_B].State = LL_GPIO_IsInputPinSet(Interfaces[SJC_DIGITAL_OUTPUT_B].Port, Interfaces[SJC_DIGITAL_OUTPUT_B].Pin);
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_DigitalInterfaces_WriteOutputStates(xSjcDigitalPort_t *Interface, const uint8_t states)
{
	uint8_t states_ = (states & 0b11);

	switch (states_) {
	case 0b00:
		LL_GPIO_ResetOutputPin(Interface[SJC_DIGITAL_OUTPUT_A].Port, Interface[SJC_DIGITAL_OUTPUT_A].Pin);
		LL_GPIO_ResetOutputPin(Interface[SJC_DIGITAL_OUTPUT_B].Port, Interface[SJC_DIGITAL_OUTPUT_B].Pin);
		Interface[SJC_DIGITAL_OUTPUT_A].State = false;
		Interface[SJC_DIGITAL_OUTPUT_B].State = false;
		break;
	case 0b01:
		LL_GPIO_SetOutputPin(Interface[SJC_DIGITAL_OUTPUT_A].Port, Interface[SJC_DIGITAL_OUTPUT_A].Pin);
		LL_GPIO_ResetOutputPin(Interface[SJC_DIGITAL_OUTPUT_B].Port, Interface[SJC_DIGITAL_OUTPUT_B].Pin);
		Interface[SJC_DIGITAL_OUTPUT_A].State = true;
		Interface[SJC_DIGITAL_OUTPUT_B].State = false;
		break;
	case 0b10:
		LL_GPIO_ResetOutputPin(Interface[SJC_DIGITAL_OUTPUT_A].Port, Interface[SJC_DIGITAL_OUTPUT_A].Pin);
		LL_GPIO_SetOutputPin(Interface[SJC_DIGITAL_OUTPUT_B].Port, Interface[SJC_DIGITAL_OUTPUT_B].Pin);
		Interface[SJC_DIGITAL_OUTPUT_A].State = false;
		Interface[SJC_DIGITAL_OUTPUT_B].State = true;
		break;
	case 0b11:
		LL_GPIO_SetOutputPin(Interface[SJC_DIGITAL_OUTPUT_A].Port, Interface[SJC_DIGITAL_OUTPUT_A].Pin);
		LL_GPIO_SetOutputPin(Interface[SJC_DIGITAL_OUTPUT_B].Port, Interface[SJC_DIGITAL_OUTPUT_B].Pin);
		Interface[SJC_DIGITAL_OUTPUT_A].State = true;
		Interface[SJC_DIGITAL_OUTPUT_A].State = true;
		break;
	default:
		break;
	}

	return SJC_RET_OK;
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
