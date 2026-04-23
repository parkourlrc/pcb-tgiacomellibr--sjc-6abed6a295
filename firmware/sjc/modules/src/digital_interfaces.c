/*
 * digital_interfaces.c
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  GPIO read/write interface for digital inputs and outputs.
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "modules/include/digital_interfaces.h"
#include "base/include/configs.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static xSjcDigitalPort_t xDigitalInputInterfaces[SJC_DIGITAL_INPUTS_SIZE];
static xSjcDigitalPort_t xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUTS_SIZE];
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void DigitalInterfaces_init(void)
{
	// Input initialization
	xDigitalInputInterfaces[SJC_JOINT_SWITCH_A] = (xSjcDigitalPort_t){SJC_DIGITAL_INPUT_PORT, SJC_JOINT_SWITCH_A_PORT, SJC_JOINT_SWITCH_A_PIN};
	xDigitalInputInterfaces[SJC_JOINT_SWITCH_B] = (xSjcDigitalPort_t){SJC_DIGITAL_INPUT_PORT, SJC_JOINT_SWITCH_B_PORT, SJC_JOINT_SWITCH_B_PIN};
	xDigitalInputInterfaces[SJC_ISOLATED_INPUT_A] = (xSjcDigitalPort_t){SJC_DIGITAL_INPUT_PORT, SJC_ISOLATED_INPUT_A_PORT, SJC_ISOLATED_INPUT_A_PIN};
	xDigitalInputInterfaces[SJC_ISOLATED_INPUT_B] = (xSjcDigitalPort_t){SJC_DIGITAL_INPUT_PORT, SJC_ISOLATED_INPUT_B_PORT, SJC_ISOLATED_INPUT_B_PIN};

#if SJC_HW_VERSION_MAJOR > 1
	xDigitalInputInterfaces[SJC_NET_ADDR_A_INPUT] = (xSjcDigitalPort_t){SJC_DIGITAL_INPUT_PORT, SJC_NET_ADDR_A_INPUT_PORT, SJC_NET_ADDR_A_INPUT_PIN};
	xDigitalInputInterfaces[SJC_NET_ADDR_B_INPUT] = (xSjcDigitalPort_t){SJC_DIGITAL_INPUT_PORT, SJC_NET_ADDR_B_INPUT_PORT, SJC_NET_ADDR_B_INPUT_PIN};
	xDigitalInputInterfaces[SJC_NET_ADDR_C_INPUT] = (xSjcDigitalPort_t){SJC_DIGITAL_INPUT_PORT, SJC_NET_ADDR_C_INPUT_PORT, SJC_NET_ADDR_C_INPUT_PIN};
#endif

	// Output initialization
	xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_A] = (xSjcDigitalPort_t){SJC_DIGITAL_OUTPUT_PORT, SJC_DIGITAL_OUTPUT_A_PORT, SJC_DIGITAL_OUTPUT_A_PIN};
	xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_B] = (xSjcDigitalPort_t){SJC_DIGITAL_OUTPUT_PORT, SJC_DIGITAL_OUTPUT_B_PORT, SJC_DIGITAL_OUTPUT_B_PIN};
	xDigitalOutputInterfaces[SJC_HEARTBEAT_OUTPUT] = (xSjcDigitalPort_t){SJC_DIGITAL_OUTPUT_PORT, SJC_HEARTBEAT_OUTPUT_PORT, SJC_HEARTBEAT_OUTPUT_PIN};
}

eSjcReturnType_t DigitalInterfaces_readInputState(const eSjcDigitalInputs_t input, bool *state)
{
	if (state == NULL || input >= SJC_DIGITAL_INPUTS_SIZE)
	{
		return SJC_RET_NOK;
	}

	*state = LL_GPIO_IsInputPinSet(xDigitalInputInterfaces[input].Port, xDigitalInputInterfaces[input].Pin);
	return SJC_RET_OK;
}

eSjcReturnType_t DigitalInterfaces_readOutputState(const eSjcDigitalOutputs_t output, bool *state)
{
	if (state == NULL || output >= SJC_DIGITAL_OUTPUTS_SIZE)
	{
		return SJC_RET_NOK;
	}

	*state = LL_GPIO_IsOutputPinSet(xDigitalOutputInterfaces[output].Port, xDigitalOutputInterfaces[output].Pin);
	return SJC_RET_OK;
}

eSjcReturnType_t DigitalInterfaces_toggleOutputState(const eSjcDigitalOutputs_t output)
{
	if (output >= SJC_DIGITAL_OUTPUTS_SIZE)
	{
		return SJC_RET_NOK;
	}

	LL_GPIO_TogglePin(xDigitalOutputInterfaces[output].Port, xDigitalOutputInterfaces[output].Pin);
	return SJC_RET_OK;
}

eSjcReturnType_t DigitalInterfaces_writeOutputState(const eSjcDigitalOutputs_t output, const bool state)
{
	if (output >= SJC_DIGITAL_OUTPUTS_SIZE)
	{
		return SJC_RET_NOK;
	}

	if (state)
		LL_GPIO_SetOutputPin(xDigitalOutputInterfaces[output].Port, xDigitalOutputInterfaces[output].Pin);
	else
		LL_GPIO_ResetOutputPin(xDigitalOutputInterfaces[output].Port, xDigitalOutputInterfaces[output].Pin);

	return SJC_RET_OK;
}

eSjcReturnType_t DigitalInterfaces_readInputStates(uint32_t *states)
{
	if (states == NULL)
	{
		return SJC_RET_NOK;
	}
	*states = 0;

	for (uint8_t i = 0; i < SJC_DIGITAL_INPUTS_SIZE; ++i)
	{
		if (LL_GPIO_IsInputPinSet(xDigitalInputInterfaces[i].Port, xDigitalInputInterfaces[i].Pin))
		{
			*states |= (1UL << i);
		}
	}
	return SJC_RET_OK;
}

eSjcReturnType_t DigitalInterfaces_readOutputStates(uint32_t *states)
{
	if (states == NULL)
	{
		return SJC_RET_NOK;
	}
	*states = 0;

	for (uint8_t i = 0; i < SJC_DIGITAL_OUTPUTS_SIZE; ++i)
	{
		if (LL_GPIO_IsOutputPinSet(xDigitalOutputInterfaces[i].Port, xDigitalOutputInterfaces[i].Pin))
		{
			*states |= (1UL << i);
		}
	}
	return SJC_RET_OK;
}

eSjcReturnType_t DigitalInterfaces_writeOutputStates(const uint32_t states)
{
	for (uint8_t i = 0; i < SJC_DIGITAL_OUTPUTS_SIZE; ++i)
	{
		if ((states >> i) & 0x01)
			LL_GPIO_SetOutputPin(xDigitalOutputInterfaces[i].Port, xDigitalOutputInterfaces[i].Pin);
		else
			LL_GPIO_ResetOutputPin(xDigitalOutputInterfaces[i].Port, xDigitalOutputInterfaces[i].Pin);
	}
	return SJC_RET_OK;
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
