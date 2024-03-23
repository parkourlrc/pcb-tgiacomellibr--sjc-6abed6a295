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
#include "debug_output.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
osSemaphoreId_t xDigitalInterfacesSemprHandle = NULL;
StaticSemaphore_t DigitalInterfacesSemaphoreControlBlock;
const osSemaphoreAttr_t DigitalInterfacesSemaphore_attributes = {
	.name = "DigitalInterfacesSemaphore",
	.cb_mem = &DigitalInterfacesSemaphoreControlBlock,
	.cb_size = sizeof(DigitalInterfacesSemaphoreControlBlock),
};
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
void SJC_DigitalInterfaces_init(void)
{
	xDigitalInterfacesSemprHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &DigitalInterfacesSemaphore_attributes);

	if (xDigitalInterfacesSemprHandle == NULL)
	{
		SJC_DebugOutput_SystemPrintf("\r\n[OS FAULT] Digital interfaces semaphore not created...");
	}
	else
	{
		osSemaphoreRelease(xDigitalInterfacesSemprHandle);
	}

	xDigitalInputInterfaces[SJC_JOINT_SWITCH_A].Type = SJC_DIGITAL_INPUT_PORT;
	xDigitalInputInterfaces[SJC_JOINT_SWITCH_A].Pin = SJC_JOINT_SWITCH_A_PIN;
	xDigitalInputInterfaces[SJC_JOINT_SWITCH_A].Port = SJC_JOINT_SWITCH_A_PORT;

	xDigitalInputInterfaces[SJC_JOINT_SWITCH_B].Type = SJC_DIGITAL_INPUT_PORT;
	xDigitalInputInterfaces[SJC_JOINT_SWITCH_B].Pin = SJC_JOINT_SWITCH_B_PIN;
	xDigitalInputInterfaces[SJC_JOINT_SWITCH_B].Port = SJC_JOINT_SWITCH_B_PORT;

	xDigitalInputInterfaces[SJC_ISOLATED_INPUT_A].Type = SJC_DIGITAL_INPUT_PORT;
	xDigitalInputInterfaces[SJC_ISOLATED_INPUT_A].Pin = SJC_ISOLATED_INPUT_A_PIN;
	xDigitalInputInterfaces[SJC_ISOLATED_INPUT_A].Port = SJC_ISOLATED_INPUT_A_PORT;

	xDigitalInputInterfaces[SJC_ISOLATED_INPUT_B].Type = SJC_DIGITAL_INPUT_PORT;
	xDigitalInputInterfaces[SJC_ISOLATED_INPUT_B].Pin = SJC_ISOLATED_INPUT_B_PIN;
	xDigitalInputInterfaces[SJC_ISOLATED_INPUT_B].Port = SJC_ISOLATED_INPUT_B_PORT;

	xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_A].Type = SJC_DIGITAL_OUTPUT_PORT;
	xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_A].Pin = SJC_DIGITAL_OUTPUT_A_PIN;
	xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_A].Port = SJC_DIGITAL_OUTPUT_A_PORT;

	xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_B].Type = SJC_DIGITAL_OUTPUT_PORT;
	xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_B].Pin = SJC_DIGITAL_OUTPUT_B_PIN;
	xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_B].Port = SJC_DIGITAL_OUTPUT_B_PORT;

	xDigitalOutputInterfaces[SJC_HEARTBEAT_OUTPUT].Type = SJC_DIGITAL_OUTPUT_PORT;
	xDigitalOutputInterfaces[SJC_HEARTBEAT_OUTPUT].Pin = SJC_HEARTBEAT_OUTPUT_PIN;
	xDigitalOutputInterfaces[SJC_HEARTBEAT_OUTPUT].Port = SJC_HEARTBEAT_OUTPUT_PORT;
}

eSjcReturnType_t SJC_DigitalInterfaces_ReadInputState(const eSjcDigitalInputs_t input, bool *state)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	*state = false;

	configASSERT(xDigitalInterfacesSemprHandle != NULL);

	if (osOK == osSemaphoreAcquire(xDigitalInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		if (input < SJC_DIGITAL_INPUTS_SIZE)
		{
			*state = LL_GPIO_IsInputPinSet(xDigitalInputInterfaces[input].Port, xDigitalInputInterfaces[input].Pin);
			ret = SJC_RET_OK;
		}

		osSemaphoreRelease(xDigitalInterfacesSemprHandle);
	}

	return ret;
}

eSjcReturnType_t SJC_DigitalInterfaces_ReadOutputState(const eSjcDigitalOutputs_t output, bool *state)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	*state = false;

	configASSERT(xDigitalInterfacesSemprHandle != NULL);

	if (osOK == osSemaphoreAcquire(xDigitalInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		if (output < SJC_DIGITAL_OUTPUTS_SIZE)
		{
			*state = LL_GPIO_IsOutputPinSet(xDigitalOutputInterfaces[output].Port, xDigitalOutputInterfaces[output].Pin);
			ret = SJC_RET_OK;
		}

		osSemaphoreRelease(xDigitalInterfacesSemprHandle);
	}

	return ret;
}

eSjcReturnType_t SJC_DigitalInterfaces_ToggleOutputState(const eSjcDigitalOutputs_t output)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	configASSERT(xDigitalInterfacesSemprHandle != NULL);

	if (osOK == osSemaphoreAcquire(xDigitalInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		if (output < SJC_DIGITAL_OUTPUTS_SIZE)
		{
			LL_GPIO_TogglePin(xDigitalOutputInterfaces[output].Port, xDigitalOutputInterfaces[output].Pin);
			ret = SJC_RET_OK;
		}

		osSemaphoreRelease(xDigitalInterfacesSemprHandle);
	}

	return ret;
}

eSjcReturnType_t SJC_DigitalInterfaces_WriteOutputState(const eSjcDigitalOutputs_t output, const bool state)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	configASSERT(xDigitalInterfacesSemprHandle != NULL);

	if (osOK == osSemaphoreAcquire(xDigitalInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		if (output < SJC_DIGITAL_OUTPUTS_SIZE)
		{
			if (state == true)
			{
				LL_GPIO_SetOutputPin(xDigitalOutputInterfaces[output].Port, xDigitalOutputInterfaces[output].Pin);
			}
			else
			{
				LL_GPIO_ResetOutputPin(xDigitalOutputInterfaces[output].Port, xDigitalOutputInterfaces[output].Pin);
			}
			ret = SJC_RET_OK;
		}

		osSemaphoreRelease(xDigitalInterfacesSemprHandle);
	}

	return ret;
}

eSjcReturnType_t SJC_DigitalInterfaces_ReadInputStates(uint32_t *states)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	*states = 0;

	configASSERT(xDigitalInterfacesSemprHandle != NULL);

	if (osOK == osSemaphoreAcquire(xDigitalInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{

		*states |= (LL_GPIO_IsInputPinSet(xDigitalInputInterfaces[SJC_ISOLATED_INPUT_A].Port, xDigitalInputInterfaces[SJC_ISOLATED_INPUT_A].Pin) << SJC_ISOLATED_INPUT_A);
		*states |= (LL_GPIO_IsInputPinSet(xDigitalInputInterfaces[SJC_ISOLATED_INPUT_B].Port, xDigitalInputInterfaces[SJC_ISOLATED_INPUT_B].Pin) << SJC_ISOLATED_INPUT_B);
		*states |= (LL_GPIO_IsInputPinSet(xDigitalInputInterfaces[SJC_JOINT_SWITCH_A].Port, xDigitalInputInterfaces[SJC_JOINT_SWITCH_A].Pin) << SJC_JOINT_SWITCH_A);
		*states |= (LL_GPIO_IsInputPinSet(xDigitalInputInterfaces[SJC_JOINT_SWITCH_B].Port, xDigitalInputInterfaces[SJC_JOINT_SWITCH_B].Pin) << SJC_JOINT_SWITCH_B);
		ret = SJC_RET_OK;

		osSemaphoreRelease(xDigitalInterfacesSemprHandle);
	}

	return ret;
}

eSjcReturnType_t SJC_DigitalInterfaces_ReadOutputStates(uint32_t *states)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	*states = 0;

	configASSERT(xDigitalInterfacesSemprHandle != NULL);

	if (osOK == osSemaphoreAcquire(xDigitalInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		*states |= (LL_GPIO_IsOutputPinSet(xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_A].Port, xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_A].Pin) << SJC_DIGITAL_OUTPUT_A);
		*states |= (LL_GPIO_IsOutputPinSet(xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_B].Port, xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_B].Pin) << SJC_DIGITAL_OUTPUT_B);
		ret = SJC_RET_OK;

		osSemaphoreRelease(xDigitalInterfacesSemprHandle);
	}

	return ret;
}

eSjcReturnType_t SJC_DigitalInterfaces_WriteOutputStates(const uint32_t states)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	configASSERT(xDigitalInterfacesSemprHandle != NULL);

	if (osOK == osSemaphoreAcquire(xDigitalInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		uint8_t states_ = (states & 0b11);

		switch (states_)
		{
		case 0b00:
			LL_GPIO_ResetOutputPin(xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_A].Port, xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_A].Pin);
			LL_GPIO_ResetOutputPin(xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_B].Port, xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_B].Pin);
			break;
		case 0b01:
			LL_GPIO_SetOutputPin(xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_A].Port, xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_A].Pin);
			LL_GPIO_ResetOutputPin(xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_B].Port, xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_B].Pin);
			break;
		case 0b10:
			LL_GPIO_ResetOutputPin(xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_A].Port, xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_A].Pin);
			LL_GPIO_SetOutputPin(xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_B].Port, xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_B].Pin);
			break;
		case 0b11:
			LL_GPIO_SetOutputPin(xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_A].Port, xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_A].Pin);
			LL_GPIO_SetOutputPin(xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_B].Port, xDigitalOutputInterfaces[SJC_DIGITAL_OUTPUT_B].Pin);
			break;
		default:
			break;
		}

		ret = SJC_RET_OK;

		osSemaphoreRelease(xDigitalInterfacesSemprHandle);
	}

	return ret;
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
