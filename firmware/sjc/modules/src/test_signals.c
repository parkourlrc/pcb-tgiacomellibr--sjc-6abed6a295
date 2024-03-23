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
#include "configs.h"
#include "test_signals.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static eSjcInputSignalState_t xSjcInputSignalState = INPUT_SIGNAL_STATE_INIT_CONFIG;
static eSjcInputType_t xSjcSignalType = INPUT_TYPE_NONE;

static float xSjcSignalStep = 0.0f;
static float xSjcSignalValue = 0.0f;
static uint32_t xSjcSignalCounter = 0;
static uint32_t xSjcSignalRepeatCounter = 0;
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //
void TestSignals_setState(const eSjcInputSignalState_t xSjcInputSignalState);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void SJC_TestSignals_Init(void)
{
	xSjcInputSignalState = INPUT_SIGNAL_STATE_INIT_CONFIG;
	xSjcSignalType = INPUT_TYPE_NONE;
}

void SJC_TestSignals_Reset(void)
{
	xSjcInputSignalState = INPUT_SIGNAL_STATE_INIT_CONFIG;
	xSjcSignalType = INPUT_TYPE_NONE;
}

eSjcReturnType_t SJC_TestSignals_SetType(const eSjcInputType_t type)
{
	xSjcSignalType = type;

	switch (xSjcSignalType)
	{
	case INPUT_TYPE_PRBS:
		xSjcSignalStep = 0.0f;
		xSjcSignalValue = 0.0f;
		xSjcSignalCounter = 0;
		xSjcSignalRepeatCounter = 0;
		break;
	case INPUT_TYPE_TRIANG:
		xSjcSignalStep = (1 / 100.0f);
		xSjcSignalValue = 0.0f;
		xSjcSignalCounter = 0;
		xSjcSignalRepeatCounter = 0;
		break;
	case INPUT_TYPE_PULSE_FAST:
		xSjcSignalStep = 0.0f;
		xSjcSignalValue = 1.0f;
		xSjcSignalCounter = 0;
		xSjcSignalRepeatCounter = 0;
		break;
	case INPUT_TYPE_PULSE_SLOW:
		xSjcSignalStep = 0.0f;
		xSjcSignalValue = 1.0f;
		xSjcSignalCounter = 0;
		xSjcSignalRepeatCounter = 0;
		break;
	default:
		xSjcSignalStep = 0.0f;
		xSjcSignalValue = 0.0f;
		xSjcSignalCounter = 0;
		xSjcSignalRepeatCounter = 0;
		break;
	}

	TestSignals_setState(INPUT_SIGNAL_STATE_RUNNING);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_TestSignals_GetState(eSjcInputSignalState_t *state)
{
	*state = xSjcInputSignalState;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_TestSignals_GetValue(float *value)
{
	if (INPUT_SIGNAL_STATE_FINISHED == xSjcInputSignalState)
	{
		*value = ZERO_FLOAT;
	}
	else
	{
		xSjcSignalCounter += (xSjcSignalType == INPUT_TYPE_NONE) ? 0 : 1;

		switch (xSjcSignalType)
		{

		case INPUT_TYPE_TRIANG:
			if (xSjcSignalCounter == 100)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 300)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 500)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 700)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 900)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 1100)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 1300)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 1500)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 1700)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 1900)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 2000)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter > 2000)
			{
				xSjcSignalStep = ZERO_FLOAT;
				xSjcSignalValue = ZERO_FLOAT;
				TestSignals_setState(INPUT_SIGNAL_STATE_FINISHED);
			}
			xSjcSignalValue += xSjcSignalStep;
			break;

		case INPUT_TYPE_PULSE_FAST:
			if (xSjcSignalCounter == 150)
			{
				xSjcSignalValue *= -1;
				xSjcSignalCounter = 0;
				xSjcSignalRepeatCounter++;
			}

			if (xSjcSignalRepeatCounter == 20)
			{
				xSjcSignalValue = ZERO_FLOAT;
				TestSignals_setState(INPUT_SIGNAL_STATE_FINISHED);
			}
			break;

		case INPUT_TYPE_PULSE_SLOW:
			if (xSjcSignalCounter == 300)
			{
				xSjcSignalValue *= -1;
				xSjcSignalCounter = 0;
				xSjcSignalRepeatCounter++;
			}

			if (xSjcSignalRepeatCounter == 20)
			{
				xSjcSignalValue = ZERO_FLOAT;
				TestSignals_setState(INPUT_SIGNAL_STATE_FINISHED);
			}
			break;

			//		case INPUT_TYPE_PRBS:
			//			xSjcSignalValue = ZERO_FLOAT;
			//			break;

		default:
			xSjcSignalValue = ZERO_FLOAT;
			TestSignals_setState(INPUT_SIGNAL_STATE_FINISHED);
			break;
		}

		*value = xSjcSignalValue;
	}

	return SJC_RET_OK;
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void TestSignals_setState(const eSjcInputSignalState_t state)
{
	xSjcInputSignalState = state;
}
// ------------------------------------------------------------------- //
