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

static float xSjcSignalTimeFactor = 0.0f;
static float xSjcSignalStep = 0.0f;
static float xSjcSignalValue = 0.0f;
static uint32_t xSjcSignalCounter = 0;
static uint32_t xSjcSignalRepeatCounter = 0;
static uint32_t xSjcSeqCounter = 0;
static int8_t xPrbsSeq[] = {-1,-1,-1,-1,-1,-1,-1,-1,-1, 1, 1, 1, 1,-1,-1,-1,
		-1, 1,-1,-1,-1, 1, 1, 1, 1,-1, 1,-1,-1, 1, 1,-1,
		-1, 1,-1,-1, 1,-1,-1,-1,-1, 1,-1, 1, 1, 1, 1,-1,
		-1,-1, 1, 1,-1,-1, 1, 1, 1, 1,-1, 1, 1,-1, 1, 1,
		 1,-1, 1,-1, 1,-1,-1,-1, 1,-1, 1,-1,-1,-1,-1, 1,
		 1,-1, 1, 1,-1, 1,-1,-1,-1, 1, 1,-1,-1,-1, 1, 1,
		 1, 1, 1, 1,-1,-1,-1, 1,-1,-1,-1, 1,-1, 1, 1,-1,
		-1,-1,-1, 1,-1, 1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1,
		 1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1,-1,-1,-1,-1, 1,
		-1, 1,-1,-1, 1,-1, 1, 1, 1, 1, 1,-1,-1, 1,-1,-1,
		-1, 1,-1,-1, 1,-1,-1, 1,-1, 1,-1,-1, 1, 1, 1, 1,
		 1,-1, 1,-1,-1,-1, 1,-1,-1,-1,-1,-1, 1, 1, 1,-1,
		-1,-1,-1, 1, 1,-1,-1, 1,-1, 1, 1,-1,-1, 1,-1, 1,
		-1,-1,-1, 1, 1, 1,-1,-1, 1,-1, 1, 1, 1,-1, 1,-1,
		-1,-1,-1,-1,-1,-1, 1,-1, 1, 1,-1, 1,-1,-1, 1, 1,
		 1,-1, 1,-1, 1, 1,-1,-1, 1, 1, 1,-1,-1, 1, 1, 1};
const static int xPrbsSeqLen = sizeof(xPrbsSeq) / sizeof(xPrbsSeq[0]);
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //
void TestSignals_setState(const eSjcInputSignalState_t xSjcInputSignalState);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void TestSignals_init(void)
{
	xSjcInputSignalState = INPUT_SIGNAL_STATE_INIT_CONFIG;
	xSjcSignalType = INPUT_TYPE_NONE;
}

void TestSignals_reset(void)
{
	xSjcInputSignalState = INPUT_SIGNAL_STATE_INIT_CONFIG;
	xSjcSignalType = INPUT_TYPE_NONE;
}

eSjcReturnType_t TestSignals_setConfig(const eSjcInputType_t type, const float time_factor)
{
	xSjcSignalType = type;

	xSjcSignalTimeFactor = time_factor;

	switch (xSjcSignalType)
	{
	case INPUT_TYPE_PRBS:
		xSjcSignalStep = 0.0f;
		xSjcSignalValue = 0.0f;
		xSjcSeqCounter = 0;
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

eSjcReturnType_t TestSignals_getState(eSjcInputSignalState_t *state)
{
	*state = xSjcInputSignalState;

	return SJC_RET_OK;
}

eSjcReturnType_t TestSignals_getValue(float *value)
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
			if (xSjcSignalCounter == 25*xSjcSignalTimeFactor)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 75*xSjcSignalTimeFactor)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 125*xSjcSignalTimeFactor)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 175*xSjcSignalTimeFactor)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 225*xSjcSignalTimeFactor)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 275*xSjcSignalTimeFactor)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 325*xSjcSignalTimeFactor)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 375*xSjcSignalTimeFactor)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 425*xSjcSignalTimeFactor)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 475*xSjcSignalTimeFactor)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter == 500*xSjcSignalTimeFactor)
			{
				xSjcSignalStep *= -1;
				xSjcSignalRepeatCounter++;
			}
			else if (xSjcSignalCounter >= 500*xSjcSignalTimeFactor)
			{
				xSjcSignalStep = ZERO_FLOAT;
				xSjcSignalValue = ZERO_FLOAT;
				TestSignals_setState(INPUT_SIGNAL_STATE_FINISHED);
			}
			xSjcSignalValue += xSjcSignalStep;
			break;

		case INPUT_TYPE_PULSE_FAST:
			if (xSjcSignalCounter == 37*xSjcSignalTimeFactor)
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
			if (xSjcSignalCounter == 75*xSjcSignalTimeFactor)
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

		case INPUT_TYPE_PRBS:

			if (50 == ++xSjcSignalRepeatCounter)
			{
				xSjcSeqCounter++;
				xSjcSignalRepeatCounter = 0;
			}
			xSjcSignalValue = xPrbsSeq[xSjcSeqCounter];

			if (xPrbsSeqLen == xSjcSeqCounter)
			{
				xSjcSeqCounter = 0;
				xSjcSignalCounter = 0;
				xSjcSignalRepeatCounter = 0;
				xSjcSignalValue = ZERO_FLOAT;
				TestSignals_setState(INPUT_SIGNAL_STATE_FINISHED);
			}

			break;

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
