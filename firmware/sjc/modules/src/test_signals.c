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
static eSjcInputSignalState_t xSjcInputSignalState = INPUT_SIGNAL_STATE_INIT;
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
	xSjcInputSignalState = INPUT_SIGNAL_STATE_INIT;
	xSjcSignalType = INPUT_TYPE_NONE;
}

void SJC_TestSignals_Reset(void)
{
	xSjcInputSignalState = INPUT_SIGNAL_STATE_CONFIG;
	xSjcSignalType = INPUT_TYPE_NONE;
}

eSjcReturnType_t SJC_TestSignals_SetType(const eSjcInputType_t type)
{
	xSjcSignalType = type;

	switch (xSjcSignalType) {
	case INPUT_TYPE_PRBS:
		xSjcSignalStep = 0.0f;
		xSjcSignalValue = 0.0f;
		xSjcSignalCounter = 0;
		xSjcSignalRepeatCounter = 0;
		break;
	case INPUT_TYPE_TRIANG:
		xSjcSignalStep = (1/2000.0f);
		xSjcSignalValue = 0.0f;
		xSjcSignalCounter = 0;
		xSjcSignalRepeatCounter = 0;
		break;
	case INPUT_TYPE_PULSE:
		xSjcSignalStep = 0.0f;
		xSjcSignalValue = 0.0f;
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

eSjcReturnType_t SJC_TestSignals_GetState(eSjcInputSignalState_t * state)
{
	*state = xSjcInputSignalState;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_TestSignals_GetValue(float * value)
{
	if (INPUT_SIGNAL_STATE_FINISHED != xSjcInputSignalState)
	{
		*value = ZERO_FLOAT;
		TestSignals_setState(INPUT_SIGNAL_STATE_FINISHED);
	}
	else
	{
		switch (xSjcSignalType) {
		case INPUT_TYPE_PRBS:
			xSjcSignalValue = ZERO_FLOAT;
			break;
		case INPUT_TYPE_TRIANG:
			if (xSjcSignalCounter < 2000) {
				xSjcSignalValue += xSjcSignalStep;
			} else if (xSjcSignalCounter < 6000) {
				xSjcSignalValue -= xSjcSignalStep;
			} else if (xSjcSignalCounter < 8000) {
				xSjcSignalValue += xSjcSignalStep;
			} else {
				xSjcSignalValue = ZERO_FLOAT;
				TestSignals_setState(INPUT_SIGNAL_STATE_FINISHED);
			}

			break;
		case INPUT_TYPE_PULSE:
			if (xSjcSignalCounter < 2000) {
				xSjcSignalValue = 1.0f;
			} else if (xSjcSignalCounter < 4000) {
				xSjcSignalValue *= -1;
			} else if (xSjcSignalCounter < 6000) {
				xSjcSignalValue *= -1;
			} else if (xSjcSignalCounter < 8000) {
				xSjcSignalValue *= -1;
			} else {
				xSjcSignalValue = ZERO_FLOAT;
				TestSignals_setState(INPUT_SIGNAL_STATE_FINISHED);
			}

			break;
		default:
			xSjcSignalValue = ZERO_FLOAT;
			TestSignals_setState(INPUT_SIGNAL_STATE_CONFIG);
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
