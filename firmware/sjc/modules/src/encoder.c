/*
 * encoder.c
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
#include "encoder.h"

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
eSjcReturnType_t SJC_Encoder_init(xSjcEncoder_t * Encoder, eSjcJoints_t Joint)
{
	eSjcReturnType_t ret = SJC_RET_OK;

	switch (Joint) {
	case SJC_ACTUATOR_JOINT:
		Encoder->Instance = SJC_ACTUATOR_JOINT_TIMER_INSTANCE;
		break;
	case SJC_AUX_JOINT_A:
		Encoder->Instance = SJC_AUX_JOINT_A_TIMER_INSTANCE;
		break;
	case SJC_AUX_JOINT_B:
		Encoder->Instance = SJC_AUX_JOINT_B_TIMER_INSTANCE;
		break;
	default:
		ret = SJC_RET_NOK;
		break;
	}

	if (SJC_RET_OK == ret)
	{
		Encoder->Counts = 0;
		Encoder->Resolution_PPR = SJC_MIN_ENCODER_RESOLUTION_PPR;
		LL_TIM_SetCounter(Encoder->Instance, 0);
	}

	return ret;
}

eSjcReturnType_t SJC_Encoder_Reset(xSjcEncoder_t * Encoder)
{
	LL_TIM_SetCounter(Encoder->Instance, 0);
	Encoder->Counts = 0;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_Encoder_Enable(xSjcEncoder_t * Encoder)
{
	LL_TIM_EnableCounter(Encoder->Instance);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_Encoder_Disable(xSjcEncoder_t * Encoder)
{
	LL_TIM_DisableCounter(Encoder->Instance);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_Encoder_GetQuadraturePulseCounter(xSjcEncoder_t * Encoder, int32_t * Counter)
{
	Encoder->Counts = LL_TIM_GetCounter(Encoder->Instance);
	*Counter = Encoder->Counts;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_Encoder_GetAngularDisplacement(xSjcEncoder_t * Encoder, float * Displacement)
{
	Encoder->Counts = LL_TIM_GetCounter(Encoder->Instance);
	*Displacement = 2*M_PI*( 1.0f*Encoder->Counts)/( 1.0f*GET_ENCODER_QUADRATURE_RESOLUTION_PPR(Encoder->Resolution_PPR) );
//	*Displacement = 2*M_PI*( (float) Encoder->Counts)/( (float) GET_ENCODER_QUADRATURE_RESOLUTION_PPR(Encoder->Resolution_PPR) );
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_Encoder_SetResolution(xSjcEncoder_t * Encoder, const uint16_t Resolution)
{
	Encoder->Resolution_PPR = Resolution;
	return SJC_RET_OK;
}


// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
