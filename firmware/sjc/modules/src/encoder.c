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
osSemaphoreId_t xEncoderInterfacesSemprHandle = NULL;
StaticSemaphore_t EncoderInterfacesSemaphoreControlBlock;
const osSemaphoreAttr_t EncoderInterfacesSemaphore_attributes = {
		.name = "EncoderInterfacesSemaphore",
		.cb_mem = &EncoderInterfacesSemaphoreControlBlock,
		.cb_size = sizeof(EncoderInterfacesSemaphoreControlBlock),
};
// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static xSjcEncoder_t Encoder[SJC_JOINTS_SIZE];
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void SJC_Encoder_init(void)
{
	xEncoderInterfacesSemprHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &EncoderInterfacesSemaphore_attributes);

	if (xEncoderInterfacesSemprHandle == NULL)
	{
		SJC_DebugOutput_SystemPrintf("\r\n[OS FAULT] Encoder interfaces semaphore not created...");
	} else
	{
		osSemaphoreRelease(xEncoderInterfacesSemprHandle);
	}

	Encoder[SJC_ACTUATOR_JOINT].Instance = SJC_ACTUATOR_JOINT_TIMER_INSTANCE;
	Encoder[SJC_AUX_JOINT_A].Instance = SJC_AUX_JOINT_A_TIMER_INSTANCE;
	Encoder[SJC_AUX_JOINT_B].Instance = SJC_AUX_JOINT_B_TIMER_INSTANCE;

	Encoder[SJC_ACTUATOR_JOINT].Counts = 0;
	Encoder[SJC_AUX_JOINT_A].Counts = 0;
	Encoder[SJC_AUX_JOINT_B].Counts = 0;

	Encoder[SJC_ACTUATOR_JOINT].Resolution_PPR = SJC_MIN_ENCODER_RESOLUTION_PPR;
	Encoder[SJC_AUX_JOINT_A].Resolution_PPR = SJC_MIN_ENCODER_RESOLUTION_PPR;
	Encoder[SJC_AUX_JOINT_B].Resolution_PPR = SJC_MIN_ENCODER_RESOLUTION_PPR;

	SJC_Encoder_Reset(SJC_ACTUATOR_JOINT);
	SJC_Encoder_Reset(SJC_AUX_JOINT_A);
	SJC_Encoder_Reset(SJC_AUX_JOINT_B);

}

void SJC_Encoder_Reset(eSjcJoints_t Joint)
{
	LL_TIM_SetCounter(Encoder[Joint].Instance, 0);
	Encoder[Joint].Counts = 0;
}

void SJC_Encoder_Enable(eSjcJoints_t Joint)
{
	LL_TIM_EnableCounter(Encoder[Joint].Instance);
}

void SJC_Encoder_Disable(eSjcJoints_t Joint)
{
	LL_TIM_DisableCounter(Encoder[Joint].Instance);
}

eSjcReturnType_t SJC_Encoder_GetCounter(eSjcJoints_t Joint, int32_t * counts)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	configASSERT( xEncoderInterfacesSemprHandle != NULL );

	if (osOK == osSemaphoreAcquire(xEncoderInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		Encoder[Joint].Counts = LL_TIM_GetCounter(Encoder[Joint].Instance);
		*counts = Encoder[Joint].Counts;
		ret = SJC_RET_OK;

		osSemaphoreRelease(xEncoderInterfacesSemprHandle);
	}

	return ret;
}

eSjcReturnType_t SJC_Encoder_GetAngularDisplacement(eSjcJoints_t Joint, float * angular_dp)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	configASSERT( xEncoderInterfacesSemprHandle != NULL );

	if (osOK == osSemaphoreAcquire(xEncoderInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		Encoder[Joint].Counts = LL_TIM_GetCounter(Encoder[Joint].Instance);
		*angular_dp = 2*M_PI*( 1.0f*Encoder[Joint].Counts)/( 1.0f*GET_ENCODER_QUADRATURE_RESOLUTION_PPR(Encoder[Joint].Resolution_PPR) );
		ret = SJC_RET_OK;

		osSemaphoreRelease(xEncoderInterfacesSemprHandle);
	}

	return ret;
}

eSjcReturnType_t SJC_Encoder_GetResolution(eSjcJoints_t Joint, uint16_t * resolution)
{
	*resolution = Encoder[Joint].Resolution_PPR;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_Encoder_SetResolution(eSjcJoints_t Joint, const uint16_t resolution)
{
	Encoder[Joint].Resolution_PPR = resolution;

	return SJC_RET_OK;
}


// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
