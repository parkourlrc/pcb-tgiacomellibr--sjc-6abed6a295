/*
 * encoder.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "modules/include/encoder.h"
#include "base/include/configs.h"
#include "modules/include/debug_output.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //
#define IS_VALID_JOINT(Joint) ((Joint) < SJC_JOINTS_SIZE)
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
static int32_t read_timer_counter(eSjcJoints_t Joint);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void Encoder_init(void)
{
	memset(Encoder, 0, sizeof(Encoder));

	xEncoderInterfacesSemprHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &EncoderInterfacesSemaphore_attributes);

	if (xEncoderInterfacesSemprHandle == NULL)
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Encoder interfaces semaphore not created...");
	}
	else
	{
		osSemaphoreRelease(xEncoderInterfacesSemprHandle);
	}

	Encoder[SJC_ACTUATOR_JOINT].Instance = SJC_ACTUATOR_JOINT_TIMER_INSTANCE;
	Encoder[SJC_AUX_JOINT_A].Instance = SJC_AUX_JOINT_A_TIMER_INSTANCE;
	Encoder[SJC_AUX_JOINT_B].Instance = SJC_AUX_JOINT_B_TIMER_INSTANCE;

	for (uint8_t i = 0; i < SJC_JOINTS_SIZE; i++)
	{
		Encoder[i].Counts = 0;
		Encoder[i].Resolution_PPR = SJC_MIN_ENCODER_RESOLUTION_PPR;
		Encoder[i].radians_per_quadrature_pulse = 1.0f;

		Encoder_reset(i);
	}
}

void Encoder_reset(eSjcJoints_t Joint)
{
	if (IS_VALID_JOINT(Joint))
	{
		configASSERT(xEncoderInterfacesSemprHandle != NULL);
		if (osOK == osSemaphoreAcquire(xEncoderInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
		{
			LL_TIM_SetCounter(Encoder[Joint].Instance, 0);
			Encoder[Joint].Counts = 0;
			osSemaphoreRelease(xEncoderInterfacesSemprHandle);
		}
	}
}

void Encoder_enable(eSjcJoints_t Joint)
{
	if (IS_VALID_JOINT(Joint))
	{
		configASSERT(xEncoderInterfacesSemprHandle != NULL);
		if (osOK == osSemaphoreAcquire(xEncoderInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
		{
			LL_TIM_EnableCounter(Encoder[Joint].Instance);
			osSemaphoreRelease(xEncoderInterfacesSemprHandle);
		}
	}
}

void Encoder_disable(eSjcJoints_t Joint)
{
	if (IS_VALID_JOINT(Joint))
	{
		configASSERT(xEncoderInterfacesSemprHandle != NULL);
		if (osOK == osSemaphoreAcquire(xEncoderInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
		{
			LL_TIM_DisableCounter(Encoder[Joint].Instance);
			osSemaphoreRelease(xEncoderInterfacesSemprHandle);
		}
	}
}

eSjcReturnType_t Encoder_getCounter(eSjcJoints_t Joint, int32_t *counts)
{
	if ((!IS_VALID_JOINT(Joint)) || (counts == NULL))
	{
		return SJC_RET_NOK;
	}

	configASSERT(xEncoderInterfacesSemprHandle != NULL);
	if (osOK == osSemaphoreAcquire(xEncoderInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		Encoder[Joint].Counts = read_timer_counter(Joint);
		*counts = Encoder[Joint].Counts;
		osSemaphoreRelease(xEncoderInterfacesSemprHandle);
	}
	return SJC_RET_OK;
}

eSjcReturnType_t Encoder_getAngularDisplacement(eSjcJoints_t Joint, float *angular_dp)
{
	if ((!IS_VALID_JOINT(Joint)) || (angular_dp == NULL))
	{
		return SJC_RET_NOK;
	}

	configASSERT(xEncoderInterfacesSemprHandle != NULL);
	if (osOK == osSemaphoreAcquire(xEncoderInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		Encoder[Joint].Counts = read_timer_counter(Joint);
		*angular_dp = Encoder[Joint].Counts * Encoder[Joint].radians_per_quadrature_pulse;
		osSemaphoreRelease(xEncoderInterfacesSemprHandle);
	}

	return SJC_RET_OK;
}

eSjcReturnType_t Encoder_getResolution(eSjcJoints_t Joint, uint16_t *resolution)
{
	if ((!IS_VALID_JOINT(Joint)) || (resolution == NULL))
	{
		return SJC_RET_NOK;
	}

	configASSERT(xEncoderInterfacesSemprHandle != NULL);
	if (osOK == osSemaphoreAcquire(xEncoderInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		*resolution = Encoder[Joint].Resolution_PPR;
		osSemaphoreRelease(xEncoderInterfacesSemprHandle);
	}
	return SJC_RET_OK;
}

eSjcReturnType_t Encoder_setResolution(eSjcJoints_t Joint, const uint16_t resolution)
{
	if ((!IS_VALID_JOINT(Joint)) || (resolution == 0))
	{
		return SJC_RET_NOK;
	}

	configASSERT(xEncoderInterfacesSemprHandle != NULL);
	if (osOK == osSemaphoreAcquire(xEncoderInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		Encoder[Joint].Resolution_PPR = resolution;
		Encoder[Joint].radians_per_quadrature_pulse = (2.0f * M_PI) / (1.0f * Encoder[Joint].Resolution_PPR);
		osSemaphoreRelease(xEncoderInterfacesSemprHandle);
	}
	return SJC_RET_OK;
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
static int32_t read_timer_counter(eSjcJoints_t Joint)
{
	int32_t count = 0;

	if (!IS_VALID_JOINT(Joint))
	{
		return 0;
	}

	switch (Joint)
	{
	case SJC_ACTUATOR_JOINT:
		count = (int32_t)LL_TIM_GetCounter(Encoder[Joint].Instance);
		break;
	default:
		count = (int16_t)LL_TIM_GetCounter(Encoder[Joint].Instance);
		break;
	}

	return count;
}

// ------------------------------------------------------------------- //
