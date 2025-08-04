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
static inline int32_t read_encoder_counter_raw(eSjcJoints_t Joint);
static inline xSjcEncoder_t *get_encoder_handle(eSjcJoints_t Joint);

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void Encoder_init(void)
{
	memset(Encoder, 0, sizeof(Encoder));

	xEncoderInterfacesSemprHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &EncoderInterfacesSemaphore_attributes);

	if (xEncoderInterfacesSemprHandle == NULL)
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Encoder semaphore not created, system may become unstable.\n");
	}
	else
	{
		osSemaphoreRelease(xEncoderInterfacesSemprHandle);
	}

	Encoder[SJC_ACTUATOR_JOINT].Instance = SJC_ACTUATOR_JOINT_TIMER_INSTANCE;
	Encoder[SJC_AUX_JOINT_TH_X].Instance = SJC_AUX_JOINT_A_TIMER_INSTANCE;
	Encoder[SJC_AUX_JOINT_TH_Y].Instance = SJC_AUX_JOINT_B_TIMER_INSTANCE;

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
	xSjcEncoder_t *enc = get_encoder_handle(Joint);
	if (enc != NULL)
	{
		configASSERT(xEncoderInterfacesSemprHandle != NULL);
		if (osOK == osSemaphoreAcquire(xEncoderInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
		{
			LL_TIM_SetCounter(enc->Instance, 0);
			enc->Counts = 0;
			osSemaphoreRelease(xEncoderInterfacesSemprHandle);
		}
	}
}

void Encoder_enable(eSjcJoints_t Joint)
{
	xSjcEncoder_t *enc = get_encoder_handle(Joint);
	if (enc != NULL)
	{
		configASSERT(xEncoderInterfacesSemprHandle != NULL);
		if (osOK == osSemaphoreAcquire(xEncoderInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
		{
			LL_TIM_EnableCounter(enc->Instance);
			osSemaphoreRelease(xEncoderInterfacesSemprHandle);
		}
	}
}

void Encoder_disable(eSjcJoints_t Joint)
{
	xSjcEncoder_t *enc = get_encoder_handle(Joint);
	if (enc != NULL)
	{
		configASSERT(xEncoderInterfacesSemprHandle != NULL);
		if (osOK == osSemaphoreAcquire(xEncoderInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
		{
			LL_TIM_DisableCounter(enc->Instance);
			osSemaphoreRelease(xEncoderInterfacesSemprHandle);
		}
	}
}

eSjcReturnType_t Encoder_getCounter(eSjcJoints_t Joint, int32_t *counts)
{
	if ((counts == NULL))
	{
		return SJC_RET_NOK;
	}

	xSjcEncoder_t *enc = get_encoder_handle(Joint);
	if (enc == NULL)
	{
		*counts = 0;
		return SJC_RET_NOK;
	}

	configASSERT(xEncoderInterfacesSemprHandle != NULL);
	if (osOK == osSemaphoreAcquire(xEncoderInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		enc->Counts = read_encoder_counter_raw(Joint);
		*counts = enc->Counts;
		osSemaphoreRelease(xEncoderInterfacesSemprHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t Encoder_getAngularDisplacement(eSjcJoints_t Joint, float *angular_dp)
{
	if (angular_dp == NULL)
	{
		return SJC_RET_NOK;
	}

	xSjcEncoder_t *enc = get_encoder_handle(Joint);
	if (enc == NULL)
	{
		*angular_dp = 0.0f;
		return SJC_RET_NOK;
	}

	configASSERT(xEncoderInterfacesSemprHandle != NULL);
	if (osOK == osSemaphoreAcquire(xEncoderInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		enc->Counts = read_encoder_counter_raw(Joint);
		*angular_dp = enc->Counts * enc->radians_per_quadrature_pulse;
		osSemaphoreRelease(xEncoderInterfacesSemprHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t Encoder_getResolution(eSjcJoints_t Joint, uint16_t *resolution)
{
	if (resolution == NULL)
	{
		return SJC_RET_NOK;
	}

	xSjcEncoder_t *enc = get_encoder_handle(Joint);
	if (enc == NULL)
	{
		*resolution = 0;
		return SJC_RET_NOK;
	}

	configASSERT(xEncoderInterfacesSemprHandle != NULL);
	if (osOK == osSemaphoreAcquire(xEncoderInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		*resolution = enc->Resolution_PPR;
		osSemaphoreRelease(xEncoderInterfacesSemprHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t Encoder_setResolution(eSjcJoints_t Joint, const uint16_t resolution)
{
	if (resolution == 0)
	{
		return SJC_RET_NOK;
	}

	xSjcEncoder_t *enc = get_encoder_handle(Joint);
	if (enc == NULL)
	{
		return SJC_RET_NOK;
	}

	configASSERT(xEncoderInterfacesSemprHandle != NULL);
	if (osOK == osSemaphoreAcquire(xEncoderInterfacesSemprHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		enc->Resolution_PPR = resolution;
		enc->radians_per_quadrature_pulse = (2.0f * M_PI) / (1.0f * enc->Resolution_PPR);
		osSemaphoreRelease(xEncoderInterfacesSemprHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
static inline int32_t read_encoder_counter_raw(eSjcJoints_t Joint)
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

static inline xSjcEncoder_t *get_encoder_handle(eSjcJoints_t Joint)
{
	if (!IS_VALID_JOINT(Joint))
	{
		return NULL;
	}
	return &Encoder[Joint];
}
// ------------------------------------------------------------------- //
