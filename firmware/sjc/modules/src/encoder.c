/*
 * encoder.c
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  Quadrature encoder interface with mutex-protected position access.
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
static osMutexId_t xEncoderInterfacesMutexHandle = NULL;
static StaticSemaphore_t EncoderInterfacesMutexControlBlock;
static const osMutexAttr_t EncoderInterfacesMutex_attributes = {
	.name = "EncoderInterfacesMutex",
	.cb_mem = &EncoderInterfacesMutexControlBlock,
	.cb_size = sizeof(EncoderInterfacesMutexControlBlock),
};
// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static xSjcEncoder_t Encoder[SJC_JOINTS_SIZE];
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //

// Verification
static inline xSjcEncoder_t *get_encoder_handle(eSjcJoints_t Joint);

// Action (assume valid enc)
static inline int32_t read_counter(xSjcEncoder_t *enc);
static inline float read_angular_position(xSjcEncoder_t *enc);

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void Encoder_init(void)
{
	memset(Encoder, 0, sizeof(Encoder));

	xEncoderInterfacesMutexHandle = osMutexNew(&EncoderInterfacesMutex_attributes);

	if (xEncoderInterfacesMutexHandle == NULL)
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Encoder mutex not created, system may become unstable.\n");
	}

	Encoder[SJC_ACTUATOR_JOINT].Instance = SJC_ACTUATOR_JOINT_TIMER_INSTANCE;
	Encoder[SJC_ACTUATOR_JOINT].Is32bit = true;
	Encoder[SJC_AUX_JOINT_TH_X].Instance = SJC_AUX_JOINT_B_TIMER_INSTANCE;
	Encoder[SJC_AUX_JOINT_TH_X].Is32bit = false;
	Encoder[SJC_AUX_JOINT_TH_Y].Instance = SJC_AUX_JOINT_A_TIMER_INSTANCE;
	Encoder[SJC_AUX_JOINT_TH_Y].Is32bit = false;

	for (uint8_t i = 0; i < SJC_JOINTS_SIZE; i++)
	{
		Encoder[i].Counts = 0;
		Encoder[i].Resolution_CPR = SJC_MIN_ENCODER_RESOLUTION_CPR;
		Encoder[i].radians_per_quadrature_pulse = 1.0f;

		Encoder_reset(i);
	}
}

void Encoder_reset(eSjcJoints_t Joint)
{
	xSjcEncoder_t *enc = get_encoder_handle(Joint);
	SJC_RETURN_IF_NULL_LOG_VOID(enc, "Encoder_reset: Joint");

	configASSERT(xEncoderInterfacesMutexHandle != NULL);
	if (osOK == osMutexAcquire(xEncoderInterfacesMutexHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		LL_TIM_SetCounter(enc->Instance, 0);
		enc->Counts = 0;
		osMutexRelease(xEncoderInterfacesMutexHandle);
	}
}

void Encoder_resetMultiple(const eSjcJoints_t joints[], uint8_t count)
{
	SJC_RETURN_IF_NULL_LOG_VOID(joints, "Encoder_resetMultiple: joints");
	if (count == 0)
	{
		return;
	}

	configASSERT(xEncoderInterfacesMutexHandle != NULL);
	if (osOK == osMutexAcquire(xEncoderInterfacesMutexHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		for (uint8_t i = 0; i < count; i++)
		{
			xSjcEncoder_t *enc = get_encoder_handle(joints[i]);
			if (enc != NULL)
			{
				LL_TIM_SetCounter(enc->Instance, 0);
				enc->Counts = 0;
			}
		}
		osMutexRelease(xEncoderInterfacesMutexHandle);
	}
}

void Encoder_resetAll(void)
{
	configASSERT(xEncoderInterfacesMutexHandle != NULL);
	if (osOK == osMutexAcquire(xEncoderInterfacesMutexHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		for (uint8_t i = 0; i < SJC_JOINTS_SIZE; i++)
		{
			LL_TIM_SetCounter(Encoder[i].Instance, 0);
			Encoder[i].Counts = 0;
		}
		osMutexRelease(xEncoderInterfacesMutexHandle);
	}
}

void Encoder_enable(eSjcJoints_t Joint)
{
	xSjcEncoder_t *enc = get_encoder_handle(Joint);
	SJC_RETURN_IF_NULL_LOG_VOID(enc, "Encoder_enable: Joint");

	configASSERT(xEncoderInterfacesMutexHandle != NULL);
	if (osOK == osMutexAcquire(xEncoderInterfacesMutexHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		LL_TIM_EnableCounter(enc->Instance);
		osMutexRelease(xEncoderInterfacesMutexHandle);
	}
}

void Encoder_disable(eSjcJoints_t Joint)
{
	xSjcEncoder_t *enc = get_encoder_handle(Joint);
	SJC_RETURN_IF_NULL_LOG_VOID(enc, "Encoder_disable: Joint");

	configASSERT(xEncoderInterfacesMutexHandle != NULL);
	if (osOK == osMutexAcquire(xEncoderInterfacesMutexHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		LL_TIM_DisableCounter(enc->Instance);
		osMutexRelease(xEncoderInterfacesMutexHandle);
	}
}

eSjcReturnType_t Encoder_getCounter(eSjcJoints_t Joint, int32_t *counts)
{
	SJC_RETURN_IF_NULL_LOG_RET(counts, "Encoder_getCounter: counts");

	*counts = 0;

	xSjcEncoder_t *enc = get_encoder_handle(Joint);
	SJC_RETURN_IF_NULL_LOG_RET(enc, "Encoder_getCounter: Joint");

	configASSERT(xEncoderInterfacesMutexHandle != NULL);
	if (osOK == osMutexAcquire(xEncoderInterfacesMutexHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		*counts = read_counter(enc);
		osMutexRelease(xEncoderInterfacesMutexHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t Encoder_getAngularPosition(eSjcJoints_t Joint, float *position)
{
	SJC_RETURN_IF_NULL_LOG_RET(position, "Encoder_getAngularPosition: position");

	*position = 0.0f;

	xSjcEncoder_t *enc = get_encoder_handle(Joint);
	SJC_RETURN_IF_NULL_LOG_RET(enc, "Encoder_getAngularPosition: Joint");

	configASSERT(xEncoderInterfacesMutexHandle != NULL);
	if (osOK == osMutexAcquire(xEncoderInterfacesMutexHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		*position = read_angular_position(enc);
		osMutexRelease(xEncoderInterfacesMutexHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t Encoder_getAllAngularPositions(float positions[SJC_JOINTS_SIZE])
{
	SJC_RETURN_IF_NULL_LOG_RET(positions, "Encoder_getAllAngularPositions: positions");

	configASSERT(xEncoderInterfacesMutexHandle != NULL);
	if (osOK == osMutexAcquire(xEncoderInterfacesMutexHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		for (uint8_t i = 0; i < SJC_JOINTS_SIZE; i++)
		{
			positions[i] = read_angular_position(&Encoder[i]);
		}
		osMutexRelease(xEncoderInterfacesMutexHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t Encoder_getAllResolutions(uint16_t resolutions[SJC_JOINTS_SIZE])
{
	SJC_RETURN_IF_NULL_LOG_RET(resolutions, "Encoder_getAllResolutions: resolutions");

	configASSERT(xEncoderInterfacesMutexHandle != NULL);
	if (osOK == osMutexAcquire(xEncoderInterfacesMutexHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		for (uint8_t i = 0; i < SJC_JOINTS_SIZE; i++)
		{
			resolutions[i] = Encoder[i].Resolution_CPR;
		}
		osMutexRelease(xEncoderInterfacesMutexHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t Encoder_getResolution(eSjcJoints_t Joint, uint16_t *resolution)
{
	SJC_RETURN_IF_NULL_LOG_RET(resolution, "Encoder_getResolution: resolution");

	*resolution = 0;

	xSjcEncoder_t *enc = get_encoder_handle(Joint);
	SJC_RETURN_IF_NULL_LOG_RET(enc, "Encoder_getResolution: Joint");

	configASSERT(xEncoderInterfacesMutexHandle != NULL);
	if (osOK == osMutexAcquire(xEncoderInterfacesMutexHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		*resolution = enc->Resolution_CPR;
		osMutexRelease(xEncoderInterfacesMutexHandle);
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
	SJC_RETURN_IF_NULL_LOG_RET(enc, "Encoder_setResolution: Joint");

	configASSERT(xEncoderInterfacesMutexHandle != NULL);
	if (osOK == osMutexAcquire(xEncoderInterfacesMutexHandle, SJC_HARDWARE_ACCESS_WAIT_TIME_MS))
	{
		enc->Resolution_CPR = resolution;
		enc->radians_per_quadrature_pulse = (2.0f * M_PI) / (1.0f * enc->Resolution_CPR);
		osMutexRelease(xEncoderInterfacesMutexHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// Verification
static inline xSjcEncoder_t *get_encoder_handle(eSjcJoints_t Joint)
{
	if (!IS_VALID_JOINT(Joint))
	{
		return NULL;
	}
	return &Encoder[Joint];
}

// Action
// Note: 16-bit counter overflow is not handled because aux joints (THx, THy)
// operate within +-90 degrees. At 8192 CPR (2048 PPR x4 quadrature),
// 90 deg = 2048 counts, well within int16_t range (+-32767).
static inline int32_t read_counter(xSjcEncoder_t *enc)
{
	enc->Counts = enc->Is32bit ? (int32_t)LL_TIM_GetCounter(enc->Instance)
							   : (int16_t)LL_TIM_GetCounter(enc->Instance);

	return enc->Counts;
}

static inline float read_angular_position(xSjcEncoder_t *enc)
{
	read_counter(enc);
	return enc->Counts * enc->radians_per_quadrature_pulse;
}
// ------------------------------------------------------------------- //
