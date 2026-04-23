/*
 * velocity_measurement.c
 *
 *  Created on: Mar 28, 2026
 *      Author: Tiago Giacomelli
 *
 *  Filtered joint velocity computation from encoder position updates.
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "modules/include/velocity_measurement.h"
#include "modules/include/debug_output.h"
#include "system_types/include/filters.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //
#define IS_VALID_JOINT(joint) ((joint) < SJC_JOINTS_SIZE)
#define VELOCITY_MUTEX_WAIT_TIME_MS ((uint32_t)2UL)
// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
static osMutexId_t xVelocityMutexHandle = NULL;
static StaticSemaphore_t xVelocityMutexControlBlock;
static const osMutexAttr_t xVelocityMutex_attributes = {
	.name = "VelocityMeasurementMutex",
	.cb_mem = &xVelocityMutexControlBlock,
	.cb_size = sizeof(xVelocityMutexControlBlock),
};
// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static float xFilteredVelocity[SJC_JOINTS_SIZE];
static float xPreviousPosition[SJC_JOINTS_SIZE];
static ExponentialFilter_t xVelocityFilter[SJC_JOINTS_SIZE];
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //
static inline float compute_velocity(eSjcJoints_t joint, float position, float dt);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void VelocityMeasurement_init(float fc, float fs)
{
	xVelocityMutexHandle = osMutexNew(&xVelocityMutex_attributes);

	if (xVelocityMutexHandle == NULL)
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Velocity measurement mutex not created.\n");
	}

	for (uint8_t i = 0; i < SJC_JOINTS_SIZE; i++)
	{
		ExponentialFilter_init(&xVelocityFilter[i], fc, fs);
		xFilteredVelocity[i] = 0.0f;
		xPreviousPosition[i] = 0.0f;
	}
}

eSjcReturnType_t VelocityMeasurement_setInitialPosition(const eSjcJoints_t joint, const float position)
{
	if (!IS_VALID_JOINT(joint))
	{
		return SJC_RET_NOK;
	}

	configASSERT(xVelocityMutexHandle != NULL);
	if (osOK == osMutexAcquire(xVelocityMutexHandle, VELOCITY_MUTEX_WAIT_TIME_MS))
	{
		xPreviousPosition[joint] = position;
		xFilteredVelocity[joint] = 0.0f;
		osMutexRelease(xVelocityMutexHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t VelocityMeasurement_setAllInitialPositions(const float positions[SJC_JOINTS_SIZE])
{
	SJC_RETURN_IF_NULL_LOG_RET(positions, "VelocityMeasurement_setAllInitialPos: positions");

	configASSERT(xVelocityMutexHandle != NULL);
	if (osOK == osMutexAcquire(xVelocityMutexHandle, VELOCITY_MUTEX_WAIT_TIME_MS))
	{
		for (uint8_t i = 0; i < SJC_JOINTS_SIZE; i++)
		{
			xPreviousPosition[i] = positions[i];
			xFilteredVelocity[i] = 0.0f;
		}
		osMutexRelease(xVelocityMutexHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t VelocityMeasurement_reset(const eSjcJoints_t joint)
{
	if (!IS_VALID_JOINT(joint))
	{
		return SJC_RET_NOK;
	}

	configASSERT(xVelocityMutexHandle != NULL);
	if (osOK == osMutexAcquire(xVelocityMutexHandle, VELOCITY_MUTEX_WAIT_TIME_MS))
	{
		ExponentialFilter_reset(&xVelocityFilter[joint]);
		xFilteredVelocity[joint] = 0.0f;
		xPreviousPosition[joint] = 0.0f;
		osMutexRelease(xVelocityMutexHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t VelocityMeasurement_resetMultiple(const eSjcJoints_t joints[], uint8_t count)
{
	SJC_RETURN_IF_NULL_LOG_RET(joints, "VelocityMeasurement_resetMultiple: joints");
	if (count == 0)
	{
		return SJC_RET_NOK;
	}

	configASSERT(xVelocityMutexHandle != NULL);
	if (osOK == osMutexAcquire(xVelocityMutexHandle, VELOCITY_MUTEX_WAIT_TIME_MS))
	{
		for (uint8_t i = 0; i < count; i++)
		{
			if (IS_VALID_JOINT(joints[i]))
			{
				ExponentialFilter_reset(&xVelocityFilter[joints[i]]);
				xFilteredVelocity[joints[i]] = 0.0f;
				xPreviousPosition[joints[i]] = 0.0f;
			}
		}
		osMutexRelease(xVelocityMutexHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t VelocityMeasurement_resetAll(void)
{
	configASSERT(xVelocityMutexHandle != NULL);
	if (osOK == osMutexAcquire(xVelocityMutexHandle, VELOCITY_MUTEX_WAIT_TIME_MS))
	{
		for (uint8_t i = 0; i < SJC_JOINTS_SIZE; i++)
		{
			ExponentialFilter_reset(&xVelocityFilter[i]);
			xFilteredVelocity[i] = 0.0f;
			xPreviousPosition[i] = 0.0f;
		}
		osMutexRelease(xVelocityMutexHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t VelocityMeasurement_update(const eSjcJoints_t joint, const float position, const float dt, float *velocity)
{
	if (!IS_VALID_JOINT(joint) || dt <= 0.0f)
	{
		return SJC_RET_NOK;
	}

	configASSERT(xVelocityMutexHandle != NULL);
	if (osOK == osMutexAcquire(xVelocityMutexHandle, VELOCITY_MUTEX_WAIT_TIME_MS))
	{
		float vel = compute_velocity(joint, position, dt);

		if (velocity != NULL)
		{
			*velocity = vel;
		}

		osMutexRelease(xVelocityMutexHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t VelocityMeasurement_updateAll(const float positions[SJC_JOINTS_SIZE], const float dt, float *velocities)
{
	SJC_RETURN_IF_NULL_LOG_RET(positions, "VelocityMeasurement_updateAll: positions");

	if (dt <= 0.0f)
	{
		return SJC_RET_NOK;
	}

	configASSERT(xVelocityMutexHandle != NULL);
	if (osOK == osMutexAcquire(xVelocityMutexHandle, VELOCITY_MUTEX_WAIT_TIME_MS))
	{
		for (uint8_t i = 0; i < SJC_JOINTS_SIZE; i++)
		{
			float vel = compute_velocity(i, positions[i], dt);

			if (velocities != NULL)
			{
				velocities[i] = vel;
			}
		}
		osMutexRelease(xVelocityMutexHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t VelocityMeasurement_get(const eSjcJoints_t joint, float *velocity)
{
	SJC_RETURN_IF_NULL_LOG_RET(velocity, "VelocityMeasurement_get: velocity");

	*velocity = 0.0f;

	if (!IS_VALID_JOINT(joint))
	{
		return SJC_RET_NOK;
	}

	configASSERT(xVelocityMutexHandle != NULL);
	if (osOK == osMutexAcquire(xVelocityMutexHandle, VELOCITY_MUTEX_WAIT_TIME_MS))
	{
		*velocity = xFilteredVelocity[joint];
		osMutexRelease(xVelocityMutexHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t VelocityMeasurement_getAll(float velocities[SJC_JOINTS_SIZE])
{
	SJC_RETURN_IF_NULL_LOG_RET(velocities, "VelocityMeasurement_getAll: velocities");

	configASSERT(xVelocityMutexHandle != NULL);
	if (osOK == osMutexAcquire(xVelocityMutexHandle, VELOCITY_MUTEX_WAIT_TIME_MS))
	{
		for (uint8_t i = 0; i < SJC_JOINTS_SIZE; i++)
		{
			velocities[i] = xFilteredVelocity[i];
		}
		osMutexRelease(xVelocityMutexHandle);
		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
/**
 * @brief Compute filtered velocity from position delta.
 * Assumes valid joint index and dt > 0. Called within mutex.
 */
static inline float compute_velocity(eSjcJoints_t joint, float position, float dt)
{
	float displacement = position - xPreviousPosition[joint];
	float raw_velocity = displacement / dt;

	xPreviousPosition[joint] = position;
	xFilteredVelocity[joint] = ExponentialFilter_update(&xVelocityFilter[joint], raw_velocity);

	return xFilteredVelocity[joint];
}
// ------------------------------------------------------------------- //
