/*
 * helpers.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "helpers.h"
// ------------------------------------------------------------------- //

// Module defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
float SJC_Helpers_RealClamp(const float var, const float LimitMin, const float LimitMax)
{
	float ret;

	if (var > LimitMax)
	{
		ret = LimitMax;
	}
	else if (var < LimitMin)
	{
		ret = LimitMin;
	}
	else
	{
		ret = var;
	}

	return ret;
}
int32_t SJC_Helpers_IntClamp(const int32_t var, const int32_t LimitMin, const int32_t LimitMax)
{
	int32_t ret;

	if (var > LimitMax)
	{
		ret = LimitMax;
	}
	else if (var < LimitMin)
	{
		ret = LimitMin;
	}
	else
	{
		ret = var;
	}

	return ret;
}
uint32_t SJC_Helpers_UintClamp(const uint32_t var, const uint32_t LimitMin, const uint32_t LimitMax)
{
	uint32_t ret;

	if (var > LimitMax)
	{
		ret = LimitMax;
	}
	else if (var < LimitMin)
	{
		ret = LimitMin;
	}
	else
	{
		ret = var;
	}

	return ret;
}

void compose_byte_array_from_ui32(const uint32_t src, uint16_t *dst)
{
	uint16_t *ptr = (uint16_t *)(&src);
	dst[0] = ptr[1];
	dst[1] = ptr[0];
}

void compose_ui32_from_byte_array(const uint16_t *src, uint32_t *dst)
{
	uint16_t *ptr = (uint16_t *)(dst);

	ptr[1] = src[0];
	ptr[0] = src[1];
}

void compose_byte_array_from_float(const float src, uint16_t *dst)
{
	uint16_t *ptr = (uint16_t *)(&src);
	dst[0] = ptr[1];
	dst[1] = ptr[0];
}

void compose_float_from_byte_array(const uint16_t *src, float *dst)
{
	uint16_t *ptr = (uint16_t *)(dst);

	ptr[1] = src[0];
	ptr[0] = src[1];
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //