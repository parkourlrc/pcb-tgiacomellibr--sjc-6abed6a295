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
#include "helpers.hpp"
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
float real_value_clamp(const float var, const float limit_min, const float limit_max)
{
	float ret;

	if (var > limit_max)
	{
		ret = limit_max;
	}
	else if (var < limit_min)
	{
		ret = limit_min;
	}
	else
	{
		ret = var;
	}

	return ret;
}
int32_t int_value_clamp(const int32_t var, const int32_t limit_min, const int32_t limit_max)
{
	int32_t ret;

	if (var > limit_max)
	{
		ret = limit_max;
	}
	else if (var < limit_min)
	{
		ret = limit_min;
	}
	else
	{
		ret = var;
	}

	return ret;
}
uint32_t unsigned_int_value_clamp(const uint32_t var, const uint32_t limit_min, const uint32_t limit_max)
{
	uint32_t ret;

	if (var > limit_max)
	{
		ret = limit_max;
	}
	else if (var < limit_min)
	{
		ret = limit_min;
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