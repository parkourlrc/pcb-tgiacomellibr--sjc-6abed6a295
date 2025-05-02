/*
 * helpers.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "system_types/include/helpers.h"
#include "modules/include/debug_output.h"
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

// --- Clamp implementations ---
float SJC_Helpers_RealClamp(const float value, const float min, const float max)
{
	if (!isfinite(value))
	{
		return ((min < 0.0f && max > 0.0f) ? 0.0f : min);
	}
	return fminf(fmaxf(value, min), max);
}

int32_t SJC_Helpers_IntClamp(const int32_t value, const int32_t min, const int32_t max)
{
	return INT_MIN(INT_MAX(value, min), max);
}

uint32_t SJC_Helpers_UintClamp(const uint32_t value, const uint32_t min, const uint32_t max)
{
	return UINT_MIN(UINT_MAX(value, min), max);
}

// --- Byte composition utilities ---
void compose_byte_array_from_ui32(const uint32_t src, uint16_t *dst)
{
	SJC_RETURN_IF_NULL_LOG_VOID(dst, "compose_byte_array_from_ui32: dst is NULL");

	uint16_t *ptr = (uint16_t *)(&src);
	dst[0] = ptr[1];
	dst[1] = ptr[0];
}

void compose_ui32_from_byte_array(const uint16_t *src, uint32_t *dst)
{
	SJC_RETURN_IF_NULL_LOG_VOID(src, "compose_ui32_from_byte_array: src is NULL");
	SJC_RETURN_IF_NULL_LOG_VOID(dst, "compose_ui32_from_byte_array: dst is NULL");

	uint16_t *ptr = (uint16_t *)(dst);
	ptr[1] = src[0];
	ptr[0] = src[1];
}

void compose_byte_array_from_float(const float src, uint16_t *dst)
{
	SJC_RETURN_IF_NULL_LOG_VOID(dst, "compose_byte_array_from_float: dst is NULL");

	uint16_t *ptr = (uint16_t *)(&src);
	dst[0] = ptr[1];
	dst[1] = ptr[0];
}

void compose_float_from_byte_array(const uint16_t *src, float *dst)
{
	SJC_RETURN_IF_NULL_LOG_VOID(src, "compose_float_from_byte_array: src is NULL");
	SJC_RETURN_IF_NULL_LOG_VOID(dst, "compose_float_from_byte_array: dst is NULL");

	uint16_t *ptr = (uint16_t *)(dst);
	ptr[1] = src[0];
	ptr[0] = src[1];
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
