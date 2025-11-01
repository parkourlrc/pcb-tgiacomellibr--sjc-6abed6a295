/*
 * helpers.c
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  Implementation of utility functions for clamping values and
 *  converting between standard C types and Modbus register arrays.
 */

#include "helpers.hpp"

/* ================================================================
 * Clamp functions
 * ================================================================ */
float float_value_clamp(const float var, const float limit_min, const float limit_max)
{
    if (var > limit_max)
        return limit_max;
    else if (var < limit_min)
        return limit_min;
    return var;
}

int32_t int32_value_clamp(const int32_t var, const int32_t limit_min, const int32_t limit_max)
{
    if (var > limit_max)
        return limit_max;
    else if (var < limit_min)
        return limit_min;
    return var;
}

uint32_t unsigned_int32_value_clamp(const uint32_t var, const uint32_t limit_min, const uint32_t limit_max)
{
    if (var > limit_max)
        return limit_max;
    else if (var < limit_min)
        return limit_min;
    return var;
}

/* ================================================================
 * Conversion helpers
 * ================================================================ */
void compose_ui16_array_from_ui32(const uint32_t src, uint16_t *dst)
{
    xSjcUInt32Conv_t conv;
    conv.u32 = src;
    // Store high word first (big-endian Modbus order)
    dst[0] = conv.u16[1];
    dst[1] = conv.u16[0];
}

void compose_ui32_from_ui16_array(const uint16_t *src, uint32_t *dst)
{
    xSjcUInt32Conv_t conv;
    conv.u16[1] = src[0]; // high word
    conv.u16[0] = src[1]; // low word
    *dst = conv.u32;
}

void compose_ui16_array_from_float(const float src, uint16_t *dst)
{
    xSjcFloatConv_t conv;
    conv.f = src;
    // Store high word first (big-endian Modbus order)
    dst[0] = conv.u16[1];
    dst[1] = conv.u16[0];
}

void compose_float_from_ui16_array(const uint16_t *src, float *dst)
{
    xSjcFloatConv_t conv;
    conv.u16[1] = src[0]; // high word
    conv.u16[0] = src[1]; // low word
    *dst = conv.f;
}
