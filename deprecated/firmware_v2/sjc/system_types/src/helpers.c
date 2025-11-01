/*
 * helpers.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 *
 *  Description:
 *    Implementations for common math helpers and Modbus-oriented
 *    word/byte composition utilities. All comments in English.
 */

// HAL includes ------------------------------------------------------ //
// (none needed here)                                                 //
// ------------------------------------------------------------------- //

// Standard library includes ----------------------------------------- //
#include <string.h> /* memcpy */
// ------------------------------------------------------------------- //

// Project includes -------------------------------------------------- //
#include "modules/include/debug_output.h"
#include "system_types/include/helpers.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// (no internal defines)                                              //
// ------------------------------------------------------------------- //

// Module data structures ------------------------------------------- //
// (no internal types)                                               //
// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// (no internal state)                                               //
// ------------------------------------------------------------------- //

// Private function prototypes -------------------------------------- //
// (none)                                                            //
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //

/**
 * @brief Clamp a float value between min and max.
 *        If bounds are inverted, they are swapped defensively.
 *        If value is non-finite, returns 0.0f when the interval spans zero,
 *        otherwise returns the lower bound.
 */
float SJC_Helpers_RealClamp(const float value, const float min, const float max)
{
    float lo = min, hi = max;
    if (lo > hi)
    {
        float t = lo;
        lo = hi;
        hi = t;
    }

    if (!isfinite(value))
    {
        return (lo < 0.0f && hi > 0.0f) ? 0.0f : lo;
    }

    return fminf(fmaxf(value, lo), hi);
}

/**
 * @brief Clamp a signed 32-bit integer between min and max.
 *        Swaps bounds defensively if inverted.
 */
int32_t SJC_Helpers_IntClamp(const int32_t value, const int32_t min, const int32_t max)
{
    int32_t lo = min, hi = max;
    if (lo > hi)
    {
        int32_t t = lo;
        lo = hi;
        hi = t;
    }
    return SJC_INT_MIN(SJC_INT_MAX(value, lo), hi);
}

/**
 * @brief Clamp an unsigned 32-bit integer between min and max.
 *        Swaps bounds defensively if inverted.
 */
uint32_t SJC_Helpers_UintClamp(const uint32_t value, const uint32_t min, const uint32_t max)
{
    uint32_t lo = min, hi = max;
    if (lo > hi)
    {
        uint32_t t = lo;
        lo = hi;
        hi = t;
    }
    return SJC_UINT_MIN(SJC_UINT_MAX(value, lo), hi);
}

/* ------------------------------ */
/* Byte/word composition utils    */
/* Modbus register order:         */
/*   - high  16-bit word first    */
/*   - low   16-bit word second   */
/* ------------------------------ */

/**
 * @brief Split a uint32_t into two 16-bit words (high first).
 */
void compose_ui16_array_from_ui32(const uint32_t src, uint16_t *dst)
{
    SJC_RETURN_IF_NULL_LOG_VOID(dst, "compose_ui16_array_from_ui32: dst is NULL");

    uint16_t w[2];
    memcpy(w, &src, sizeof(src)); /* read native layout */
    dst[0] = w[1];                /* high word first */
    dst[1] = w[0];                /* low  word second */
}

/**
 * @brief Join two 16-bit words into a uint32_t (expects high first).
 */
void compose_ui32_from_ui16_array(const uint16_t *src, uint32_t *dst)
{
    SJC_RETURN_IF_NULL_LOG_VOID(src, "compose_ui32_from_ui16_array: src is NULL");
    SJC_RETURN_IF_NULL_LOG_VOID(dst, "compose_ui32_from_ui16_array: dst is NULL");

    uint16_t w[2] = {src[1], src[0]}; /* reorder to native: low, high */
    memcpy(dst, w, sizeof(*dst));
}

/**
 * @brief Split a float into two 16-bit words (high first).
 *        Assumes IEEE-754 binary32 representation.
 */
void compose_ui16_array_from_float(const float src, uint16_t *dst)
{
    SJC_RETURN_IF_NULL_LOG_VOID(dst, "compose_ui16_array_from_float: dst is NULL");

    uint16_t w[2];
    memcpy(w, &src, sizeof(src));
    dst[0] = w[1]; /* high word first */
    dst[1] = w[0]; /* low  word second */
}

/**
 * @brief Join two 16-bit words into a float (expects high first).
 *        Assumes IEEE-754 binary32 representation.
 */
void compose_float_from_ui16_array(const uint16_t *src, float *dst)
{
    SJC_RETURN_IF_NULL_LOG_VOID(src, "compose_float_from_ui16_array: src is NULL");
    SJC_RETURN_IF_NULL_LOG_VOID(dst, "compose_float_from_ui16_array: dst is NULL");

    uint16_t w[2] = {src[1], src[0]}; /* reorder to native: low, high */
    memcpy(dst, w, sizeof(*dst));
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// (none)                                                              //
// ------------------------------------------------------------------- //
