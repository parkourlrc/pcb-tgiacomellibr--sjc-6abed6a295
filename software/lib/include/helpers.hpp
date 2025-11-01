/*
 * helpers.hpp
 *
 *  Created on: Sep 17, 2023
 *      Author: Tiago Giacomelli
 *
 *  Utility functions for clamping values and safely converting
 *  between native types (float, uint32_t) and Modbus register arrays (uint16_t[2]).
 */

#ifndef SJC_HELPERS_HPP_
#define SJC_HELPERS_HPP_

#include <stdint.h>

/* ================================================================
 * Type definitions for safe conversions
 * ================================================================ */

/**
 * @brief Union for safe conversion between uint32_t and two uint16_t registers.
 */
typedef union
{
    uint32_t u32;    ///< 32-bit unsigned integer representation
    uint16_t u16[2]; ///< 2x 16-bit unsigned registers (little-endian split)
} xSjcUInt32Conv_t;

/**
 * @brief Union for safe conversion between float and two uint16_t registers.
 */
typedef union
{
    float f;         ///< 32-bit floating point representation
    uint16_t u16[2]; ///< 2x 16-bit unsigned registers (little-endian split)
} xSjcFloatConv_t;

/* ================================================================
 * Clamp functions
 * ================================================================ */

/**
 * @brief Clamp a float variable within [limit_min, limit_max].
 */
float float_value_clamp(const float var, const float limit_min, const float limit_max);

/**
 * @brief Clamp a signed integer variable within [limit_min, limit_max].
 */
int32_t int32_value_clamp(const int32_t var, const int32_t limit_min, const int32_t limit_max);

/**
 * @brief Clamp an unsigned integer variable within [limit_min, limit_max].
 */
uint32_t unsigned_int32_value_clamp(const uint32_t var, const uint32_t limit_min, const uint32_t limit_max);

/* ================================================================
 * Conversion helpers
 * ================================================================ */

/**
 * @brief Split a uint32_t into two uint16_t registers (big-endian order for Modbus).
 */
void compose_ui16_array_from_ui32(const uint32_t src, uint16_t *dst);

/**
 * @brief Compose a uint32_t from two uint16_t registers (big-endian order for Modbus).
 */
void compose_ui32_from_ui16_array(const uint16_t *src, uint32_t *dst);

/**
 * @brief Split a float into two uint16_t registers (big-endian order for Modbus).
 */
void compose_ui16_array_from_float(const float src, uint16_t *dst);

/**
 * @brief Compose a float from two uint16_t registers (big-endian order for Modbus).
 */
void compose_float_from_ui16_array(const uint16_t *src, float *dst);

#endif /* SJC_HELPERS_HPP_ */
