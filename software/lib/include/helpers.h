/*
 * data_structures.h
 *
 *  Created on: Sep 17, 2023
 *      Author: tiago
 */

#ifndef SJC__SYSTEM_TYPES__HELPERS_H_
#define SJC__SYSTEM_TYPES__HELPERS_H_

#include "stdint.h"

// Clamp functions
float real_value_clamp(const float var, const float LimitMin, const float LimitMax);
int32_t int_value_clamp(const int32_t var, const int32_t LimitMin, const int32_t LimitMax);
uint32_t unsigned_int_value_clamp(const uint32_t var, const uint32_t LimitMin, const uint32_t LimitMax);

void compose_byte_array_from_ui32(const uint32_t src, uint16_t *dst);
void compose_ui32_from_byte_array(const uint16_t *src, uint32_t *dst);

void compose_byte_array_from_float(const float src, uint16_t *dst);
void compose_float_from_byte_array(const uint16_t *src, float *dst);

#endif /* SJC__SYSTEM_TYPES__HELPERS_H_ */