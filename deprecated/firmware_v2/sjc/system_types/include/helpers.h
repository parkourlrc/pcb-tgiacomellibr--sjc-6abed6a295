/*
 * data_structures.h
 *
 *  Created on: Sep 17, 2023
 *      Author: tiago
 */

#ifndef SJC__SYSTEM_TYPES__HELPERS_H_
#define SJC__SYSTEM_TYPES__HELPERS_H_

#include "math.h"
#include "stdint.h"

// Defines
#define RAD_TO_DEG (180 / M_PI)
#define DEG_TO_RAD (M_PI / 180)
#define DEF_RAD_PER_SEC_TO_RPM (30.0f / M_PI)
#define DEF_RPM_TO_RAD_PER_SEC (M_PI / 30.0f)

#define CONV_MILLI_TO_BASE_FLOAT(X) (X / 1000.0f)

// Clamp functions
float SJC_Helpers_RealClamp(const float var, const float LimitMin, const float LimitMax);
int32_t SJC_Helpers_IntClamp(const int32_t var, const int32_t LimitMin, const int32_t LimitMax);
uint32_t SJC_Helpers_UintClamp(const uint32_t var, const uint32_t LimitMin, const uint32_t LimitMax);

void compose_byte_array_from_ui32(const uint32_t src, uint16_t *dst);
void compose_ui32_from_byte_array(const uint16_t *src, uint32_t *dst);

void compose_byte_array_from_float(const float src, uint16_t *dst);
void compose_float_from_byte_array(const uint16_t *src, float *dst);

#endif /* SJC__SYSTEM_TYPES__HELPERS_H_ */
