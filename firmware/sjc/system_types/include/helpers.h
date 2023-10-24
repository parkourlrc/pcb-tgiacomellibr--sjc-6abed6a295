/*
 * data_structures.h
 *
 *  Created on: Sep 17, 2023
 *      Author: tiago
 */

#ifndef SJC__SYSTEM_TYPES__HELPERS_H_
#define SJC__SYSTEM_TYPES__HELPERS_H_

// Clamp functions
float SJC_Helpers_RealClamp(const float var, const float LimitMin, const float LimitMax);
int32_t SJC_Helpers_IntClamp(const int32_t var, const int32_t LimitMin, const int32_t LimitMax);
uint32_t SJC_Helpers_UintClamp(const uint32_t var, const uint32_t LimitMin, const uint32_t LimitMax);

#endif /* SJC__SYSTEM_TYPES__HELPERS_H_ */
