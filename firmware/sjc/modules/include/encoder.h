/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__ENCODER_H_
#define SJC__MODULES__ENCODER_H_

#include "types.h"

eSjcReturnType_t SJC_Encoder_init(xSjcEncoder_t * Encoder, eSjcJoints_t Joint);
eSjcReturnType_t SJC_Encoder_Reset(xSjcEncoder_t * Encoder);
eSjcReturnType_t SJC_Encoder_Enable(xSjcEncoder_t * Encoder);
eSjcReturnType_t SJC_Encoder_Disable(xSjcEncoder_t * Encoder);
eSjcReturnType_t SJC_Encoder_GetCount(xSjcEncoder_t * Encoder, int32_t * Counter);
eSjcReturnType_t SJC_Encoder_GetAngularDisplacement(xSjcEncoder_t * Encoder, float * Displacement);

eSjcReturnType_t SJC_Encoder_SetResolution(xSjcEncoder_t * Encoder, const uint16_t Resolution);

#endif /* SJC__MODULES__ENCODER_H_ */
