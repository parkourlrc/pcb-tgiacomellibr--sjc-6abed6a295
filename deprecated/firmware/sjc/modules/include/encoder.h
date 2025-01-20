/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__ENCODER_H_
#define SJC__MODULES__ENCODER_H_

#include "system_types/include/types.h"

void SJC_Encoder_init(void);
void SJC_Encoder_Reset(eSjcJoints_t Joint);
void SJC_Encoder_Enable(eSjcJoints_t Joint);
void SJC_Encoder_Disable(eSjcJoints_t Joint);

eSjcReturnType_t SJC_Encoder_GetCounter(eSjcJoints_t Joint, int32_t *counts);
eSjcReturnType_t SJC_Encoder_GetAngularDisplacement(eSjcJoints_t Joint, float *angular_dp);

eSjcReturnType_t SJC_Encoder_GetResolution(eSjcJoints_t Joint, uint16_t *resolution);
eSjcReturnType_t SJC_Encoder_SetResolution(eSjcJoints_t Joint, const uint16_t resolution);

#endif /* SJC__MODULES__ENCODER_H_ */
