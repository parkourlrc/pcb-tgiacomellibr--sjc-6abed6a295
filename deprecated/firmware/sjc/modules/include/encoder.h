/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__ENCODER_H_
#define SJC__MODULES__ENCODER_H_

#include "system_types/include/types.h"

void Encoder_init(void);
void Encoder_reset(eSjcJoints_t Joint);
void Encoder_enable(eSjcJoints_t Joint);
void Encoder_disable(eSjcJoints_t Joint);

eSjcReturnType_t Encoder_getCounter(eSjcJoints_t Joint, int32_t *counts);
eSjcReturnType_t Encoder_getAngularDisplacement(eSjcJoints_t Joint, float *angular_dp);

eSjcReturnType_t Encoder_getResolution(eSjcJoints_t Joint, uint16_t *resolution);
eSjcReturnType_t Encoder_setResolution(eSjcJoints_t Joint, const uint16_t resolution);

#endif /* SJC__MODULES__ENCODER_H_ */
