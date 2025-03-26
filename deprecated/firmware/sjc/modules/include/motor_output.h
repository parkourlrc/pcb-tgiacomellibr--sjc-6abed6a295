/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__DC_MOTOR_OUTPUT_H_
#define SJC__MODULES__DC_MOTOR_OUTPUT_H_

#include "system_types/include/types.h"

void MotorOutput_init(void);
void MotorOutput_enable(void);
void MotorOutput_disable(void);
eSjcReturnType_t MotorOutput_getState(bool *state);

#endif /* SJC__MODULES__DC_MOTOR_OUTPUT_H_ */
