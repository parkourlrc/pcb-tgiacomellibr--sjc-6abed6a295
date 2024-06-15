/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__PID_CONTROLLER_H_
#define SJC__MODULES__PID_CONTROLLER_H_

#include "types.h"

void SJC_PidController_Init();

float PidController_compute(const float ControllerError, const float dt);
float PidController_getError(void);
float PidController_getReference(void);
void PidController_setReference(const float reference);

xSjcRange_t PidController_getActuatorEffortLimits(void);
void PidController_setActuatorEffortLimits(const xSjcRange_t limits);

xSjcServoPidParameters_t PidController_getControlParameters(void);
void PidController_setControlParameters(const xSjcServoPidParameters_t Parameters);

#endif /* MOTOR_H_ */
