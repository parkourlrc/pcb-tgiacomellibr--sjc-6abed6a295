/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__PID_CONTROLLER_H_
#define SJC__MODULES__PID_CONTROLLER_H_

#include "system_types/include/types.h"

void SJC_PidController_Init();

float SJC_PidController_Compute(const float ControllerError, const float dt);
float SJC_PidController_GetError(void);
float SJC_PidController_GetReference(void);
void SJC_PidController_SetReference(const float reference);

xSjcRange_t SJC_PidController_GetActuatorEffortLimits(void);
void SJC_PidController_SetActuatorEffortLimits(const xSjcRange_t limits);

xSjcServoPidParameters_t SJC_PidController_GetControlParameters(void);
void SJC_PidController_SetControlParameters(const xSjcServoPidParameters_t Parameters);

#endif /* MOTOR_H_ */
