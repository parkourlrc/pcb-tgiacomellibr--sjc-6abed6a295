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

float SJC_PidController_Compute(const float ControllerError, const float dt);
float SJC_PidController_GetError(void);
float SJC_PidController_GetReference(void);
void SJC_PidController_SetReference(const float reference);

xPidControllerParameters_t SJC_PidController_GetControlParameters(void);
void SJC_PidController_SetControlParameters(const xPidControllerParameters_t Parameters);

#endif /* MOTOR_H_ */
