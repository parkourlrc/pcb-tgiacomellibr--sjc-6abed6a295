/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__PID_CONTROLLER_H_
#define SJC__MODULES__PID_CONTROLLER_H_

#include "types.h"

eSjcReturnType_t SJC_PidController_Init(xSjcController_t * Controller);
float SJC_PidController_Compute(const float ControlError, const float dt);
float SJC_PidController_GetError(void);
float SJC_PidController_GetReference(void);
void SJC_PidController_SetReference(const float reference);

eSjcReturnType_t SJC_PidController_GetControlParameters(xControllerParameters_t * Parameters);
eSjcReturnType_t SJC_PidController_SetControlParameters(const xControllerParameters_t Parameters);
eSjcReturnType_t SJC_PidController_GetControlUpdateRateHz(float * config);
eSjcReturnType_t SJC_PidController_SetControlUpdateRateHz(const float config);
eSjcReturnType_t SJC_PidController_GetControlUpdatePeriodMs(uint32_t * config);

#endif /* MOTOR_H_ */
