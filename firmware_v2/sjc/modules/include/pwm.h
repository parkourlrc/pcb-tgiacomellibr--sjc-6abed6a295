/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__PWM_H_
#define SJC__MODULES__PWM_H_

#include "types.h"

void SJC_PWM_Init(const eSjcPwmBaseFreq_t BaseFrequency);
void SJC_PWM_Enable(void);
void SJC_PWM_Disable(void);

float SJC_PWM_GetMinDutyCycle(void);
float SJC_PWM_GetMaxDutyCycle(void);
float SJC_PWM_GetDutyCycle(void);
eSjcReturnType_t SJC_PWM_SetDutyCycle(const float duty);

uint32_t SJC_PWM_GetFrequency(void);
uint32_t SJC_PWM_SetFrequency(const eSjcPwmBaseFreq_t BaseFrequency);

#endif /* MOTOR_H_ */
