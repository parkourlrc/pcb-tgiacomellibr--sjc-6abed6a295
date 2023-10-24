/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__PWM_H_
#define SJC__MODULES__PWM_H_

#include "types.h"

void SJC_PWM_Init(xSjcActuatorPwm_t * Pwm, const eSjcPwmBaseFreq_t BaseFrequency);
void SJC_PWM_Enable(xSjcActuatorPwm_t * Pwm);
void SJC_PWM_Disable(xSjcActuatorPwm_t * Pwm);

float SJC_PWM_GetMinDutyCycle(xSjcActuatorPwm_t * Pwm);
float SJC_PWM_GetMaxDutyCycle(xSjcActuatorPwm_t * Pwm);
float SJC_PWM_GetDutyCycle(xSjcActuatorPwm_t * Pwm);
eSjcReturnType_t SJC_PWM_SetDutyCycle(xSjcActuatorPwm_t * Pwm, const float duty);

uint32_t SJC_PWM_GetFrequency(xSjcActuatorPwm_t * Pwm);
uint32_t SJC_PWM_SetFrequency(xSjcActuatorPwm_t * Pwm, const eSjcPwmBaseFreq_t BaseFrequency);


#endif /* MOTOR_H_ */
