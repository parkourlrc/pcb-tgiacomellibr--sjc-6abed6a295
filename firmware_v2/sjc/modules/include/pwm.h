/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__PWM_H_
#define SJC__MODULES__PWM_H_

#include "types.h"

void PWM_init(const eSjcPwmBaseFreq_t BaseFrequency);
void PWM_enable(void);
void PWM_disable(void);

float PWM_getMinDutyCycle(void);
float PWM_getMaxDutyCycle(void);
float PWM_getDutyCycle(void);
eSjcReturnType_t PWM_setDutyCycle(const float duty);

uint32_t PWM_getFrequency(void);
uint32_t PWM_setFrequency(const eSjcPwmBaseFreq_t BaseFrequency);

#endif /* MOTOR_H_ */
