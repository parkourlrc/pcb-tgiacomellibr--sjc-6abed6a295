/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__PWM_SIGNAL_H
#define SJC__MODULES__PWM_SIGNAL_H

#include "system_types/include/types.h"

void PwmSignal_init(const eSjcPwmBaseFreq_t BaseFrequency);
void PwmSignal_enable(void);
void PwmSignal_disable(void);

float PwmSignal_getMinDutyCycle(void);
float PwmSignal_getMaxDutyCycle(void);
float PwmSignal_getDutyCycle(void);
eSjcReturnType_t PwmSignal_setDutyCycle(const float duty);

uint32_t PwmSignal_getFrequency(void);
uint32_t PwmSignal_setFrequency(const eSjcPwmBaseFreq_t BaseFrequency);

#endif /* SJC__MODULES__PWM_SIGNAL_H */
