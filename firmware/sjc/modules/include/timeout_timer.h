/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__TIMEOUT_TIMER_H_
#define SJC__MODULES__TIMEOUT_TIMER_H_

#include "types.h"

void SJC_Timer_Init(void);
uint32_t SJC_Timer_GetTimeMs(void);
uint32_t SJC_Timer_GetTimeSec(void);

void SJC_Timer_DelayMs(uint32_t delayInMs);
void SJC_Timer_DelaySec(uint32_t delayInSec);

uint32_t SJC_Timer_TimeoutCheckMs(uint32_t delayInMs, uint32_t seed);
uint32_t SJC_Timer_TimeoutCheckSec(uint32_t delayInSec, uint32_t seed);

#endif /* MOTOR_H_ */
