/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__TIMEOUT_TIMER_H_
#define SJC__MODULES__TIMEOUT_TIMER_H_

#include "types.h"

void Timer_init(void);
uint32_t Timer_getTimeMs(void);
uint32_t Timer_getTimeSec(void);

void Timer_delayMs(uint32_t delayInMs);
void Timer_delaySec(uint32_t delayInSec);

uint32_t Timer_timeoutCheckMs(uint32_t delayInMs, uint32_t seed);
uint32_t Timer_timeoutCheckSec(uint32_t delayInSec, uint32_t seed);

#endif /* MOTOR_H_ */
