/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__TIMEOUT_TIMER_H_
#define SJC__MODULES__TIMEOUT_TIMER_H_

#include "system_types/include/types.h"

void TimeoutTimer_init(void);
uint32_t TimeoutTimer_getTimeMs(void);
uint32_t TimeoutTimer_getTimeSec(void);

void TimeoutTimer_delayMs(uint32_t delayInMs);
void TimeoutTimer_delaySec(uint32_t delayInSec);

uint32_t TimeoutTimer_timeoutCheckMs(uint32_t delayInMs, uint32_t seed);
uint32_t TimeoutTimer_timeoutCheckSec(uint32_t delayInSec, uint32_t seed);

#endif /* MOTOR_H_ */
