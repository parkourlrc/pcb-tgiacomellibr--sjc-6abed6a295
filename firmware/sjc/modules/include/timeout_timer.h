/*
 * timeout_timer.h
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  System uptime timer with millisecond/second delay and timeout utilities.
 */

#ifndef SJC__MODULES__TIMEOUT_TIMER_H_
#define SJC__MODULES__TIMEOUT_TIMER_H_

#include "system_types/include/types.h"

/**
 * @brief Initializes the timeout timer module.
 */
void TimeoutTimer_init(void);

/**
 * @brief Returns system uptime in milliseconds.
 * @return Uptime [ms].
 */
uint32_t TimeoutTimer_getTimeMs(void);

/**
 * @brief Returns system uptime in seconds.
 * @return Uptime [s].
 */
uint32_t TimeoutTimer_getTimeSec(void);

/**
 * @brief Blocking delay in milliseconds.
 * @param delayInMs Delay duration [ms].
 */
void TimeoutTimer_delayMs(uint32_t delayInMs);

/**
 * @brief Blocking delay in seconds.
 * @param delayInSec Delay duration [s].
 */
void TimeoutTimer_delaySec(uint32_t delayInSec);

/**
 * @brief Check if a timeout has elapsed since a given seed time.
 * @param delayInMs Timeout duration [ms].
 * @param seed Starting time reference [ms].
 * @return 1 if timed out, 0 otherwise.
 */
uint32_t TimeoutTimer_timeoutCheckMs(uint32_t delayInMs, uint32_t seed);

/**
 * @brief Check if a timeout has elapsed since a given seed time.
 * @param delayInSec Timeout duration [s].
 * @param seed Starting time reference [s].
 * @return 1 if timed out, 0 otherwise.
 */
uint32_t TimeoutTimer_timeoutCheckSec(uint32_t delayInSec, uint32_t seed);

#endif /* SJC__MODULES__TIMEOUT_TIMER_H_ */
