/*
 * timeout_timer.c
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  HAL tick-based system uptime counter with delay and timeout checks.
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "stm32f4xx_hal.h"
// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "modules/include/timeout_timer.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //
static __inline uint32_t _GetTick(void);

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void TimeoutTimer_init(void)
{
}

uint32_t TimeoutTimer_getTimeMs(void)
{
	return (_GetTick());
}

uint32_t TimeoutTimer_getTimeSec(void)
{
	return (_GetTick() / 1000);
}

void TimeoutTimer_delayMs(uint32_t delayInMs)
{
	uint32_t compval = delayInMs;
	uint32_t start = _GetTick();
	while ((_GetTick() - start) < compval)
		; // This comparison tolerates overflow
}

void TimeoutTimer_delaySec(uint32_t delayInSec)
{
	uint32_t compval = delayInSec * 1000;
	uint32_t start = _GetTick();
	while ((_GetTick() - start) < compval)
		; // This comparison tolerates overflow
}

uint32_t TimeoutTimer_timeoutCheckMs(uint32_t delayInMs, uint32_t seed)
{
	uint32_t compval = delayInMs;
	return (!((_GetTick() - seed) < compval)); // This comparison tolerates overflow
}

uint32_t TimeoutTimer_timeoutCheckSec(uint32_t delayInSec, uint32_t seed)
{
	uint32_t compval = delayInSec * 1000;
	return (!((_GetTick() - seed) < compval)); // This comparison tolerates overflow
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
static __inline uint32_t _GetTick(void)
{
	return (HAL_GetTick());
}

// ------------------------------------------------------------------- //
