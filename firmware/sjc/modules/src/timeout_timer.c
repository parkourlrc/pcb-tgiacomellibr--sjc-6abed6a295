/*
 * motor.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */


// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "stm32f4xx.h"
// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "timeout_timer.h"
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
void SJC_Timer_Init(void)
{

}

uint32_t SJC_Timer_GetTimeMs(void)
{
	return (_GetTick());
}

uint32_t SJC_Timer_GetTimeSec(void)
{
	return (_GetTick()/1000);
}

void SJC_Timer_DelayMs(uint32_t delayInMs)
{
	uint32_t compval =  delayInMs;
	uint32_t start = _GetTick();
	while( (_GetTick() - start) < compval);  //This comparison tolerates overflow
}

void SJC_Timer_DelaySec(uint32_t delayInSec)
{
	uint32_t compval =  delayInSec*1000;
	uint32_t start = _GetTick();
	while( (_GetTick() - start) < compval);  //This comparison tolerates overflow
}

int32_t SJC_Timer_TimeoutCheckMs(uint32_t delayInMs, uint32_t seed)
{
	uint32_t compval = delayInMs;
	return ( !( (_GetTick() - seed) < compval) ); //This comparison tolerates overflow
}

uint32_t SJC_Timer_TimeoutCheckSec(uint32_t delayInSec, uint32_t seed)
{
	uint32_t compval = delayInSec*1000;
	return ( !( (_GetTick() - seed) < compval) ); //This comparison tolerates overflow
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
static __inline uint32_t _GetTick(void)
{
	return (HAL_GetTick());
}

// ------------------------------------------------------------------- //
