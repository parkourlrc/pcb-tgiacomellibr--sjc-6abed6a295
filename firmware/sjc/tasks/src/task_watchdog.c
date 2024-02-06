/*
 * network_task.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */


// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "main.h"
#include "cmsis_os.h"
// ------------------------------------------------------------------- //

// NETWORK includes -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "sjc.h"
#include "configs.h"
#include "network.h"
#include "analog_interfaces.h"
#include "debug_output.h"
#include "timeout_timer.h"
#include "network_configs.h"

// ------------------------------------------------------------------- //


// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#define NETWORK_SHARED_DATA_MUTEX_WAIT_TIME_MS			( (uint32_t ) 10U)
// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //
static osTimerId_t xHeartbeatTimerId;
// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //
//volatile unsigned long ulHighFrequencyTimerTicks;
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void vHeartbeatTimerCallback(void *argument);
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void WatchdogTaskApp(void *argument)
{
	/* USER CODE BEGIN WatchdogTaskApp */

	osDelay(SJC_TASK_INIT_DELAY);
	SJC_DebugOutput_SystemPrintf("\r\n[OS] Watchdog task initiated!");

	uint32_t tick = osKernelGetTickCount();

	xHeartbeatTimerId = osTimerNew(vHeartbeatTimerCallback, osTimerPeriodic, (void *)0, NULL);
	configASSERT(xHeartbeatTimerId != NULL);
	osTimerStart(xHeartbeatTimerId, SJC_HEARTBEAT_CALLBACK_PERIOD_MS);

	uint32_t timestamp;

	xSjcMeanSampleTimeMeasurement_t Ts;

	//	uint32_t counts = 0;

	/* Infinite loop */
	for (;;) {
		tick += pdMS_TO_TICKS(SJC_DEBUG_TASK_UPDATE_PERIOD_MS);

		SJC_GetTime(SJC_TIME_MS, &timestamp);

		SJC_AnalogInterfaces_GetMeanConversionDelay(&Ts);

//		SJC_ToggleDigitalOutput(SJC_HEARTBEAT_OUTPUT);

		float MeanSampleTime_us = (1.0f*Ts.AccumulatedTimerTicks)/(1.0f*Ts.Samples);

		SJC_DebugOutput_SystemPrintf("\n\r[OS] Runtime (%lu ms) - Analog. Conv. Delay: (%.3f us)", timestamp, MeanSampleTime_us);

//		if (++counts == 10)
//		{
//			counts = 0;
//			SJC_DebugOutput_PresentRuntimeStatistics();
//		}

		osDelayUntil(tick);

	}
	/* USER CODE END MainTaskApp */
}

void vHeartbeatTimerCallback(void *argument)
{
	SJC_ToggleDigitalOutput(SJC_HEARTBEAT_OUTPUT);
}

//void configureTimerForRunTimeStats(void)
//{
//	MX_TIM6_Init();
//
//	ulHighFrequencyTimerTicks = 0;
//
//	LL_TIM_SetCounter(TIM6, 0);
//	LL_TIM_EnableIT_UPDATE(TIM6);
//	LL_TIM_EnableCounter(TIM6);
//}

//unsigned long getRunTimeCounterValue(void)
//{
//	return osKernelGetTickCount();
//}

//void TIM6_DAC_IRQHandler(void)
//{
//	if (LL_TIM_IsActiveFlag_UPDATE(TIM6) & LL_TIM_IsEnabledIT_UPDATE(TIM6))
//	{
//		LL_TIM_ClearFlag_UPDATE(TIM6);
//		ulHighFrequencyTimerTicks++;
//	}
//}

//void TIM6_DAC_IRQHandler(void)
//{
//	if (LL_TIM_IsActiveFlag_UPDATE(TIM6) & LL_TIM_IsEnabledIT_UPDATE(TIM6))
//	{
//		LL_TIM_ClearFlag_UPDATE(TIM6);
//		ulHighFrequencyTimerTicks++;
//	}
//}
// ------------------------------------------------------------------- //
