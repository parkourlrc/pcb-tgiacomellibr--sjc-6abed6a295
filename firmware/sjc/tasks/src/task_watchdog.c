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

// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void WatchdogTaskApp(void *argument)
{
	/* USER CODE BEGIN WatchdogTaskApp */

	osDelay(SJC_TASK_INIT_DELAY);
	SJC_DebugOutput_printf("\r\n[OS] Watchdog task initiated!");

	uint32_t tick = osKernelGetTickCount();

	uint32_t timestamp;

	xSjcMeanSampleTimeMeasurement_t Ts;

	/* Infinite loop */
	for (;;) {
		tick += pdMS_TO_TICKS(SJC_DEBUG_TASK_UPDATE_PERIOD_MS);

		SJC_GetTime(SJC_TIME_MS, &timestamp);

		SJC_AnalogInterfaces_GetMeanSampleTime(&Ts);

		SJC_ToggleDigitalOutput(SJC_HEARTBEAT_OUTPUT);

		float MeanSampleTime_us = (Ts.AccumulatedTimerTicks)/(Ts.Samples);

		SJC_DebugOutput_printf("\n\r[OS] Runtime (%lu ms) - Analog Ts: (%.3f us)", timestamp, MeanSampleTime_us);


		osDelayUntil(tick);

	}
	/* USER CODE END MainTaskApp */
}

// ------------------------------------------------------------------- //
