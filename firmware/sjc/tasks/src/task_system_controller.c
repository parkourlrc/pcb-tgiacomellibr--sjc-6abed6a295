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
#include "task_common.h"

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
#define SYSTEM_MANAGER_SHARED_DATA_MUTEX_WAIT_TIME_MS	( (uint32_t ) 2U)
// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
void SystemTaskApp(void *argument);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void SystemTaskApp(void *argument) {
	/* USER CODE BEGIN SystemTaskApp */

	if (SJC_RET_NOK == SJC_Network_Init(NETWORK_BASE_ADRESS, NETWORK_BAUD_RATE, 'N'))
	{
		SJC_SYSTEM_RESET();
	}

	uint32_t tick = osKernelGetTickCount();

	uint32_t timestamp;
	uint32_t cycle = 0;

	xSjcSampleTimeMeasurement_t Ts;

	/* Infinite loop */
	for (;;) {
		tick += pdMS_TO_TICKS(SJC_SYSTEM_CONTROLLER_UPDATE_PERIOD_MS);

		if (osOK == SJC_TaskCommon_SharedDataMutexTake(SYSTEM_MANAGER_SHARED_DATA_MUTEX_WAIT_TIME_MS))
		{
			SJC_Network_Update();
			SJC_ExecuteNetworkCommands();
			SJC_UpdateNetworkDataRegisters();

			SJC_TaskCommon_SharedDataMutexGive();

			SJC_GetTime(SJC_TIME_MS, &timestamp);

			if (++cycle % 200 == 0)
			{
				SJC_AnalogInterfaces_GetMeanSampleTime(&Ts);
				float MeanSampleTime_ms = 1000.0f*(Ts.timesteps)/(Ts.samples);
				SJC_DebugOutput_printf("\n\r[OS] Runtime (%lu ms) - Analog Ts: (%.3f ms)", timestamp, MeanSampleTime_ms);
				SJC_ToggleDigitalOutput(SJC_HEARTBEAT_OUTPUT);
			}

			osDelayUntil(tick);
		}
	}
	/* USER CODE END MainTaskApp */
}

// ------------------------------------------------------------------- //
