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
#include "network_configs.h"
#include "task_common.h"

// ------------------------------------------------------------------- //

// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //
static uint32_t TaskTic, TaskToc;
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void NetworkTaskApp(void *argument)
{
	/* USER CODE BEGIN NetworkTaskApp */
	if (SJC_RET_NOK == SJC_Network_Init(NETWORK_BASE_ADRESS))
	{
		SJC_SYSTEM_RESET();
	}

	SJC_DebugOutput_SystemPrintf("\r\n[OS] Network task initiated!"
								 "\r\n[OS] Modbus communication enabled!");

	uint32_t tick = osKernelGetTickCount();

	/* Infinite loop */
	for (;;)
	{
		tick += pdMS_TO_TICKS(SJC_NETWORK_TASK_UPDATE_PERIOD_MS);

		SJC_GetTime(SJC_TIME_MS, &TaskTic);

		//SJC_ProcessNetworkData();
		SJC_ExecuteNetworkDaemon();
		SJC_UpdateInputRegisters();
		SJC_UpdateHoldingRegisters();

		SJC_GetTime(SJC_TIME_MS, &TaskToc);

		SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_NETWORK_TASK_RUNTIME, (TaskToc-TaskTic));



		osDelayUntil(tick);
	}
	/* USER CODE END MainTaskApp */
}

// ------------------------------------------------------------------- //
