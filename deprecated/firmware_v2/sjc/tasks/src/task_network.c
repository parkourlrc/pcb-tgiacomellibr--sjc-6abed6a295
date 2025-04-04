/*
 * network_task.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "cmsis_os.h"
#include "main.h"
#include "modules/include/modbus_network.h"
// ------------------------------------------------------------------- //

// NETWORK includes -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "base/include/configs.h"
#include "base/include/network_configs.h"
#include "base/include/sjc.h"
#include "modules/include/analog_interfaces.h"
#include "modules/include/debug_output.h"
#include "tasks/include/task_common.h"
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
	uint32_t xNetworkAdress = 0;
	ModbusNetwork_getAddress(&xNetworkAdress);

	if (SJC_RET_NOK == ModbusNetwork_init(xNetworkAdress))
	{
		SJC_SYSTEM_RESET();
	}

	// SJC info
	SJC_presentSystemInfo();

	DebugOutput_systemPrintf("\r\n[OS] Network task initiated!"
							 "\r\n[OS] Modbus communication enabled!");

	uint32_t tick = osKernelGetTickCount();

	/* Infinite loop */
	for (;;)
	{
		tick += pdMS_TO_TICKS(SJC_NETWORK_TASK_UPDATE_PERIOD_MS);

		SJC_getTime(SJC_TIME_MS, &TaskTic);

		SJC_executeNetworkDaemon();
		SJC_updateInputRegisters();
		SJC_updateHoldingRegisters();

		SJC_getTime(SJC_TIME_MS, &TaskToc);

		SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_NETWORK_TASK_RUNTIME, (TaskToc - TaskTic));

		osDelayUntil(tick);
	}
	/* USER CODE END MainTaskApp */
}
// ------------------------------------------------------------------- //
