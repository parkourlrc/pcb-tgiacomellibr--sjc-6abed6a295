/*
 * task_network.c
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  Modbus network polling and register update task.
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "cmsis_os.h"
#include "main.h"
#include "modules/include/modbus_network.h"
#include "port.h"
// ------------------------------------------------------------------- //

// NETWORK includes -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "base/include/configs.h"
#include "base/include/sjc.h"
#include "common/include/network_configs.h"
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
static uint32_t TaskTic, TaskToc, TaskLastTic;
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
#if SJC_HW_VERSION_MAJOR > 1
	uint32_t xNetworkAdress = 0;
	ModbusNetwork_getAddress(&xNetworkAdress);

	if (SJC_RET_NOK == ModbusNetwork_init(xNetworkAdress))
#else
	if (SJC_RET_NOK == ModbusNetwork_init(NETWORK_BASE_ADDRESS))
#endif
	{
		SJC_SYSTEM_RESET();
	}

	// SJC info
	SJC_presentSystemInfo();

	DebugOutput_systemPrintf("\r\n[OS] Network task initiated!"
							 "\r\n[OS] Modbus communication enabled!");

#ifndef USE_QUEUE_BASED_EVENT_HANDLER
	TickType_t networkLastWakeTime = xTaskGetTickCount();
#endif
	SJC_getTime(SJC_TIME_MS, &TaskLastTic);

	/* Infinite loop */
	for (;;)
	{
		SJC_getTime(SJC_TIME_MS, &TaskTic);
		SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_NETWORK_TASK_PERIOD_MS, (TaskTic - TaskLastTic));
		TaskLastTic = TaskTic;

		SJC_executeNetworkDaemon();

		SJC_getTime(SJC_TIME_MS, &TaskToc);

		SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_NETWORK_TASK_EXEC_MS, (TaskToc - TaskTic));

#ifndef USE_QUEUE_BASED_EVENT_HANDLER
		vTaskDelayUntil(&networkLastWakeTime, pdMS_TO_TICKS(SJC_NETWORK_TASK_UPDATE_PERIOD_MS));
#endif
	}
	/* USER CODE END MainTaskApp */
}
// ------------------------------------------------------------------- //
