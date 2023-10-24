/*
 * network_task.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */


// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include <task_system_manager.h>
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
static osMutexId_t DataMutexHandle;

osThreadId_t SystemTaskHandle;
StaticTask_t SystemTaskControlBlock;

uint32_t SystemTaskBuffer[ 512 ];
const osThreadAttr_t SystemTask_attributes = {
		.name = "SystemTask",
		.cb_mem = &SystemTaskControlBlock,
		.cb_size = sizeof(SystemTaskControlBlock),
		.stack_mem = &SystemTaskBuffer[0],
		.stack_size = sizeof(SystemTaskBuffer),
		.priority = (osPriority_t) osPriorityNormal1,
};
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
void SystemTaskApp(void *argument);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t SJC_TaskSystemManager_SetSharedDataMutex(osMutexId_t MutexHandle)
{
	DataMutexHandle = MutexHandle;

	if (DataMutexHandle == NULL)
	{
		SJC_DebugOutput_printf("\r\n[OS] System manager task: Shared data mutex not available...");
		return SJC_RET_NOK;
	} else
	{
		return SJC_RET_OK;
	}
}

eSjcReturnType_t SJC_TaskSystemManager_Init(void)
{
	SystemTaskHandle = osThreadNew(SystemTaskApp, NULL, &SystemTask_attributes);

	if (SystemTaskHandle == NULL)
	{
		return SJC_RET_NOK;
	}
	else
	{
		return SJC_RET_OK;
	}
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void SystemTaskApp(void *argument) {
	/* USER CODE BEGIN SystemTaskApp */
	uint32_t tick;

	tick = osKernelGetTickCount();

	/* Infinite loop */
	for (;;) {
		tick += pdMS_TO_TICKS(SJC_SYSTEM_MANAGER_UPDATE_PERIOD_MS);
		if (osOK == osMutexAcquire(DataMutexHandle, SYSTEM_MANAGER_SHARED_DATA_MUTEX_WAIT_TIME_MS))
		{

			osMutexRelease(DataMutexHandle);

			osDelayUntil(tick);
		}
	}
	/* USER CODE END MainTaskApp */
}

// ------------------------------------------------------------------- //
