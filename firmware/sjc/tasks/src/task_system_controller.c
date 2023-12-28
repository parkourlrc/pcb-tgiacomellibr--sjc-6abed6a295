/*
 * network_task.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */


// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "task_system_controller.h"
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
eSjcReturnType_t SJC_TaskSystemController_SetSharedDataMutex(osMutexId_t MutexHandle)
{
	DataMutexHandle = MutexHandle;

	if (DataMutexHandle == NULL)
	{
		SJC_DebugOutput_printf("\r\n[OS] System controller task: Shared data mutex not available...");
		return SJC_RET_NOK;
	} else
	{
		return SJC_Network_Init(NETWORK_BASE_ADRESS, NETWORK_BAUD_RATE, 'N');
	}
}

eSjcReturnType_t SJC_TaskSystemController_Init(void)
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

	uint32_t timestamp;
	uint32_t cycle = 0;

	SJC_EnableJointActuators();


	/* Infinite loop */
	for (;;) {
		tick += pdMS_TO_TICKS(SJC_SYSTEM_CONTROLLER_UPDATE_PERIOD_MS);
		if (osOK == osMutexAcquire(DataMutexHandle, SYSTEM_MANAGER_SHARED_DATA_MUTEX_WAIT_TIME_MS))
		{
			SJC_Network_Update();
			SJC_ExecuteNetworkCommands();
			SJC_UpdateNetworkDataRegisters();

			// temp
			// --------------------------------------------------------------------------------------------
			float ref;
			SJC_GetControlReference(&ref);
			SJC_SetJointActuatorVoltage(ref);
			// --------------------------------------------------------------------------------------------

			osMutexRelease(DataMutexHandle);

			SJC_GetTime(SJC_TIME_MS, &timestamp);

			if (++cycle % 100 == 0)
			{
				SJC_DebugOutput_printf("\n\r[OS] System controller (%lu ms)", timestamp);
				SJC_ToggleDigitalOutput(SJC_HEARTBEAT_OUTPUT);
			}

			osDelayUntil(tick);
		}
	}
	/* USER CODE END MainTaskApp */
}

// ------------------------------------------------------------------- //
