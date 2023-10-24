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
#include "tim.h"
// ------------------------------------------------------------------- //

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "task_network.h"

#include "sjc.h"
#include "configs.h"
#include "network.h"
#include "debug_output.h"
#include "network_configs.h"
// ------------------------------------------------------------------- //


// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#define ANALOG_SENSORS_CONV_BUFFER_SIZE					( (uint32_t ) 32UL)
#define SENSORS_SHARED_DATA_MUTEX_WAIT_TIME_MS			( osWaitForever )
// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //
static osMutexId_t DataMutexHandle;

osThreadId_t NetworkTaskHandle;
StaticTask_t NetworkTaskControlBlock;

uint32_t NetworkTaskBuffer[ 512 ];
const osThreadAttr_t NetworkTask_attributes = {
		.name = "NetworkTask",
		.cb_mem = &NetworkTaskControlBlock,
		.cb_size = sizeof(NetworkTaskControlBlock),
		.stack_mem = &NetworkTaskBuffer[0],
		.stack_size = sizeof(NetworkTaskBuffer),
		.priority = (osPriority_t) osPriorityNormal,
};

// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
void NetworkTaskApp(void *argument);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t SJC_TaskNetwork_SetSharedDataMutex(osMutexId_t MutexHandle)
{
	DataMutexHandle = MutexHandle;

	if (DataMutexHandle == NULL)
	{
		SJC_DebugOutput_printf("\r\n[OS] Sensors task: Shared data mutex not available...");
		return SJC_RET_NOK;
	} else
	{
		return SJC_Network_Init(NETWORK_BASE_ADRESS);
	}
}

eSjcReturnType_t SJC_TaskNetwork_Init(void)
{
	NetworkTaskHandle = osThreadNew(NetworkTaskApp, NULL, &NetworkTask_attributes);

	if (NetworkTaskHandle == NULL)
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
void NetworkTaskApp(void *argument)
{
	/* USER CODE BEGIN MainTaskApp */

	uint32_t tick;
	tick = osKernelGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		tick += pdMS_TO_TICKS(SJC_NETWORK_UPDATE_PERIOD_MS);
		if (osOK == osMutexAcquire(DataMutexHandle, SENSORS_SHARED_DATA_MUTEX_WAIT_TIME_MS))
		{
			osMutexRelease(DataMutexHandle);
			SJC_UpdateNetworkRegisters();
			SJC_Network_Update();
			SJC_ReadNetworkCommands();
			static uint8_t idx = 0 ;

			idx++;
			if (idx == 20)
			{
				SJC_DebugOutput_printf("\r\n[OS] Time: %lu ms", SJC_Timer_GetTimeMs());
				idx = 0;
			}
			osMutexRelease(DataMutexHandle);
			osDelayUntil(tick);
		}
	}
	/* USER CODE END MainTaskApp */
}

//void TIM4_IRQHandler(void)
//{
//	/* USER CODE BEGIN TIM4_IRQn 0 */
//	if(LL_TIM_IsActiveFlag_UPDATE(TIM4) && LL_TIM_IsEnabledIT_UPDATE(TIM4))
//	{
//		LL_TIM_ClearFlag_UPDATE(TIM4);
//
//
////		SJC_DebugOutput_printf("\r\n%.2f", Iin);
//	}
//	/* USER CODE END TIM4_IRQn 0 */
//	/* USER CODE BEGIN TIM4_IRQn 1 */
//
//	/* USER CODE END TIM4_IRQn 1 */
//}
// ------------------------------------------------------------------- //
