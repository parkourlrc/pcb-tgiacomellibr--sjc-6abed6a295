/*
 * common.c
 *
 *  Created on: Oct 4, 2023
 *      Author: Tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "configs.h"
#include "debug_output.h"
#include "analog_interfaces.h"
// ------------------------------------------------------------------- //


// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "task_common.h"

// ------------------------------------------------------------------- //

// Common data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
osSemaphoreId_t SharedDataSemaphoreHandle;
StaticSemaphore_t SharedDataSemaphoreControlBlock;

osThreadId_t WatchdogTaskHandle;
osThreadId_t NetworkTaskHandle;
osThreadId_t LoggerTaskHandle;
osThreadId_t ControllerTaskHandle;
osThreadId_t MeasurementsTaskHandle;

StaticTask_t WatchdogTaskControlBlock;
StaticTask_t NetworkTaskControlBlock;
StaticTask_t LoggerTaskControlBlock;
StaticTask_t ControllerTaskControlBlock;
StaticTask_t MeasurementsTaskControlBlock;

uint32_t WatchdogTaskBuffer[ 256 ];
uint32_t NetworkTaskBuffer[ 256 ];
uint32_t LoggerTaskBuffer[ 512 ];
uint32_t ControllerTaskBuffer[ 4096 ];
uint32_t MeasurementsTaskBuffer[ 256 ];

const osThreadAttr_t WatchdogTask_attributes = {
		.name = "WatchdogTask",
		.cb_mem = &WatchdogTaskControlBlock,
		.cb_size = sizeof(WatchdogTaskControlBlock),
		.stack_mem = &WatchdogTaskBuffer[0],
		.stack_size = sizeof(WatchdogTaskBuffer),
		.priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t NetworkTask_attributes = {
		.name = "NetworkTask",
		.cb_mem = &NetworkTaskControlBlock,
		.cb_size = sizeof(NetworkTaskControlBlock),
		.stack_mem = &NetworkTaskBuffer[0],
		.stack_size = sizeof(NetworkTaskBuffer),
		.priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t ControllerTask_attributes = {
		.name = "ControllerTask",
		.cb_mem = &ControllerTaskControlBlock,
		.cb_size = sizeof(ControllerTaskControlBlock),
		.stack_mem = &ControllerTaskBuffer[0],
		.stack_size = sizeof(ControllerTaskBuffer),
		.priority = (osPriority_t) osPriorityRealtime,
};

const osThreadAttr_t MeasurementsTask_attributes = {
		.name = "MeasurementsTask",
		.cb_mem = &MeasurementsTaskControlBlock,
		.cb_size = sizeof(MeasurementsTaskControlBlock),
		.stack_mem = &MeasurementsTaskBuffer[0],
		.stack_size = sizeof(MeasurementsTaskBuffer),
		.priority = (osPriority_t) osPriorityRealtime7,
};

const osThreadAttr_t LoggerTask_attributes = {
		.name = "LoggerTask",
		.cb_mem = &LoggerTaskControlBlock,
		.cb_size = sizeof(LoggerTaskControlBlock),
		.stack_mem = &LoggerTaskBuffer[0],
		.stack_size = sizeof(LoggerTaskBuffer),
		.priority = (osPriority_t) osPriorityAboveNormal,
};

const osSemaphoreAttr_t SharedDataSemaphore_attributes = {
		.name = "SharedDataSemaphore",
		.cb_mem = &SharedDataSemaphoreControlBlock,
		.cb_size = sizeof(SharedDataSemaphoreControlBlock),
};
// ------------------------------------------------------------------- //

// Common variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
void CommonTaskApp(void *argument);
extern void WatchdogTaskApp(void *argument);
extern void NetworkTaskApp(void *argument);
extern void LoggerTaskApp(void *argument);
extern void ControllerTaskApp(void *argument);
extern void MeasurementsTaskApp(void *argument);

eSjcReturnType_t InitOsThreads(void);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void SJC_TaskCommon_Init(void)
{
	if (SJC_RET_NOK == InitOsThreads())
	{
		SJC_DebugOutput_printf("\r\n[OS FAULT] Kernel initialization failed! Resetting...!");
		SJC_SYSTEM_RESET();
	}
}


osStatus_t SJC_TaskCommon_SharedDataSemaphoreTake(const uint32_t timeout)
{
	if (SharedDataSemaphoreHandle == NULL)
	{
		return osErrorResource;
	}
	else
	{
		return osSemaphoreAcquire(SharedDataSemaphoreHandle, timeout);
	}
}

osStatus_t SJC_TaskCommon_SharedDataSemaphoreGive(void)
{
	if (SharedDataSemaphoreHandle == NULL)
	{
		return osErrorResource;
	}
	else
	{
		return osSemaphoreRelease(SharedDataSemaphoreHandle);
	}
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t InitOsThreads(void)
{
	// Semaphores creation
	SharedDataSemaphoreHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &SharedDataSemaphore_attributes);

	if (SharedDataSemaphoreHandle == NULL)
	{
		SJC_DebugOutput_printf("\r\n[OS FAULT] Shared data mutex not created...");
		return SJC_RET_NOK;
	} else
	{
		osSemaphoreRelease(SharedDataSemaphoreHandle);
	}

	// Tasks creation
	WatchdogTaskHandle = osThreadNew(WatchdogTaskApp, NULL, &WatchdogTask_attributes);
	NetworkTaskHandle = osThreadNew(NetworkTaskApp, NULL, &NetworkTask_attributes);
	ControllerTaskHandle = osThreadNew(ControllerTaskApp, NULL, &ControllerTask_attributes);
	MeasurementsTaskHandle = osThreadNew(MeasurementsTaskApp, NULL, &MeasurementsTask_attributes);
	//	LoggerTaskHandle = osThreadNew(LoggerTaskApp, NULL, &LoggerTask_attributes);

	if (NULL == WatchdogTaskHandle)
	{
		SJC_DebugOutput_printf("\r\n[OS FAULT] Watchdog task not initiated...");
		return SJC_RET_NOK;
	}

	if (NULL == NetworkTaskHandle)
	{
		SJC_DebugOutput_printf("\r\n[OS FAULT] Network task not initiated...");
		return SJC_RET_NOK;
	}

	if (NULL == ControllerTaskHandle)
	{
		SJC_DebugOutput_printf("\r\n[OS FAULT] Joint controller task not initiated...");
		return SJC_RET_NOK;
	}

	if (NULL == MeasurementsTaskHandle)
	{
		SJC_DebugOutput_printf("\r\n[OS FAULT] Analog measurements task not initiated...");
		return SJC_RET_NOK;
	}

	//	if (NULL == LoggerTaskHandle)
	//	{
	//		SJC_DebugOutput_printf("\r\n[OS FAULT] Data logger task not initiated...");
	//		return SJC_RET_NOK;
	//	}

	return SJC_RET_OK;
}
// ------------------------------------------------------------------- //


