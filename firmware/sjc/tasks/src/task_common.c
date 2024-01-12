/*
 * common.c
 *
 *  Created on: Oct 4, 2023
 *      Author: Tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "task_common.h"

// ------------------------------------------------------------------- //

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "debug_output.h"
#include "analog_interfaces.h"
// ------------------------------------------------------------------- //


// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#define BINARY_SEMPHR_COUNT_SIZE							((uint32_t) 1U)
#define BINARY_SEMPHR_INITIAL_COUNT							((uint32_t) 1U)
// ------------------------------------------------------------------- //

// Common data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
osSemaphoreId_t SharedDataSemaphoreHandle;
osSemaphoreId_t SerialDebugSemaphoreHandle;
osSemaphoreId_t UsbDebugSemaphoreHandle;

StaticSemaphore_t SharedDataSemaphoreControlBlock;
StaticSemaphore_t SerialDebugSemaphoreControlBlock;
StaticSemaphore_t UsbDebugSemaphoreControlBlock;

osThreadId_t WatchdogTaskHandle;
osThreadId_t NetworkTaskHandle;
osThreadId_t LoggerTaskHandle;
osThreadId_t ControllerTaskHandle;
osThreadId_t AnalogTaskHandle;

StaticTask_t WatchdogTaskControlBlock;
StaticTask_t NetworkTaskControlBlock;
StaticTask_t LoggerTaskControlBlock;
StaticTask_t ControllerTaskControlBlock;
StaticTask_t AnalogTaskControlBlock;

uint32_t WatchdogTaskBuffer[ 256 ];
uint32_t NetworkTaskBuffer[ 256 ];
uint32_t LoggerTaskBuffer[ 512 ];
uint32_t ControllerTaskBuffer[ 4096 ];
uint32_t AnalogTaskBuffer[ 256 ];

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

const osSemaphoreAttr_t SerialDebugSemaphore_attributes = {
		.name = "SerialDebugSemaphore",
		.cb_mem = &SerialDebugSemaphoreControlBlock,
		.cb_size = sizeof(SerialDebugSemaphoreControlBlock)
};

const osSemaphoreAttr_t UsbDebugSemaphore_attributes = {
		.name = "UsbDebugSemaphore",
		.cb_mem = &UsbDebugSemaphoreControlBlock,
		.cb_size = sizeof(UsbDebugSemaphoreControlBlock)
};
// ------------------------------------------------------------------- //

// Common variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
extern void WatchdogTaskApp(void *argument);
extern void NetworkTaskApp(void *argument);
extern void LoggerTaskApp(void *argument);
extern void ControllerTaskApp(void *argument);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t SJC_TaskCommon_InitThreads(void)
{

	// Semaphores creation
	SharedDataSemaphoreHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &SharedDataSemaphore_attributes);

	SerialDebugSemaphoreHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &SerialDebugSemaphore_attributes);

	UsbDebugSemaphoreHandle = osSemaphoreNew(BINARY_SEMPHR_COUNT_SIZE, BINARY_SEMPHR_INITIAL_COUNT, &UsbDebugSemaphore_attributes);

	if (SharedDataSemaphoreHandle == NULL)
	{
		SJC_DebugOutput_printf("\r\n[OS] Shared data mutex not created...");
		return SJC_RET_NOK;
	} else
	{
		osSemaphoreRelease(SharedDataSemaphoreHandle);
	}

	if (SerialDebugSemaphoreHandle == NULL)
	{
		SJC_DebugOutput_printf("\r\n[OS] Serial debug semaphore not created...");
		return SJC_RET_NOK;
	} else
	{
		osSemaphoreRelease(SerialDebugSemaphoreHandle);
	}

	if (UsbDebugSemaphoreHandle == NULL)
	{
		SJC_DebugOutput_printf("\r\n[OS] USB debug semaphore not created...");
		return SJC_RET_NOK;
	} else
	{
		osSemaphoreRelease(UsbDebugSemaphoreHandle);
	}

	// Tasks creation
	WatchdogTaskHandle = osThreadNew(WatchdogTaskApp, NULL, &WatchdogTask_attributes);
	NetworkTaskHandle = osThreadNew(NetworkTaskApp, NULL, &NetworkTask_attributes);
	ControllerTaskHandle = osThreadNew(ControllerTaskApp, NULL, &ControllerTask_attributes);
	//	LoggerTaskHandle = osThreadNew(LoggerTaskApp, NULL, &LoggerTask_attributes);

	if (NULL == WatchdogTaskHandle)
	{
		SJC_DebugOutput_printf("\r\n[OS] Watchdog task not initiated...");
		return SJC_RET_NOK;
	}

	if (NULL == NetworkTaskHandle)
	{
		SJC_DebugOutput_printf("\r\n[OS] Network task not initiated...");
		return SJC_RET_NOK;
	}

	if (NULL == ControllerTaskHandle)
	{
		SJC_DebugOutput_printf("\r\n[OS] Joint controller task not initiated...");
		return SJC_RET_NOK;
	}

	//	if (NULL == LoggerTaskHandle)
	//	{
	//		SJC_DebugOutput_printf("\r\n[OS] Data logger task not initiated...");
	//		return SJC_RET_NOK;
	//	}

	return SJC_RET_OK;
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

osStatus_t SJC_TaskCommon_SerialDebugSemaphoreTake(const uint32_t timeout)
{
	if (SerialDebugSemaphoreHandle == NULL)
	{
		return osErrorResource;
	}
	else
	{
		return osSemaphoreAcquire(SerialDebugSemaphoreHandle, timeout);
	}
}

osStatus_t SJC_TaskCommon_SerialDebugSemaphoreGive(void)
{
	if (SerialDebugSemaphoreHandle == NULL)
	{
		return osErrorResource;
	}
	else
	{
		return osSemaphoreRelease(SerialDebugSemaphoreHandle);
	}
}

osStatus_t SJC_TaskCommon_UsbDebugSemaphoreTake(const uint32_t timeout)
{
	if (UsbDebugSemaphoreHandle == NULL)
	{
		return osErrorResource;
	}
	else
	{
		return osSemaphoreAcquire(UsbDebugSemaphoreHandle, timeout);
	}
}

osStatus_t SJC_TaskCommon_UsbDebugSemaphoreGive(void)
{
	if (UsbDebugSemaphoreHandle == NULL)
	{
		return osErrorResource;
	}
	else
	{
		return osSemaphoreRelease(UsbDebugSemaphoreHandle);
	}
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //


