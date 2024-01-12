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


// ------------------------------------------------------------------- //

// Common data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
osMutexId_t SharedDataMutexHandle;
osMutexId_t SerialDebugMutexHandle;
osMutexId_t UsbDebugMutexHandle;

StaticSemaphore_t SharedDataMutexControlBlock;
StaticSemaphore_t SerialDebugMutexControlBlock;
StaticSemaphore_t UsbDebugMutexControlBlock;

osThreadId_t SystemTaskHandle;
osThreadId_t LoggerTaskHandle;
osThreadId_t ControllerTaskHandle;

StaticTask_t SystemTaskControlBlock;
StaticTask_t LoggerTaskControlBlock;
StaticTask_t ControllerTaskControlBlock;

uint32_t SystemTaskBuffer[ 256 ];
uint32_t LoggerTaskBuffer[ 512 ];
uint32_t ControllerTaskBuffer[ 4096 ];

const osThreadAttr_t SystemTask_attributes = {
		.name = "SystemTask",
		.cb_mem = &SystemTaskControlBlock,
		.cb_size = sizeof(SystemTaskControlBlock),
		.stack_mem = &SystemTaskBuffer[0],
		.stack_size = sizeof(SystemTaskBuffer),
		.priority = (osPriority_t) osPriorityNormal1,
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
		.priority = (osPriority_t) osPriorityBelowNormal,
};

const osMutexAttr_t SharedDataMutex_attributes = {
		.name = "SharedDataMutex",
		.cb_mem = &SharedDataMutexControlBlock,
		.cb_size = sizeof(SharedDataMutexControlBlock),
};

const osMutexAttr_t SerialDebugMutex_attributes = {
		.name = "SerialDebugMutex",
		.cb_mem = &SerialDebugMutexControlBlock,
		.cb_size = sizeof(SerialDebugMutexControlBlock),
};

const osMutexAttr_t UsbDebugMutex_attributes = {
		.name = "UsbDebugMutex",
		.cb_mem = &UsbDebugMutexControlBlock,
		.cb_size = sizeof(UsbDebugMutexControlBlock),
};
// ------------------------------------------------------------------- //

// Common variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
extern void SystemTaskApp(void *argument);
extern void LoggerTaskApp(void *argument);
extern void ControllerTaskApp(void *argument);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t SJC_TaskCommon_InitThreads(void)
{

	// Mutex creation
	SharedDataMutexHandle = osMutexNew(&SharedDataMutex_attributes);
	SerialDebugMutexHandle = osMutexNew(&SerialDebugMutex_attributes);
	UsbDebugMutexHandle = osMutexNew(&UsbDebugMutex_attributes);

	if (SharedDataMutexHandle == NULL)
	{
		SJC_DebugOutput_printf("\r\n[OS] Shared data mutex not created...");
		return SJC_RET_NOK;
	} else
	{
		osMutexRelease(SharedDataMutexHandle);
		SJC_DebugOutput_printf("\r\n[OS] Shared data mutex created!");
	}

	if (SerialDebugMutexHandle == NULL)
	{
		SJC_DebugOutput_printf("\r\n[OS] Serial debug mutex not created...");
		return SJC_RET_NOK;
	} else
	{
		osMutexRelease(SerialDebugMutexHandle);
		SJC_DebugOutput_printf("\r\n[OS] Serial debug mutex created!");
	}

	if (UsbDebugMutexHandle == NULL)
	{
		SJC_DebugOutput_printf("\r\n[OS] USB debug mutex not created...");
		return SJC_RET_NOK;
	} else
	{
		osMutexRelease(UsbDebugMutexHandle);
		SJC_DebugOutput_printf("\r\n[OS] USB Debug mutex created!");
	}

	// Tasks creation

	SystemTaskHandle = osThreadNew(SystemTaskApp, NULL, &SystemTask_attributes);
	//	LoggerTaskHandle = osThreadNew(LoggerTaskApp, NULL, &LoggerTask_attributes);
	ControllerTaskHandle = osThreadNew(ControllerTaskApp, NULL, &ControllerTask_attributes);

	if (NULL == SystemTaskHandle)
	{
		SJC_DebugOutput_printf("\r\n[OS] System manager task not initiated...");
		return SJC_RET_NOK;
	}
	else
	{
		SJC_DebugOutput_printf("\r\n[OS] System manager task created!");
	}

	//	if (NULL == LoggerTaskHandle)
	//	{
	//		SJC_DebugOutput_printf("\r\n[OS] Data logger task created!");
	//	}
	//	else
	//	{
	//		SJC_DebugOutput_printf("\r\n[OS] Data logger task not initiated...");
	//		return SJC_RET_NOK;
	//	}

	if (NULL == ControllerTaskHandle)
	{
		SJC_DebugOutput_printf("\r\n[OS] Joint controller task not initiated...");
		return SJC_RET_NOK;
	}
	else
	{
		SJC_DebugOutput_printf("\r\n[OS] Joint controller task created!");
	}


	return SJC_RET_OK;
}

osStatus_t SJC_TaskCommon_SharedDataMutexTake(const uint32_t timeout)
{
	if (SharedDataMutexHandle == NULL)
	{
		return osErrorResource;
	}
	else
	{
		return osMutexAcquire(SharedDataMutexHandle, timeout);
	}
}

osStatus_t SJC_TaskCommon_SharedDataMutexGive(void)
{
	if (SharedDataMutexHandle == NULL)
	{
		return osErrorResource;
	}
	else
	{
		return osMutexRelease(SharedDataMutexHandle);
	}
}

osStatus_t SJC_TaskCommon_SerialDebugMutexTake(const uint32_t timeout)
{
	if (SerialDebugMutexHandle == NULL)
	{
		return osErrorResource;
	}
	else
	{
		return osMutexAcquire(SerialDebugMutexHandle, timeout);
	}
}

osStatus_t SJC_TaskCommon_SerialDebugMutexGive(void)
{
	if (SerialDebugMutexHandle == NULL)
	{
		return osErrorResource;
	}
	else
	{
		return osMutexRelease(SerialDebugMutexHandle);
	}
}

osStatus_t SJC_TaskCommon_UsbDebugMutexTake(const uint32_t timeout)
{
	if (UsbDebugMutexHandle == NULL)
	{
		return osErrorResource;
	}
	else
	{
		return osMutexAcquire(UsbDebugMutexHandle, timeout);
	}
}

osStatus_t SJC_TaskCommon_UsbDebugMutexGive(void)
{
	if (UsbDebugMutexHandle == NULL)
	{
		return osErrorResource;
	}
	else
	{
		return osMutexRelease(UsbDebugMutexHandle);
	}
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //


