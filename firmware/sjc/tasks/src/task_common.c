/*
 * common.c
 *
 *  Created on: Oct 4, 2023
 *      Author: Tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
// ------------------------------------------------------------------- //

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "tasks/include/task_common.h"
#include "base/include/configs.h"
#include "base/include/network_configs.h"
#include "base/include/sjc.h"
#include "modules/include/analog_interfaces.h"
#include "modules/include/data_logger.h"
#include "modules/include/debug_output.h"
#include "modules/include/modbus_network.h"
#include "modules/include/test_signals.h"

// ------------------------------------------------------------------- //

// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Common data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
#define DATA_LOGGER_QUEUE_LENGTH 10
#define DATA_LOGGER_ITEM_SIZE sizeof(xSjcJointControllerMeasurements_t)
static StaticQueue_t xDataLoggerStaticQueue;
uint8_t ucDataLoggerQueueStorageArea[DATA_LOGGER_QUEUE_LENGTH * DATA_LOGGER_ITEM_SIZE];
QueueHandle_t xDataLoggerQueueHandle;

osThreadId_t WatchdogTaskHandle;
osThreadId_t DataLoggerTaskHandle;
osThreadId_t NetworkTaskHandle;
// osThreadId_t ControllerTaskHandle;
// osThreadId_t MeasurementsTaskHandle;
osThreadId_t ControllerAndMeasurementsTaskHandle;

StaticTask_t WatchdogTaskControlBlock;
StaticTask_t DataLoggerTaskControlBlock;
StaticTask_t NetworkTaskControlBlock;
// StaticTask_t ControllerTaskControlBlock;
// StaticTask_t MeasurementsTaskControlBlock;
StaticTask_t ControllerAndMeasurementsTaskControlBlock;

uint32_t WatchdogTaskBuffer[768];
uint32_t DataLoggerTaskBuffer[3072];
uint32_t NetworkTaskBuffer[4096];
// uint32_t ControllerTaskBuffer[ 3072 ];
// uint32_t MeasurementsTaskBuffer[ 256 ];
uint32_t ControllerAndMeasurementsTaskBuffer[8192];

static TaskTimeMeasurement_t xTimeMeasurements[TIME_LOG_FUNCTIONS_NAME_SIZE];
static uint32_t xTaskHighWaterMarkMeasurements[TASK_HIGH_WTM_LIST_SIZE];

const osThreadAttr_t WatchdogTask_attributes = {
	.name = "WatchdogTask",
	.cb_mem = &WatchdogTaskControlBlock,
	.cb_size = sizeof(WatchdogTaskControlBlock),
	.stack_mem = &WatchdogTaskBuffer[0],
	.stack_size = sizeof(WatchdogTaskBuffer),
	.priority = (osPriority_t)osPriorityLow,
};

const osThreadAttr_t DataLoggerTask_attributes = {
	.name = "DataLoggerTask",
	.cb_mem = &DataLoggerTaskControlBlock,
	.cb_size = sizeof(DataLoggerTaskControlBlock),
	.stack_mem = &DataLoggerTaskBuffer[0],
	.stack_size = sizeof(DataLoggerTaskBuffer),
	.priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t NetworkTask_attributes = {
	.name = "NetworkTask",
	.cb_mem = &NetworkTaskControlBlock,
	.cb_size = sizeof(NetworkTaskControlBlock),
	.stack_mem = &NetworkTaskBuffer[0],
	.stack_size = sizeof(NetworkTaskBuffer),
	.priority = (osPriority_t)osPriorityAboveNormal7,
};

const osThreadAttr_t ControllerAndMeasurementsTask_attributes = {
	.name = "ControlMeasTask",
	.cb_mem = &ControllerAndMeasurementsTaskControlBlock,
	.cb_size = sizeof(ControllerAndMeasurementsTaskControlBlock),
	.stack_mem = &ControllerAndMeasurementsTaskBuffer[0],
	.stack_size = sizeof(ControllerAndMeasurementsTaskBuffer),
	.priority = (osPriority_t)osPriorityRealtime7,
};

// ------------------------------------------------------------------- //

// Common variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
extern void WatchdogTaskApp(void *argument);
extern void DataLoggerTaskApp(void *argument);
extern void NetworkTaskApp(void *argument);
extern void LoggerTaskApp(void *argument);
// extern void ControllerTaskApp(void *argument);
// extern void MeasurementsTaskApp(void *argument);
extern void ControllerMeasTaskApp(void *argument);

eSjcReturnType_t InitOsObjects(void);
eSjcReturnType_t InitOsThreads(void);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void SJC_TaskCommon_initObjects(void)
{
	/* Init Startup Objects */
	if (SJC_RET_NOK == InitOsObjects())
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Kernel initialization failed! Resetting...!");
		SJC_SYSTEM_RESET();
	}
}

void SJC_TaskCommon_initThreads(void)
{
	/* Init Startup Tasks */
	if (SJC_RET_NOK == InitOsThreads())
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Kernel initialization failed! Resetting...!");
		SJC_SYSTEM_RESET();
	}
}

float SJC_TaskCommon_GetTimeMeasurement(const eTimeLogFunctionList_t time_log_fcn)
{
	float result = 0.0f;

	if (time_log_fcn < TIME_LOG_FUNCTIONS_NAME_SIZE)
	{
		result = (xTimeMeasurements[time_log_fcn].Idx > 0) ? ((float)xTimeMeasurements[time_log_fcn].Ts_ms) / xTimeMeasurements[time_log_fcn].Idx : 0.0f;
		xTimeMeasurements[time_log_fcn] = TaskTimeMeasurementEmptyObj;
	}

	return result;
}

void SJC_TaskCommon_SetTimeMeasurement(const eTimeLogFunctionList_t time_log_fcn, const uint32_t time)
{
	if (time_log_fcn < TIME_LOG_FUNCTIONS_NAME_SIZE)
	{
		xTimeMeasurements[time_log_fcn].Idx++;
		xTimeMeasurements[time_log_fcn].Ts_ms += time;
	}
}

uint32_t SJC_TaskCommon_GetHighWaterMarkInfo(const eTaskHighWaterMarkInfoList_t task_name)
{
	return ((task_name < TASK_HIGH_WTM_LIST_SIZE) ? xTaskHighWaterMarkMeasurements[task_name] : 0);
}

void SJC_TaskCommon_UpdateHighWaterMarkInfo(void)
{
	xTaskHighWaterMarkMeasurements[TASK_HIGH_WTM_INFO_NETWORK_TASK] = uxTaskGetStackHighWaterMark(NetworkTaskHandle);
	xTaskHighWaterMarkMeasurements[TASK_HIGH_WTM_INFO_WATCHDOG_TASK] = uxTaskGetStackHighWaterMark(WatchdogTaskHandle);
	xTaskHighWaterMarkMeasurements[TASK_HIGH_WTM_INFO_DATA_LOGGER_TASK] = uxTaskGetStackHighWaterMark(DataLoggerTaskHandle);
	xTaskHighWaterMarkMeasurements[TASK_HIGH_WTM_INFO_CONTROLLER_MEAS_TASK] = uxTaskGetStackHighWaterMark(ControllerAndMeasurementsTaskHandle);
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t InitOsObjects(void)
{
	// Shared queues creation
	xDataLoggerQueueHandle = xQueueCreateStatic(DATA_LOGGER_QUEUE_LENGTH,
												DATA_LOGGER_ITEM_SIZE,
												ucDataLoggerQueueStorageArea,
												&xDataLoggerStaticQueue);

	if (NULL == xDataLoggerQueueHandle)
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Datalogger queue not initiated...");
		return SJC_RET_NOK;
	}

	return SJC_RET_OK;
}

eSjcReturnType_t InitOsThreads(void)
{
	// Tasks creation
	NetworkTaskHandle = osThreadNew(NetworkTaskApp, NULL, &NetworkTask_attributes);
	if (NULL == NetworkTaskHandle)
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Network task not initiated...");
		return SJC_RET_NOK;
	}

	DataLoggerTaskHandle = osThreadNew(DataLoggerTaskApp, NULL, &DataLoggerTask_attributes);
	if (NULL == DataLoggerTaskHandle)
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Data logger task not initiated...");
		return SJC_RET_NOK;
	}

	WatchdogTaskHandle = osThreadNew(WatchdogTaskApp, NULL, &WatchdogTask_attributes);
	if (NULL == WatchdogTaskHandle)
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Watchdog task not initiated...");
		return SJC_RET_NOK;
	}

	ControllerAndMeasurementsTaskHandle = osThreadNew(ControllerMeasTaskApp, NULL, &ControllerAndMeasurementsTask_attributes);
	if (NULL == ControllerAndMeasurementsTaskHandle)
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Controller and measurements task not initiated...");
		return SJC_RET_NOK;
	}

	return SJC_RET_OK;
}
// ------------------------------------------------------------------- //
