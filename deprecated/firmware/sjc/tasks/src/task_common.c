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
#include "modules/include/modbus_network.h"
#include "base/include/sjc.h"
#include "base/include/configs.h"
#include "base/include/network_configs.h"
#include "modules/include/analog_interfaces.h"
#include "modules/include/data_logger.h"
#include "modules/include/debug_output.h"
#include "modules/include/test_signals.h"
#include "tasks/include/task_common.h"

// ------------------------------------------------------------------- //

// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //

// Common data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
#define DATA_LOGGER_QUEUE_LENGTH    10
#define DATA_LOGGER_ITEM_SIZE       sizeof( xSjcJointControllerMeasurements_t )
static StaticQueue_t xDataLoggerStaticQueue;
uint8_t ucDataLoggerQueueStorageArea[ DATA_LOGGER_QUEUE_LENGTH * DATA_LOGGER_ITEM_SIZE ];
QueueHandle_t xDataLoggerQueueHandle;


osThreadId_t WatchdogTaskHandle;
osThreadId_t DataLoggerTaskHandle;
osThreadId_t NetworkTaskHandle;
//osThreadId_t ControllerTaskHandle;
//osThreadId_t MeasurementsTaskHandle;
osThreadId_t ControllerAndMeasurementsTaskHandle;


StaticTask_t WatchdogTaskControlBlock;
StaticTask_t DataLoggerTaskControlBlock;
StaticTask_t NetworkTaskControlBlock;
//StaticTask_t ControllerTaskControlBlock;
//StaticTask_t MeasurementsTaskControlBlock;
StaticTask_t ControllerAndMeasurementsTaskControlBlock;

uint32_t WatchdogTaskBuffer[ 256 ];
uint32_t DataLoggerTaskBuffer[ 2048 ];
uint32_t NetworkTaskBuffer[ 2048 ];
//uint32_t ControllerTaskBuffer[ 3072 ];
//uint32_t MeasurementsTaskBuffer[ 256 ];
uint32_t ControllerAndMeasurementsTaskBuffer[ 4096 ];

static TaskTimeMeasurement_t xTimeMeasurements[TIME_LOG_FUNCTIONS_NAME_SIZE];
static uint32_t xTaskHighWaterMarkMeasurements[TASK_HIGH_WTM_LIST_SIZE];


const osThreadAttr_t WatchdogTask_attributes = {
		.name = "WatchdogTask",
		.cb_mem = &WatchdogTaskControlBlock,
		.cb_size = sizeof(WatchdogTaskControlBlock),
		.stack_mem = &WatchdogTaskBuffer[0],
		.stack_size = sizeof(WatchdogTaskBuffer),
		.priority = (osPriority_t) osPriorityLow,
};

const osThreadAttr_t DataLoggerTask_attributes = {
		.name = "DataLoggerTask",
		.cb_mem = &DataLoggerTaskControlBlock,
		.cb_size = sizeof(DataLoggerTaskControlBlock),
		.stack_mem = &DataLoggerTaskBuffer[0],
		.stack_size = sizeof(DataLoggerTaskBuffer),
		.priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t NetworkTask_attributes = {
		.name = "NetworkTask",
		.cb_mem = &NetworkTaskControlBlock,
		.cb_size = sizeof(NetworkTaskControlBlock),
		.stack_mem = &NetworkTaskBuffer[0],
		.stack_size = sizeof(NetworkTaskBuffer),
		.priority = (osPriority_t) osPriorityAboveNormal7,
};

//const osThreadAttr_t ControllerTask_attributes = {
//		.name = "ControllerTask",
//		.cb_mem = &ControllerTaskControlBlock,
//		.cb_size = sizeof(ControllerTaskControlBlock),
//		.stack_mem = &ControllerTaskBuffer[0],
//		.stack_size = sizeof(ControllerTaskBuffer),
//		.priority = (osPriority_t) osPriorityRealtime,
//};
//
//const osThreadAttr_t MeasurementsTask_attributes = {
//		.name = "AnalogMeasTask",
//		.cb_mem = &MeasurementsTaskControlBlock,
//		.cb_size = sizeof(MeasurementsTaskControlBlock),
//		.stack_mem = &MeasurementsTaskBuffer[0],
//		.stack_size = sizeof(MeasurementsTaskBuffer),
//		.priority = (osPriority_t) osPriorityRealtime7,
//};

const osThreadAttr_t ControllerAndMeasurementsTask_attributes = {
		.name = "ControlMeasTask",
		.cb_mem = &ControllerAndMeasurementsTaskControlBlock,
		.cb_size = sizeof(ControllerAndMeasurementsTaskControlBlock),
		.stack_mem = &ControllerAndMeasurementsTaskBuffer[0],
		.stack_size = sizeof(ControllerAndMeasurementsTaskBuffer),
		.priority = (osPriority_t) osPriorityRealtime7,
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
//extern void ControllerTaskApp(void *argument);
//extern void MeasurementsTaskApp(void *argument);
extern void ControllerMeasTaskApp(void *argument);

eSjcReturnType_t InitOsThreads(void);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void SJC_TaskCommon_Init(void)
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
		result = (xTimeMeasurements[time_log_fcn].Idx > 0) ? ((float) xTimeMeasurements[time_log_fcn].Ts_ms)/xTimeMeasurements[time_log_fcn].Idx : 0.0f;
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
	return ( (task_name < TASK_HIGH_WTM_LIST_SIZE) ? xTaskHighWaterMarkMeasurements[task_name] : 0);
}

void SJC_TaskCommon_UpdateHighWaterMarkInfo(void)
{
	xTaskHighWaterMarkMeasurements[TASK_HIGH_WTM_INFO_CONTROLLER_MEAS_TASK] = uxTaskGetStackHighWaterMark(ControllerAndMeasurementsTaskHandle);
	xTaskHighWaterMarkMeasurements[TASK_HIGH_WTM_INFO_DATA_LOGGER_TASK] = uxTaskGetStackHighWaterMark(DataLoggerTaskHandle);
	xTaskHighWaterMarkMeasurements[TASK_HIGH_WTM_INFO_NETWORK_TASK] = uxTaskGetStackHighWaterMark(NetworkTaskHandle);
	xTaskHighWaterMarkMeasurements[TASK_HIGH_WTM_INFO_WATCHDOG_TASK] = uxTaskGetStackHighWaterMark(WatchdogTaskHandle);
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t InitOsThreads(void)
{
	// Shared queues creation

	xDataLoggerQueueHandle = xQueueCreateStatic( DATA_LOGGER_QUEUE_LENGTH,
	    							 DATA_LOGGER_ITEM_SIZE,
									 ucDataLoggerQueueStorageArea,
	                                 &xDataLoggerStaticQueue );

	configASSERT( xDataLoggerQueueHandle );

	// Tasks creation
	WatchdogTaskHandle = osThreadNew(WatchdogTaskApp, NULL, &WatchdogTask_attributes);
	DataLoggerTaskHandle = osThreadNew(DataLoggerTaskApp, NULL, &DataLoggerTask_attributes);
	NetworkTaskHandle = osThreadNew(NetworkTaskApp, NULL, &NetworkTask_attributes);
	//	ControllerTaskHandle = osThreadNew(ControllerTaskApp, NULL, &ControllerTask_attributes);
	//	MeasurementsTaskHandle = osThreadNew(MeasurementsTaskApp, NULL, &MeasurementsTask_attributes);
	ControllerAndMeasurementsTaskHandle = osThreadNew(ControllerMeasTaskApp, NULL, &ControllerAndMeasurementsTask_attributes);

	if (NULL == NetworkTaskHandle)
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Network task not initiated...");
		return SJC_RET_NOK;
	}

	if (NULL == DataLoggerTaskHandle)
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Data logger task not initiated...");
		return SJC_RET_NOK;
	}

	if (NULL == WatchdogTaskHandle)
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Watchdog task not initiated...");
		return SJC_RET_NOK;
	}

	if (NULL == ControllerAndMeasurementsTaskHandle)
	{
		DebugOutput_systemPrintf("\r\n[OS FAULT] Controller and measurements task not initiated...");
		return SJC_RET_NOK;
	}

	//	if (NULL == ControllerTaskHandle)
	//	{
	//		DebugOutput_SystemPrintf("\r\n[OS FAULT] Joint controller task not initiated...");
	//		return SJC_RET_NOK;
	//	}
	//
	//	if (NULL == MeasurementsTaskHandle)
	//	{
	//		DebugOutput_SystemPrintf("\r\n[OS FAULT] Analog measurements task not initiated...");
	//		return SJC_RET_NOK;
	//	}

	return SJC_RET_OK;
}
// ------------------------------------------------------------------- //


