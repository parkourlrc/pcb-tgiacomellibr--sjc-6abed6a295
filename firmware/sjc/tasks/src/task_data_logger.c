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
#include "modules/include/data_logger.h"
#include "modules/include/debug_output.h"
#include "tasks/include/task_common.h"
// ------------------------------------------------------------------- //

// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //
extern QueueHandle_t xDataLoggerQueueHandle;
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
void InitLogTerminal(void);
void TransmitLogData(const xSjcJointControllerMeasurements_t data);
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void DataLoggerTaskApp(void *argument)
{
	/* USER CODE BEGIN WatchdogTaskApp */

	xSjcJointControllerMeasurements_t xQueueMsg = xSjcJointControllerMeasurementsEmptyObj;

	DebugOutput_systemPrintf("\r\n[OS] Data logger task initiated!");
	InitLogTerminal();

	/* Infinite loop */
	for (;;)
	{
		if (xQueueReceive(xDataLoggerQueueHandle, &xQueueMsg, pdMS_TO_TICKS(portMAX_DELAY)) == pdPASS)
		{
			SJC_getTime(SJC_TIME_MS, &TaskTic);

			TransmitLogData(xQueueMsg);

			SJC_getTime(SJC_TIME_MS, &TaskToc);

			SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_DATA_LOGGER_TASK_RUNTIME, (TaskToc - TaskTic));
		}
	}
	/* USER CODE END MainTaskApp */
}

void InitLogTerminal(void)
{
	DataLogger_transmit("\n\n\n\r%%Timestamp, ActuatorCurrent, ActuatorVoltage, SystemVoltage, ActuatorDisplacement, ActuatorSpeedRpm, ControllerReference, ControllerError");
}

void TransmitLogData(const xSjcJointControllerMeasurements_t Log)
{
	// DataLogger_transmit("\r\n%lu, %.3f, %.2f, %.2f, %.4f, %.3f, %.3f, %.4f",
	// 					Log.Timestamp,
	// 					Log.ActuatorCurrent,
	// 					Log.ActuatorVoltage,
	// 					Log.SystemVoltage,
	// 					Log.ActuatorDisplacement,
	// 					Log.ActuatorSpeedRpm,
	// 					Log.ControllerReference,
	// 					Log.ControllerError);

	DataLogger_transmit(
	    "{"
	    "\"t\":%lu,"
	    "\"i\":%.3f,"
	    "\"v\":%.2f,"
	    "\"vs\":%.2f,"
	    "\"dp\":%.4f,"
	    "\"rpm\":%.3f,"
	    "\"r\":%.3f,"
	    "\"e\":%.4f"
	    "}\r\n",
	    Log.Timestamp,
	    Log.ActuatorCurrent,
	    Log.ActuatorVoltage,
	    Log.SystemVoltage,
	    Log.ActuatorDisplacement,
	    Log.ActuatorSpeedRpm,
	    Log.ControllerReference,
	    Log.ControllerError);
}
//  ------------------------------------------------------------------- //
