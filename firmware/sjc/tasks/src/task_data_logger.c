/*
 * task_data_logger.c
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  Data logging task: queue consumer for buffered serial transmission.
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "cmsis_os.h"
#include "main.h"
// ------------------------------------------------------------------- //

// NETWORK includes -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "base/include/configs.h"
#include "base/include/sjc.h"
#include "common/include/network_configs.h"
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
static uint32_t TaskTic, TaskToc, TaskLastTic;
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
static void InitLogTerminal(void);
static void TransmitLogData(const xSjcJointControllerMeasurements_t data);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void DataLoggerTaskApp(void *argument)
{
	/* USER CODE BEGIN WatchdogTaskApp */

	xSjcJointControllerMeasurements_t xQueueMsg = xSjcJointControllerMeasurementsEmptyObj;

	DebugOutput_systemPrintf("\r\n[OS] Data logger task initiated!");
	InitLogTerminal();
	SJC_getTime(SJC_TIME_MS, &TaskLastTic);

	/* Infinite loop */

	for (;;)
	{
		if (xQueueReceive(xDataLoggerQueueHandle, &xQueueMsg, portMAX_DELAY) == pdPASS)
		{
			SJC_getTime(SJC_TIME_MS, &TaskTic);
			SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_DATA_LOGGER_TASK_PERIOD_MS, (TaskTic - TaskLastTic));
			TaskLastTic = TaskTic;

			TransmitLogData(xQueueMsg);

			SJC_getTime(SJC_TIME_MS, &TaskToc);

			SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_DATA_LOGGER_TASK_EXEC_MS, (TaskToc - TaskTic));
		}
	}
	/* USER CODE END MainTaskApp */
}

static void InitLogTerminal(void)
{
	DataLogger_transmit("\n\n\n\r%%Timestamp_ms, ActuatorCurrent, ActuatorVoltage, SystemVoltage, ActuatorAngularPosition, ActuatorSpeedRpm, ControllerReference, ControllerError");
}

static void TransmitLogData(const xSjcJointControllerMeasurements_t Log)
{
	// DataLogger_transmit("\r\n%lu, %.3f, %.2f, %.2f, %.4f, %.3f, %.3f, %.4f",
	// 					Log.Timestamp_ms,
	// 					Log.ActuatorCurrent,
	// 					Log.ActuatorVoltage,
	// 					Log.SystemVoltage,
	// 					Log.ActuatorAngularPosition,
	// 					Log.ActuatorAngularRate,
	// 					Log.ActuatorSpeedRpm,
	// 					Log.ControllerReference,
	// 					Log.ControllerError);

	DataLogger_transmit(
		"{"
		"\"t\":%lu,"
		"\"i\":%.3f,"
		"\"v\":%.2f,"
		"\"vs\":%.2f,"
		"\"pos\":%.4f,"
		"\"vel\":%.4f,"
		"\"rpm\":%.3f,"
		"\"r\":%.3f,"
		"\"e\":%.4f,"
		"\"aux01\":%.4f,"
		"\"aux02\":%.4f"
		"}\r\n",
		Log.Timestamp_ms,
		Log.ActuatorCurrent,
		Log.ActuatorVoltage,
		Log.SystemVoltage,
		Log.ActuatorAngularPosition,
		Log.ActuatorAngularRate,
		Log.ActuatorSpeedRpm,
		Log.ControllerReference,
		Log.ControllerError,
		Log.AuxPositionMeas01,
		Log.AuxPositionMeas02);
}
//  ------------------------------------------------------------------- //
