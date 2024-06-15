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
// ------------------------------------------------------------------- //

// NETWORK includes -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "sjc.h"
#include "configs.h"
#include "data_logger.h"
#include "debug_output.h"
#include "task_common.h"
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
void TransmitLogData(const xSjcJointControllerMeasurements_t data);
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void DataLoggerTaskApp(void *argument)
{
	/* USER CODE BEGIN WatchdogTaskApp */

	xSjcJointControllerMeasurements_t xQueueMsg = xSjcJointControllerMeasurementsEmptyObj;

	DebugOutput_systemPrintf("\r\n[OS] Data logger task initiated!");


	/* Infinite loop */
	for (;;)
	{
		if (xQueueReceive(xDataLoggerQueueHandle, &xQueueMsg, pdMS_TO_TICKS(portMAX_DELAY)) == pdPASS)
		{
			SJC_getTime(SJC_TIME_MS, &TaskTic);

			TransmitLogData(xQueueMsg);

			SJC_getTime(SJC_TIME_MS, &TaskToc);

			SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_WATCHDOG_TASK_RUNTIME, (TaskToc-TaskTic));				}
	}
	/* USER CODE END MainTaskApp */
}

void TransmitLogData(const xSjcJointControllerMeasurements_t Log)
{
	DataLogger_transmitData("\r\n%lu, %.2f, %.2f, %.2f, %.4f, %.3f, %.3f, %.3f",
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
