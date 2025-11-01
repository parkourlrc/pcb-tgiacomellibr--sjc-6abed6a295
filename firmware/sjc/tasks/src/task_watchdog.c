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
#include "task.h"
// ------------------------------------------------------------------- //

// NETWORK includes -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "base/include/configs.h"
#include "base/include/sjc.h"
#include "common/include/network_configs.h"
#include "modules/include/analog_interfaces.h"
#include "modules/include/debug_output.h"
#include "tasks/include/task_common.h"
// ------------------------------------------------------------------- //

// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //
typedef enum
{
	WD_STATE_STATISTICS = 0,
	WD_STATE_MEMORY,
	WD_STATE_RTOS_REPORT,
	WD_STATE_LIST_SIZE
} eWatchdogTaskState_t;
// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //
static uint32_t TaskTic, TaskToc, watchdog_last_wake_tick;
static float MeanSampleTime_us = 0.0f;
eWatchdogTaskState_t xWatchdogState = WD_STATE_STATISTICS;
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void WatchdogTaskApp(void *argument)
{
	/* USER CODE BEGIN WatchdogTaskApp */

	osDelay(SJC_TASK_INIT_DELAY);
	DebugOutput_systemPrintf("\r\n[OS] Watchdog task initiated!");

	TickType_t lastWakeTime = xTaskGetTickCount();

	xSjcMeanSampleTimeMeasurement_t Ts;

	uint32_t uptime;
	float period_logs[TIME_LOG_FUNCTIONS_NAME_SIZE / 2];
	float exec_logs[TIME_LOG_FUNCTIONS_NAME_SIZE / 2];

	// uint32_t counts = 0;

	/* Infinite loop */
	for (;;)
	{
		SJC_getTime(SJC_TIME_MS, &TaskTic);
		SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_WATCHDOG_TASK_PERIOD_MS, (TaskTic - watchdog_last_wake_tick));
		watchdog_last_wake_tick = TaskTic;

		switch (xWatchdogState)
		{
		case WD_STATE_STATISTICS:
			AnalogInterfaces_getMeanConversionDelay(&Ts);

			uptime = TaskTic / 1000;
			period_logs[TIME_LOG_CONTROLLER_TASK_PERIOD_MS] = SJC_TaskCommon_GetTimeMeasurement(TIME_LOG_CONTROLLER_TASK_PERIOD_MS);
			period_logs[TIME_LOG_DATA_LOGGER_TASK_PERIOD_MS] = SJC_TaskCommon_GetTimeMeasurement(TIME_LOG_DATA_LOGGER_TASK_PERIOD_MS);
			period_logs[TIME_LOG_MEASUREMENTS_TASK_PERIOD_MS] = SJC_TaskCommon_GetTimeMeasurement(TIME_LOG_MEASUREMENTS_TASK_PERIOD_MS);
			period_logs[TIME_LOG_NETWORK_TASK_PERIOD_MS] = SJC_TaskCommon_GetTimeMeasurement(TIME_LOG_NETWORK_TASK_PERIOD_MS);
			period_logs[TIME_LOG_WATCHDOG_TASK_PERIOD_MS] = SJC_TaskCommon_GetTimeMeasurement(TIME_LOG_WATCHDOG_TASK_PERIOD_MS);

			exec_logs[TIME_LOG_CONTROLLER_TASK_EXEC_MS] = SJC_TaskCommon_GetTimeMeasurement(TIME_LOG_CONTROLLER_TASK_EXEC_MS);
			exec_logs[TIME_LOG_DATA_LOGGER_TASK_EXEC_MS] = SJC_TaskCommon_GetTimeMeasurement(TIME_LOG_DATA_LOGGER_TASK_EXEC_MS);
			exec_logs[TIME_LOG_MEASUREMENTS_TASK_EXEC_MS] = SJC_TaskCommon_GetTimeMeasurement(TIME_LOG_MEASUREMENTS_TASK_EXEC_MS);
			exec_logs[TIME_LOG_NETWORK_TASK_EXEC_MS] = SJC_TaskCommon_GetTimeMeasurement(TIME_LOG_NETWORK_TASK_EXEC_MS);
			exec_logs[TIME_LOG_WATCHDOG_TASK_EXEC_MS] = SJC_TaskCommon_GetTimeMeasurement(TIME_LOG_WATCHDOG_TASK_EXEC_MS);

			DebugOutput_systemPrintf("\n\r[OS] Runtime Stats.:"
									 "\n\r - Uptime: %lu s"
									 "\n\r - Analog. Conv. Delay: %.2f us"
									 "\n\r - Controller period: %.2f ms - Exec.: %.2f ms"
									 "\n\r - Data Logger period: %.2f ms - Exec.: %.2f ms"
									 "\n\r - Measurement period: %.2f ms - Exec.: %.2f ms"
									 "\n\r - Network period: %.2f ms - Exec.: %.2f ms"
									 "\n\r - Watchdog period: %.2f ms - Exec.: %.2f ms",
									 uptime,
									 (Ts.AccumulatedTimerTicks > 0) ? (1.0f * Ts.AccumulatedTimerTicks) / (1.0f * Ts.Samples) : 0.0f,
									 period_logs[TIME_LOG_CONTROLLER_TASK_PERIOD_MS],
									 exec_logs[TIME_LOG_CONTROLLER_TASK_EXEC_MS],
									 period_logs[TIME_LOG_DATA_LOGGER_TASK_PERIOD_MS],
									 exec_logs[TIME_LOG_DATA_LOGGER_TASK_EXEC_MS],
									 period_logs[TIME_LOG_MEASUREMENTS_TASK_PERIOD_MS],
									 exec_logs[TIME_LOG_MEASUREMENTS_TASK_EXEC_MS],
									 period_logs[TIME_LOG_NETWORK_TASK_PERIOD_MS],
									 exec_logs[TIME_LOG_NETWORK_TASK_EXEC_MS],
									 period_logs[TIME_LOG_WATCHDOG_TASK_PERIOD_MS],
									 exec_logs[TIME_LOG_WATCHDOG_TASK_EXEC_MS]);

			xWatchdogState = WD_STATE_MEMORY;
			break;
		case WD_STATE_MEMORY:
			SJC_TaskCommon_UpdateHighWaterMarkInfo();
			DebugOutput_systemPrintf("\n\r[OS] HighWaterMark:"
									 "\n\r - Watchdog Task: %lu B"
									 "\n\r - Data Logger Task: %.lu B"
									 "\n\r - Network Task: %lu B"
									 "\n\r - ControlMeas Task: %lu B\n",
									 SJC_TaskCommon_GetHighWaterMarkInfo(TASK_HIGH_WTM_INFO_WATCHDOG_TASK),
									 SJC_TaskCommon_GetHighWaterMarkInfo(TASK_HIGH_WTM_INFO_DATA_LOGGER_TASK),
									 SJC_TaskCommon_GetHighWaterMarkInfo(TASK_HIGH_WTM_INFO_NETWORK_TASK),
									 SJC_TaskCommon_GetHighWaterMarkInfo(TASK_HIGH_WTM_INFO_CONTROLLER_MEAS_TASK));
			xWatchdogState = WD_STATE_RTOS_REPORT;
			break;

		default: // WD_STATE_RTOS_REPORT
			DebugOutput_presentRuntimeStatistics();
			xWatchdogState = WD_STATE_STATISTICS;
			break;
		}

		SJC_getTime(SJC_TIME_MS, &TaskToc);
		SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_WATCHDOG_TASK_EXEC_MS, (TaskToc - TaskTic));

		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SJC_DEBUG_TASK_UPDATE_PERIOD_MS));
	}
	/* USER CODE END MainTaskApp */
}

//  ------------------------------------------------------------------- //
