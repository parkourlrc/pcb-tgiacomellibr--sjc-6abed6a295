/*
 * network_task.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "modules/include/modbus_network.h"
#include "main.h"
#include "cmsis_os.h"
#include "task.h"
// ------------------------------------------------------------------- //

// NETWORK includes -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "base/include/sjc.h"
#include "base/include/configs.h"
#include "base/include/network_configs.h"
#include "modules/include/analog_interfaces.h"
#include "modules/include/debug_output.h"
#include "tasks/include/task_common.h"
// ------------------------------------------------------------------- //

// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //
typedef enum {
	WD_STATE_STATISTICS = 0,
	WD_STATE_MEMORY,
	WD_STATE_RTOS_REPORT,
	WD_STATE_LIST_SIZE
} eWatchdogTaskState_t;
// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //
static uint32_t TaskTic, TaskToc;
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

	uint32_t tick = osKernelGetTickCount();

	xSjcMeanSampleTimeMeasurement_t Ts;

	// uint32_t counts = 0;

	/* Infinite loop */
	for (;;)
	{
		tick += pdMS_TO_TICKS(SJC_DEBUG_TASK_UPDATE_PERIOD_MS);

		SJC_GetTime(SJC_TIME_MS, &TaskTic);

		switch (xWatchdogState) {
		case WD_STATE_STATISTICS:
			AnalogInterfaces_getMeanConversionDelay(&Ts);
			MeanSampleTime_us = (Ts.AccumulatedTimerTicks > 0) ? (1.0f * Ts.AccumulatedTimerTicks) / (1.0f * Ts.Samples) : 0.0f;

			DebugOutput_systemPrintf("\n\r[OS] Runtime Stats.:"
					"\n\r - Uptime: %lu s"
					"\n\r - Analog. Conv. Delay: %.2f us"
					"\n\r - Controller Ts: %.3f ms"
					"\n\r - Controller Runtime: %.2f ms"
					"\n\r - Measurement Task Runtime: %.2f ms"
					"\n\r - Network Task: %.2f ms"
					"\n\r - Watchdog Task: %.2f ms"
					"\n\r - Data Logger Task: %.2f ms\n",
					TaskTic/1000,
					MeanSampleTime_us,
					SJC_TaskCommon_GetTimeMeasurement(TIME_LOG_CONTROLLER_SAMPLE_TIME),
					SJC_TaskCommon_GetTimeMeasurement(TIME_LOG_CONTROLLER_RUNTIME),
					SJC_TaskCommon_GetTimeMeasurement(TIME_LOG_MEASUREMENTS_RUNTIME),
					SJC_TaskCommon_GetTimeMeasurement(TIME_LOG_NETWORK_TASK_RUNTIME),
					SJC_TaskCommon_GetTimeMeasurement(TIME_LOG_WATCHDOG_TASK_RUNTIME),
					SJC_TaskCommon_GetTimeMeasurement(TIME_LOG_DATA_LOGGER_TASK_RUNTIME));
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

		SJC_GetTime(SJC_TIME_MS, &TaskToc);
		SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_WATCHDOG_TASK_RUNTIME, (TaskToc-TaskTic));

		osDelayUntil(tick);
	}
	/* USER CODE END MainTaskApp */
}

// void configureTimerForRunTimeStats(void)
//{
//	MX_TIM6_Init();
//
//	ulHighFrequencyTimerTicks = 0;
//
//	LL_TIM_SetCounter(TIM6, 0);
//	LL_TIM_EnableIT_UPDATE(TIM6);
//	LL_TIM_EnableCounter(TIM6);
// }

// unsigned long getRunTimeCounterValue(void)
//{
//	return osKernelGetTickCount();
// }

// void TIM6_DAC_IRQHandler(void)
//{
//	if (LL_TIM_IsActiveFlag_UPDATE(TIM6) & LL_TIM_IsEnabledIT_UPDATE(TIM6))
//	{
//		LL_TIM_ClearFlag_UPDATE(TIM6);
//		ulHighFrequencyTimerTicks++;
//	}
// }

// void TIM6_DAC_IRQHandler(void)
//{
//	if (LL_TIM_IsActiveFlag_UPDATE(TIM6) & LL_TIM_IsEnabledIT_UPDATE(TIM6))
//	{
//		LL_TIM_ClearFlag_UPDATE(TIM6);
//		ulHighFrequencyTimerTicks++;
//	}
// }
//  ------------------------------------------------------------------- //
