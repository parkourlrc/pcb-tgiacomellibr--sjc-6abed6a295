/*
 * common.h
 *
 *  Created on: Oct 4, 2023
 *      Author: Tiago
 */

#ifndef SJC__TASKS__COMMON_H_
#define SJC__TASKS__COMMON_H_

#include "types.h"

typedef enum {
	TIME_LOG_CONTROLLER_RUNTIME = 0,
	TIME_LOG_CONTROLLER_SAMPLE_TIME,
	TIME_LOG_MEASUREMENTS_RUNTIME,
	TIME_LOG_NETWORK_TASK_RUNTIME,
	TIME_LOG_WATCHDOG_TASK_RUNTIME,
	TIME_LOG_DATA_LOGGER_TASK_RUNTIME,
	TIME_LOG_FUNCTIONS_NAME_SIZE,
} eTimeLogFunctionList_t;

typedef enum {
	TASK_HIGH_WTM_INFO_CONTROLLER_MEAS_TASK = 0,
	TASK_HIGH_WTM_INFO_DATA_LOGGER_TASK,
	TASK_HIGH_WTM_INFO_NETWORK_TASK,
	TASK_HIGH_WTM_INFO_WATCHDOG_TASK,
	TASK_HIGH_WTM_LIST_SIZE
} eTaskHighWaterMarkInfoList_t;

typedef struct {
	uint32_t Ts_ms;
	uint32_t Idx;
} TaskTimeMeasurement_t;
static const TaskTimeMeasurement_t TaskTimeMeasurementEmptyObj = {0};

void SJC_TaskCommon_initThreads(void);
void SJC_TaskCommon_initCommonObjects(void);

float SJC_TaskCommon_GetTimeMeasurement(const eTimeLogFunctionList_t time_log_fcn);
void SJC_TaskCommon_SetTimeMeasurement(const eTimeLogFunctionList_t time_log_fcn, const uint32_t time);

void SJC_TaskCommon_UpdateHighWaterMarkInfo(void);
uint32_t SJC_TaskCommon_GetHighWaterMarkInfo(const eTaskHighWaterMarkInfoList_t task_name);

#endif /* SJC__TASKS__COMMON_H_ */
