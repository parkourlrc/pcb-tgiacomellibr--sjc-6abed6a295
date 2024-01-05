/*
 * common.c
 *
 *  Created on: Oct 4, 2023
 *      Author: Tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "task_common.h"
#include "task_data_logger.h"
#include "task_joint_controller.h"
#include "task_system_controller.h"
#include "cmsis_os.h"
// ------------------------------------------------------------------- //

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "debug_output.h"
// ------------------------------------------------------------------- //


// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //

// Common data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Common variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //
osMutexId_t SharedDataMutexHandle;
StaticSemaphore_t SharedDataMutexControlBlock;
const osMutexAttr_t SharedDataMutex_attributes = {
		.name = "SharedDataMutex",
		.cb_mem = &SharedDataMutexControlBlock,
		.cb_size = sizeof(SharedDataMutexControlBlock),
};
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t InitDataStructures(void);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t SJC_TaskCommon_InitThreads(void)
{
	if (SJC_RET_OK == InitDataStructures())
	{
		SJC_DebugOutput_printf("\r\n[OS] Data structures created!");
	}
	else
	{
		SJC_DebugOutput_printf("\r\n[OS] Data structures not created...");
		return SJC_RET_NOK;
	}

	if (SJC_RET_NOK == SJC_TaskActuatorController_SetSharedDataMutex(SharedDataMutexHandle))
	{
		return SJC_RET_NOK;
	}

	if (SJC_RET_NOK == SJC_TaskDataLogger_SetSharedDataMutex(SharedDataMutexHandle))
	{
		return SJC_RET_NOK;
	}

	if (SJC_RET_NOK == SJC_TaskSystemController_SetSharedDataMutex(SharedDataMutexHandle))
	{
		return SJC_RET_NOK;
	}

	if (SJC_RET_OK == SJC_TaskActuatorController_Init())
	{
		SJC_DebugOutput_printf("\r\n[OS] Controller task created!");
	}
	else
	{
		SJC_DebugOutput_printf("\r\n[OS] Controller task not initiated...");
		return SJC_RET_NOK;
	}

//	if (SJC_RET_OK == SJC_TaskDataLogger_Init())
//	{
//		SJC_DebugOutput_printf("\r\n[OS] Data logger task created!");
//	}
//	else
//	{
//		SJC_DebugOutput_printf("\r\n[OS] Data logger task not initiated...");
//		return SJC_RET_NOK;
//	}

	if (SJC_RET_OK == SJC_TaskSystemController_Init())
	{
		SJC_DebugOutput_printf("\r\n[OS] System manager task created!");
	}
	else
	{
		SJC_DebugOutput_printf("\r\n[OS] System manager task not initiated...");
		return SJC_RET_NOK;
	}

	return SJC_RET_OK;
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t InitDataStructures(void)
{
	SharedDataMutexHandle = osMutexNew(&SharedDataMutex_attributes);

	if (SharedDataMutexHandle == NULL)
	{
		SJC_DebugOutput_printf("\r\n[OS] Shared data mutex not created...");
		return SJC_RET_NOK;
	} else
	{
		osMutexRelease(SharedDataMutexHandle);
		SJC_DebugOutput_printf("\r\n[OS] Shared data mutex created!");
	}

	return SJC_RET_OK;
}
// ------------------------------------------------------------------- //


