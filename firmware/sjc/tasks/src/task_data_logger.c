/*
 * network_task.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */


// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include <actuator.h>
#include <task_data_logger.h>
#include "main.h"
#include "cmsis_os.h"
#include "usbd_cdc.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
// ------------------------------------------------------------------- //

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "sjc.h"
#include "configs.h"
#include "debug_output.h"
// ------------------------------------------------------------------- //


// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#define DATA_LOGGER_SHARED_DATA_MUTEX_WAIT_TIME_MS		( (uint32_t ) 1U)
#define DATA_LOGGER_TX_BUFF_SIZE						( (uint32_t ) 1024U)
// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //
static osMutexId_t DataMutexHandle;

osThreadId_t LoggerTaskHandle;
StaticTask_t LoggerTaskControlBlock;

uint32_t LoggerTaskBuffer[ 512 ];
const osThreadAttr_t LoggerTask_attributes = {
		.name = "LoggerTask",
		.cb_mem = &LoggerTaskControlBlock,
		.cb_size = sizeof(LoggerTaskControlBlock),
		.stack_mem = &LoggerTaskBuffer[0],
		.stack_size = sizeof(LoggerTaskBuffer),
		.priority = (osPriority_t) osPriorityBelowNormal,
};

char xTxBuff[DATA_LOGGER_TX_BUFF_SIZE];
int uTxBuffLen;
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
void LoggerTaskApp(void *argument);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t SJC_TaskDataLogger_SetSharedDataMutex(osMutexId_t MutexHandle)
{
	DataMutexHandle = MutexHandle;

	if (DataMutexHandle == NULL)
	{
		SJC_DebugOutput_printf("\r\n[OS] Data logger task: Shared data mutex not available...");
		return SJC_RET_NOK;
	} else
	{
		return SJC_RET_OK;
	}
}

eSjcReturnType_t SJC_TaskDataLogger_Init(void){
	LoggerTaskHandle = osThreadNew(LoggerTaskApp, NULL, &LoggerTask_attributes);

	if (LoggerTaskHandle == NULL)
	{
		return SJC_RET_NOK;
	}
	else
	{
		return SJC_RET_OK;
	}
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void LoggerTaskApp(void *argument)
{
	/* USER CODE BEGIN LoggerTaskApp */
	uint32_t tick;
	uint32_t last_tick;
	tick = osKernelGetTickCount();
	last_tick = tick;
	//	xSjcLogData_t xLogData;

	/* Infinite loop */
	for(;;)
	{
		tick += pdMS_TO_TICKS(SJC_DATA_LOGGER_UPDATE_PERIOD_MS);
		if (osOK == osMutexAcquire(DataMutexHandle, DATA_LOGGER_SHARED_DATA_MUTEX_WAIT_TIME_MS))
		{
			uint32_t TimeStamp;
			SJC_GetTime(SJC_TIME_MS, &TimeStamp);
//			uTxBuffLen = sprintf(xTxBuff, "\r\n[OS] Timestamp: %lu", TimeStamp);
			uTxBuffLen = sprintf(xTxBuff, "\r\n%lu", TimeStamp-last_tick);
			last_tick = TimeStamp;
			CDC_Transmit_FS(((uint8_t *)(&xTxBuff)), uTxBuffLen);
			//		xLogData.Timestamp = HAL_GetTick();
			//		xLogData.AuxEncoderA = SJC_DigitalInterfaces_GetEncoderCounter(SJC_AUX_ENCODER_A);
			//		xLogData.AuxEncoderB = SJC_DigitalInterfaces_GetEncoderCounter(SJC_AUX_ENCODER_B);
			//		xLogData.JointEncoder = SJC_DigitalInterfaces_GetEncoderCounter(SJC_JOINT_ENCODER);
			//		xLogData.MotorCurrent = SJC_Motor_GetElectricCurrent();
			//		xLogData.SystemVoltage = SJC_Motor_GetPowerSupplyVoltage();

			osMutexRelease(DataMutexHandle);

			osDelayUntil(tick);
		}

	}
	/* USER CODE END MainTaskApp */
}
// ------------------------------------------------------------------- //
