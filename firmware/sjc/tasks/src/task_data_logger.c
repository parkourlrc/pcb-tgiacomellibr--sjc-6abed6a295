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
#include "usbd_cdc.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
// ------------------------------------------------------------------- //

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "task_common.h"

#include "sjc.h"
#include "configs.h"
#include "motor_drive.h"
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


char xTxBuff[DATA_LOGGER_TX_BUFF_SIZE];
int uTxBuffLen;
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
void LoggerTaskApp(void *argument);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //

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
		if (osOK == SJC_TaskCommon_SharedDataMutexTake(DATA_LOGGER_SHARED_DATA_MUTEX_WAIT_TIME_MS))
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

			SJC_TaskCommon_SharedDataMutexGive();

			osDelayUntil(tick);
		}

	}
	/* USER CODE END MainTaskApp */
}
// ------------------------------------------------------------------- //
