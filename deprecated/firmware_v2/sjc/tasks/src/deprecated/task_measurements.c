///*
// * network_task.c
// *
// *  Created on: Sep 9, 2023
// *      Author: tiago
// */
//
//
//// HAL includes ------------------------------------------------------ //
//// ------------------------------------------------------------------- //
//#include "main.h"
//#include "cmsis_os.h"
//// ------------------------------------------------------------------- //
//
//// NETWORK includes -------------------------------------------------- //
//// ------------------------------------------------------------------- //
//
//// ------------------------------------------------------------------- //
//
//// Task includes ----------------------------------------------------- //
//// ------------------------------------------------------------------- //
//#include "sjc.h"
//#include "configs.h"
//#include "network.h"
//#include "analog_interfaces.h"
//#include "debug_output.h"
//#include "network_configs.h"
//#include "task_common.h"
//// ------------------------------------------------------------------- //
//
//
//// Task defines ------------------------------------------------------ //
//// ------------------------------------------------------------------- //
//#define TASK_NOTIFICATION_MAX_WAIT_TIME_MS				( (uint32_t ) 2U)
//// ------------------------------------------------------------------- //
//
//// Task data structures ---------------------------------------------- //
//// ------------------------------------------------------------------- //
//typedef enum {
//	TASK_MEASUREMENT_STATE_UNINITIALIZED = 0,
//	TASK_MEASUREMENT_STATE_RUNNING
//} eSjcTaskMeasurementState_t;
//// ------------------------------------------------------------------- //
//
//// Task variables ---------------------------------------------------- //
//// ------------------------------------------------------------------- //
//TaskHandle_t xMeasurementsTaskHandle;
//static eSjcTaskMeasurementState_t xTaskState = TASK_MEASUREMENT_STATE_UNINITIALIZED;
//
//static uint32_t TaskTic, TaskToc;
//// ------------------------------------------------------------------- //
//
//// Private function prototypes --------------------------------------- //
//// ------------------------------------------------------------------- //
//
//// ------------------------------------------------------------------- //
//
//// Public functions -------------------------------------------------- //
//// ------------------------------------------------------------------- //
//
//// ------------------------------------------------------------------- //
//
//// Private functions ------------------------------------------------- //
//// ------------------------------------------------------------------- //
//void MeasurementsTaskApp(void *argument)
//{
//	/* USER CODE BEGIN MeasurementsTaskApp */
//
//	osDelay(SJC_TASK_INIT_DELAY);
//
//	xMeasurementsTaskHandle = xTaskGetCurrentTaskHandle();
//
//	SJC_DebugOutput_SystemPrintf("\r\n[OS] Analog measurements task initiated!");
//
//	AnalogInterfaces_measurementEnable();
//
//	uint32_t ulNotificationValue = 0;
//
//	xTaskState = TASK_MEASUREMENT_STATE_RUNNING;
//
////	uint32_t tick = osKernelGetTickCount();
//
//	/* Infinite loop */
//	for (;;) {
//
//		SJC_getTime(SJC_TIME_MS, &TaskTic);
//
//		ulNotificationValue = ulTaskNotifyTake( pdTRUE, TASK_NOTIFICATION_MAX_WAIT_TIME_MS );
//
//		if( ulNotificationValue == 1 )
//		{
//			AnalogInterfaces_updateConversionValues();
//		}
//		else
//		{
//			if (LL_TIM_IsEnabledCounter(SJC_PWM_INSTANCE))
//			{
//				SJC_DebugOutput_SystemPrintf("\n\n\n\r[OS FAULT] ADC Task notification error");
//			}
//		}
//
//		SJC_getTime(SJC_TIME_MS, &TaskToc);
//
//		SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_MEASUREMENTS_RUNTIME, (TaskToc-TaskTic));
//	}
//
//	AnalogInterfaces_measurementDisable();
//	/* USER CODE END MainTaskApp */
//}
//
//void ADC_IRQHandler(void)
//{
//	if(LL_ADC_IsActiveFlag_JEOS(SJC_ADC_INSTANCE) != 0)
//	{
//		/* Clear flag ADC group injected end of sequence conversions */
//		LL_ADC_ClearFlag_JEOS(SJC_ADC_INSTANCE);
//
//		switch (xTaskState) {
//		case TASK_MEASUREMENT_STATE_RUNNING:
//			/* Call interruption treatment function */
//			SJC_AnalogInterfaces_AdcConversionIsr();
//
//			/* Dispatch conversion and filtering */
//			if (SJC_RET_OK == AnalogInterfaces_checkUpdateConversionTime())
//			{
//				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//				configASSERT( xMeasurementsTaskHandle != NULL );
//				vTaskNotifyGiveFromISR( xMeasurementsTaskHandle, &xHigherPriorityTaskWoken );
//				portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//			}
//			break;
//		default:
//			break;
//		}
//	}
//
//	/* Check whether ADC group regular overrun caused the ADC interruption */
//	if(LL_ADC_IsActiveFlag_OVR(SJC_ADC_INSTANCE) != 0)
//	{
//		/* Clear flag ADC group regular overrun */
//		LL_ADC_ClearFlag_OVR(SJC_ADC_INSTANCE);
//
//		/* Call interruption treatment function */
//		SJC_DebugOutput_SystemPrintf("\r\n[HW FAULT] ADC Overrun!\n");
//		Error_Handler();
//	}
//}
//
//// ------------------------------------------------------------------- //
