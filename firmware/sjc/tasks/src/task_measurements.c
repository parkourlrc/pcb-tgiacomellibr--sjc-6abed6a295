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
#include "network.h"
#include "analog_interfaces.h"
#include "debug_output.h"
#include "timeout_timer.h"
#include "network_configs.h"

// ------------------------------------------------------------------- //


// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#define TASK_NOTIFICATION_MAX_WAIT_TIME_MS				( (uint32_t ) 2U)
// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //
TaskHandle_t xMeasurementsTaskHandle;
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void MeasurementsTaskApp(void *argument)
{
	/* USER CODE BEGIN MeasurementsTaskApp */

	osDelay(SJC_TASK_INIT_DELAY);

	xMeasurementsTaskHandle = xTaskGetCurrentTaskHandle();

	SJC_DebugOutput_printf("\r\n[OS] Analog measurements task initiated!");

	SJC_AnalogInterfaces_MeasurementEnable();

	uint32_t ulNotificationValue = 0;

	//	uint32_t tic, toc, dt;
	//	tic = osKernelGetTickCount();

	/* Infinite loop */
	for (;;) {

		ulNotificationValue = ulTaskNotifyTake( pdTRUE, TASK_NOTIFICATION_MAX_WAIT_TIME_MS );

		if( ulNotificationValue == 1 )
		{
			SJC_AnalogInterfaces_UpdateConversionValues();
			//toc = osKernelGetTickCount();
			//
			//dt = (toc > tic) ? (toc-tic) : (toc-tic+1000);
			//if (dt > 1)
			//{
			//	SJC_DebugOutput_printf("\r\n\nWait: %lu", dt);
			//}
			//tic = toc;
		}
		else
		{
			if (LL_TIM_IsEnabledCounter(SJC_PWM_INSTANCE))
			{
				SJC_DebugOutput_printf("\n\n\n\r[OS FAULT] ADC Task notification error");
			}
		}


	}
	/* USER CODE END MainTaskApp */
}

void ADC_IRQHandler(void)
{
	static uint32_t counts = 0;
	if(LL_ADC_IsActiveFlag_JEOS(SJC_ADC_INSTANCE) != 0)
	{
		/* Clear flag ADC group injected end of sequence conversions */
		LL_ADC_ClearFlag_JEOS(SJC_ADC_INSTANCE);

		/* Call interruption treatment function */
		SJC_AnalogInterfaces_AdcConversionIsr();

		/* Dispatch conversion and filtering */
		if (++counts == 20)
		{
			counts = 0;
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			configASSERT( xMeasurementsTaskHandle != NULL );
			vTaskNotifyGiveFromISR( xMeasurementsTaskHandle, &xHigherPriorityTaskWoken );
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}
	}

	/* Check whether ADC group regular overrun caused the ADC interruption */
	if(LL_ADC_IsActiveFlag_OVR(SJC_ADC_INSTANCE) != 0)
	{
		/* Clear flag ADC group regular overrun */
		LL_ADC_ClearFlag_OVR(SJC_ADC_INSTANCE);

		/* Call interruption treatment function */
		SJC_DebugOutput_printf("\r\n[HW FAULT] ADC Overrun!\n");
		Error_Handler();
	}
}

// ------------------------------------------------------------------- //
