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
#define MEASUREMENTS_SHARED_DATA_MUTEX_WAIT_TIME_MS		( (uint32_t ) 2U)
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

	uint32_t tick = osKernelGetTickCount();
	uint32_t ulNotificationValue = 0;

	SJC_AnalogInterfaces_MeasurementEnable();

	/* Infinite loop */
	for (;;) {
		tick += pdMS_TO_TICKS(SJC_NETWORK_TASK_UPDATE_PERIOD_MS);

		ulNotificationValue = ulTaskNotifyTake( pdTRUE, 2 );

		if( ulNotificationValue == 1 )
		{
			SJC_AnalogInterfaces_UpdateConversionValues();
		}
		else
		{
			// The call to ulTaskNotifyTake() timed out.
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
		SJC_DebugOutput_printf("\r\n[FAULT] ADC Overrun!\n");
		Error_Handler();
	}
}

// ------------------------------------------------------------------- //
