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

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "task_joint_controller.h"

#include "sjc.h"
#include "configs.h"
#include "debug_output.h"
#include "analog_sensors.h"
// ------------------------------------------------------------------- //


// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#define CONTROLLER_SHARED_DATA_MUTEX_WAIT_TIME_MS		( osWaitForever )
#define CONTROLLER_MAX_BLOCK_TIME_TICKS					pdMS_TO_TICKS( 1 )

#define ANALOG_SENSORS_CONV_BUFFER_SIZE					( (uint32_t ) 32UL)
#define ANALOG_CONV_MEASUREMENTS_TARGET					( (uint32_t ) 4UL)
#define ANALOG_CONVS_REACHED_EVENT_NOTIFICATION    		0x01

// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //
extern ADC_HandleTypeDef hadc1;

// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //
static osMutexId_t DataMutexHandle;

osThreadId_t ControllerTaskHandle;
StaticTask_t ControllerTaskControlBlock;

uint32_t ControllerTaskBuffer[ 256 ];
const osThreadAttr_t ControllerTask_attributes = {
		.name = "ControllerTask",
		.cb_mem = &ControllerTaskControlBlock,
		.cb_size = sizeof(ControllerTaskControlBlock),
		.stack_mem = &ControllerTaskBuffer[0],
		.stack_size = sizeof(ControllerTaskBuffer),
		.priority = (osPriority_t) osPriorityRealtime,
};

static TaskHandle_t xHandlingTask;

static uint32_t xAnalogConversionCounter;
static uint32_t xAnalogSensorsConversionBuffer[ANALOG_SENSORS_CONV_BUFFER_SIZE];

static float xAnalogSensorsValueBuffer[SJC_ANALOG_SENSOR_SIZE];
static xSjcJoint_t Joint;
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
void ControllerTaskApp(void *argument);
void MeasurementsUpdate(void)
{

}

void ControllerFunc(xSjcJoint_t Joint);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t SJC_TaskJointController_SetSharedDataMutex(osMutexId_t MutexHandle)
{
	DataMutexHandle = MutexHandle;

	if (DataMutexHandle == NULL)
	{
		SJC_DebugOutput_printf("\r\n[OS] Controller task: Shared data mutex not available...");
		return SJC_RET_NOK;
	} else
	{
		return SJC_RET_OK;
	}
}

eSjcReturnType_t SJC_TaskJointController_Init(void){
	ControllerTaskHandle = osThreadNew(ControllerTaskApp, NULL, &ControllerTask_attributes);

	if (ControllerTaskHandle == NULL)
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
void ControllerTaskApp(void *argument)
{
	/* USER CODE BEGIN ControllerTaskApp */
	xAnalogConversionCounter = 0;

	SJC_InitAdc();
	SJC_EnableAdc(xAnalogSensorsConversionBuffer);
	uint32_t tick;
	tick = osKernelGetTickCount();

	uint32_t ulNotifiedValue;
	BaseType_t xResult;

	/* Infinite loop */
	for(;;)
	{
		uint32_t Period = SJC_GetControlUpdatePeriodMs();
		tick += pdMS_TO_TICKS(Period);

		xResult = xTaskNotifyWait( 	pdFALSE, 0xffffffff, &ulNotifiedValue, CONTROLLER_MAX_BLOCK_TIME_TICKS*5 );

		if ( pdPASS == xResult )
		{
			if( ( ulNotifiedValue & ANALOG_CONVS_REACHED_EVENT_NOTIFICATION ) != 0 )
			{
				MeasurementsUpdate();

				SJC_GetJointStates(SJC_ACTUATOR_JOINT, &Joint);
				ControllerFunc(Joint);
			}

		}
	}
	/* USER CODE END MainTaskApp */
}

void ControllerFunc(xSjcJoint_t Joint)
{
	switch (Joint.Controller.Mode) {
				//			case SJC_SPEED_CONTROL:
				//
				//				break;
				//
				//			case SJC_TORQUE_CONTROL:
				//
				//				break;
				//
				//			case SJC_PRBS_EXCITATION:
				//
				//				break;

				case SJC_OPEN_LOOP_CONTROL:
					SJC_EnableJointActuators();
					SJC_SetJointActuatorVoltage(Joint.Actuator.OutputVoltage);

					break;

				default: // SJC_CONTROL_DISABLED
					SJC_DisableJointActuators();
					break;
				}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint32_t CurrentConv = xAnalogSensorsConversionBuffer[0];
	uint32_t VoltageConv = xAnalogSensorsConversionBuffer[1];

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	float TempActual;
	float TempPrev;

	TempPrev = xAnalogSensorsValueBuffer[SJC_CURRENT_SENSOR_STD];
	SJC_AnalogSensors_ComputeConversionValue(SJC_CURRENT_SENSOR_STD, CurrentConv, &TempActual);
	SJC_AnalogSensors_ComputeFilteredValue(SJC_CURRENT_SENSOR_STD, TempPrev, TempActual, &xAnalogSensorsValueBuffer[SJC_CURRENT_SENSOR_STD]);

	TempPrev = xAnalogSensorsValueBuffer[SJC_VOLTAGE_SENSOR];
	SJC_AnalogSensors_ComputeConversionValue(SJC_VOLTAGE_SENSOR, VoltageConv, &TempActual);
	SJC_AnalogSensors_ComputeFilteredValue(SJC_VOLTAGE_SENSOR, TempPrev, TempActual, &xAnalogSensorsValueBuffer[SJC_CURRENT_SENSOR_STD]);

	xAnalogConversionCounter++;

	if (ANALOG_CONV_MEASUREMENTS_TARGET == xAnalogConversionCounter)
	{
		xAnalogConversionCounter = 0;
		xTaskNotifyFromISR(xHandlingTask, ANALOG_CONVS_REACHED_EVENT_NOTIFICATION, eSetBits, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

	}

}

void TIM4_IRQHandler(void)
{
	/* USER CODE BEGIN TIM4_IRQn 0 */
	if(LL_TIM_IsActiveFlag_UPDATE(TIM4) && LL_TIM_IsEnabledIT_UPDATE(TIM4))
	{
		LL_TIM_ClearFlag_UPDATE(TIM4);


//		SJC_DebugOutput_printf("\r\n%.2f", Iin);
	}
	/* USER CODE END TIM4_IRQn 0 */
	/* USER CODE BEGIN TIM4_IRQn 1 */

	/* USER CODE END TIM4_IRQn 1 */
}
// ------------------------------------------------------------------- //
