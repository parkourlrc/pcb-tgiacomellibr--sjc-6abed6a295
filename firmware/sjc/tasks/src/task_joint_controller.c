/*
 * network_task.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */


// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "main.h"
#include "adc.h"
#include "cmsis_os.h"
// ------------------------------------------------------------------- //

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "task_joint_controller.h"

#include "sjc.h"
#include "configs.h"
#include "debug_output.h"
#include "analog_interfaces.h"
// ------------------------------------------------------------------- //


// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#define CONTROLLER_SHARED_DATA_MUTEX_WAIT_TIME_MS		( osWaitForever )
#define CONTROLLER_MAX_BLOCK_TIME_TICKS					pdMS_TO_TICKS( 1 )

#define ANALOG_CONV_MEASUREMENTS_TARGET					( (uint32_t ) 20UL)
#define ANALOG_CONVS_REACHED_EVENT_NOTIFICATION    		0x01

#define SYSTEM_MANAGER_SHARED_DATA_MUTEX_WAIT_TIME_MS	( (uint32_t ) 1U)

// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //
//extern ADC_HandleTypeDef hadc1;

// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //
static osMutexId_t DataMutexHandle;

osThreadId_t ControllerTaskHandle;
StaticTask_t ControllerTaskControlBlock;

uint32_t ControllerTaskBuffer[ 4096 ];
const osThreadAttr_t ControllerTask_attributes = {
		.name = "ControllerTask",
		.cb_mem = &ControllerTaskControlBlock,
		.cb_size = sizeof(ControllerTaskControlBlock),
		.stack_mem = &ControllerTaskBuffer[0],
		.stack_size = sizeof(ControllerTaskBuffer),
		.priority = (osPriority_t) osPriorityRealtime,
};

static xSjcJoint_t Joint;
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
void ControllerFunc(xSjcJoint_t Joint);
void ControllerTaskApp(void *argument);

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
eSjcReturnType_t SJC_TaskActuatorController_SetSharedDataMutex(osMutexId_t MutexHandle)
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

eSjcReturnType_t SJC_TaskActuatorController_Init(void){
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
	SJC_AnalogInterfaces_Enable();
	//	SJC_InitAdc(xAnalogSensorsConversionBuffer, ANALOG_SENSORS_CONV_BUFFER_SIZE);
	//	SJC_EnableAdc();

	uint32_t tick;
	tick = osKernelGetTickCount();

	//	uint32_t ulNotifiedValue;
	//	BaseType_t xResult;

	/* Infinite loop */
	for(;;)
	{
		uint32_t Period;
		SJC_GetControlUpdatePeriodMs(&Period);
		tick += pdMS_TO_TICKS(Period);

		//		xResult = xTaskNotifyWait( 	pdFALSE, 0xffffffff, &ulNotifiedValue, CONTROLLER_MAX_BLOCK_TIME_TICKS*5 );
		//
		//		if ( pdPASS == xResult )
		//		{
		//			if( ( ulNotifiedValue & ANALOG_CONVS_REACHED_EVENT_NOTIFICATION ) != 0 )
		//			{
		//				MeasurementsUpdate();
		//
		//				SJC_GetJointActuatorState(SJC_ACTUATOR_JOINT, &Joint);
		//				ControllerFunc(Joint);
		//			}
		//
		//		}
		if (osOK == osMutexAcquire(DataMutexHandle, SYSTEM_MANAGER_SHARED_DATA_MUTEX_WAIT_TIME_MS))
		{
			SJC_GetJointActuatorHandle(&Joint);
			ControllerFunc(Joint);
			osMutexRelease(DataMutexHandle);
			osDelayUntil(tick);
		}

	}
}



//void AdcGrpInjectedUnitaryConvComplete_Callback(void)
//{
//	uint32_t VrefConv = xAnalogSensorsConversionBuffer[0];
//	uint32_t CurrentConv = xAnalogSensorsConversionBuffer[1];
//	uint32_t VoltageConv = xAnalogSensorsConversionBuffer[2];
//	//	HAL_ADC_Stop(&hadc1);
//
//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//
//	static float TempActual[3] = {0, 0, 0};
//	static float TempPrev[3] = {0, 0, 0};
//
//
//
//	//	LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_1);
//	//	LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_2);
//	//	LL_ADC_INJ_ReadConversionData12(SJC_ADC_INSTANCE, LL_ADC_INJ_RANK_3);
//
//	TempPrev[0] = TempActual[0];
//	TempActual[0] = __LL_ADC_CALC_VREFANALOG_VOLTAGE(VrefConv, LL_ADC_RESOLUTION_12B);
//
//	TempPrev[1] = xAnalogSensorsValueBuffer[SJC_CURRENT_SENSOR_STD];
//	SJC_AnalogSensors_ComputeConversionValue(SJC_CURRENT_SENSOR_STD, CurrentConv, &TempActual[1]);
//	SJC_AnalogSensors_ComputeFilteredValue(SJC_CURRENT_SENSOR_STD, TempPrev[1], TempActual[1], &xAnalogSensorsValueBuffer[SJC_CURRENT_SENSOR_STD]);
//
//	TempPrev[2] = xAnalogSensorsValueBuffer[SJC_VOLTAGE_SENSOR];
//	SJC_AnalogSensors_ComputeConversionValue(SJC_VOLTAGE_SENSOR, VoltageConv, &TempActual[2]);
//	SJC_AnalogSensors_ComputeFilteredValue(SJC_VOLTAGE_SENSOR, TempPrev[2], TempActual[2], &xAnalogSensorsValueBuffer[SJC_CURRENT_SENSOR_STD]);
//
//	xAnalogConversionCounter++;
//
//	if (ANALOG_CONV_MEASUREMENTS_TARGET == xAnalogConversionCounter)
//	{
//		xAnalogConversionCounter = 0;
//		xTaskNotifyFromISR(xHandlingTask, ANALOG_CONVS_REACHED_EVENT_NOTIFICATION, eSetBits, &xHigherPriorityTaskWoken);
//		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//
//	}
//}

void ControllerFunc(xSjcJoint_t Joint)
{
	eSjcControlMode_t ControlMode;
	float Ref;
	SJC_GetControlMode(&ControlMode);
	SJC_GetControlReference(&Ref);


	switch (ControlMode) {
	//			case SJC_SPEED_CONTROL:
	//
	//				break;
	//
	//	case SJC_TORQUE_CONTROL:
	//		SJC_EnableJointActuators();
	//
	//		break;
	//
	//			case SJC_PRBS_EXCITATION:
	//
	//				break;

	case SJC_OPEN_LOOP_CONTROL:
		SJC_EnableJointActuators();
		SJC_SetJointActuatorVoltage(Ref);
		float aux1, aux2, aux3;
		SJC_GetJointActuatorPowerSupplyVoltage(&aux1);
		SJC_GetJointActuatorVoltage(&aux2);
		SJC_GetJointActuatorCurrent(&aux3);
		uint32_t stmp;
		SJC_GetTime(SJC_TIME_MS, &stmp);
//		SJC_DebugOutput_UsbPrintf("Stamp: %lu, Vin: %f, Vout: %f, I: %f\n", stmp, aux1, aux2, aux3);
		SJC_DebugOutput_UsbPrintf("Vin: %f, Vout: %f, I: %f\n", aux1, aux2, aux3);

		break;

	default: // SJC_CONTROL_DISABLED
		SJC_DisableJointActuators();
		break;
	}
}

//void TIM4_IRQHandler(void)
//{
//	/* USER CODE BEGIN TIM4_IRQn 0 */
//	if (LL_TIM_IsActiveFlag_UPDATE(TIM4) & LL_TIM_IsEnabledIT_UPDATE(TIM4))
//	{
//		LL_TIM_ClearFlag_UPDATE(TIM4);
//		LL_ADC_INJ_StartConversionSWStart(ADC1);
//	}
//	/* USER CODE END TIM4_IRQn 0 */
//	/* USER CODE BEGIN TIM4_IRQn 1 */
//
//	/* USER CODE END TIM4_IRQn 1 */
//}
// ------------------------------------------------------------------- //
