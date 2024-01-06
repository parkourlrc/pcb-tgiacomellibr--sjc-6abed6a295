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
#define CONTROLLER_MAX_BLOCK_TIME_TICKS					pdMS_TO_TICKS(  ) )

#define ANALOG_CONV_MEASUREMENTS_TARGET					( (uint32_t ) 20UL)
#define ANALOG_CONVS_REACHED_EVENT_NOTIFICATION    		0x01

#define CONTROLLER_MUTEX_WAIT_TIME_MS					( portMAX_DELAY )

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
		.priority = (osPriority_t) osPriorityBelowNormal7,
};

// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
void ControllerFunc(void);
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

	uint32_t tick;
	tick = osKernelGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		uint32_t Period;
		SJC_GetControlUpdatePeriodMs(&Period);
		tick += pdMS_TO_TICKS(Period);

		if (osOK == osMutexAcquire(DataMutexHandle, CONTROLLER_MUTEX_WAIT_TIME_MS))
		{
			ControllerFunc();
			osMutexRelease(DataMutexHandle);
			osDelayUntil(tick);
		}
	}
}


void ControllerFunc(void)
{
	static uint32_t stmp;
	eSjcControlMode_t ControlMode;
	static float Ref, aux1, aux2, aux3, Displacement, ds, dt;

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
		SJC_GetJointActuatorPowerSupplyVoltage(&aux1);
		SJC_GetJointActuatorVoltage(&aux2);
		SJC_GetJointActuatorCurrent(&aux3);

		SJC_GetJointRelativeAngularDisplacement(SJC_ACTUATOR_JOINT, &ds, Displacement);
		Displacement += ds;

		dt = -1e-3*stmp;
		SJC_GetTime(SJC_TIME_MS, &stmp);
		dt += 1e-3*stmp;

		float vel = ds*60/(2*M_PI*dt);
		SJC_DebugOutput_UsbPrintf("\rVin: %.3f, Eff: %.3f, I: %.3f, Dp: %.3f, Rpm: %.3f\n", aux1, aux2, aux3, ds, vel);


		break;

	default: // SJC_CONTROL_DISABLED
		SJC_DisableJointActuators();
		break;
	}
}

// ------------------------------------------------------------------- //
