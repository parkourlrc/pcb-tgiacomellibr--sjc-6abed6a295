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
#define CONTROLLER_SHARED_DATA_MUTEX_WAIT_TIME_MS		( (uint32_t) 1U )

// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //
//extern ADC_HandleTypeDef hadc1;

// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //
osMutexId_t SystemMutexHandle;

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

extern volatile bool bMeasurementDone;
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
	SystemMutexHandle = MutexHandle;

	if (SystemMutexHandle == NULL)
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
	SJC_AnalogInterfaces_MeasurementEnable();

	uint32_t tick = osKernelGetTickCount();
	uint32_t ControlPeriod;

	/* Infinite loop */
	for(;;)
	{
		SJC_GetControlUpdatePeriodMs(&ControlPeriod);
		tick += pdMS_TO_TICKS(ControlPeriod);

		if (osOK == osMutexAcquire(SystemMutexHandle, CONTROLLER_SHARED_DATA_MUTEX_WAIT_TIME_MS))
		{
//			bMeasurementDone = false;
//			tick = osKernelGetTickCount();

			ControllerFunc();
			osMutexRelease(SystemMutexHandle);

			osDelayUntil(tick);
		}

	}
}


void ControllerFunc(void)
{
	eSjcControlMode_t ControlMode;
	float Ref, PsuVoltage = 0, ActuatorVoltage = 0, ActuatorCurrent = 0, ds = 0, dt = 0, vel = 0;
	static float Displacement;
	static uint32_t stmp;

	SJC_GetControlMode(&ControlMode);
	SJC_GetControlReference(&Ref);

	switch (ControlMode) {
	//	case SJC_SPEED_CONTROL:
	//	case SJC_TORQUE_CONTROL:
	//	case SJC_PRBS_EXCITATION:

	case SJC_OPEN_LOOP_CONTROL:
		SJC_EnableJointActuators();
		SJC_SetJointActuatorVoltage(Ref);
		SJC_GetJointActuatorPowerSupplyVoltage(&PsuVoltage);
		SJC_GetJointActuatorVoltage(&ActuatorVoltage);
		SJC_GetJointActuatorCurrent(&ActuatorCurrent);

		SJC_GetJointRelativeAngularDisplacement(SJC_ACTUATOR_JOINT, &ds, Displacement);
		Displacement += ds;

		dt = -1e-3*stmp;
		SJC_GetTime(SJC_TIME_MS, &stmp);
		dt += 1e-3*stmp;

		vel = ds*60/(2*M_PI*dt);
		vel /= 7600;

		break;

	default: // SJC_CONTROL_DISABLED
		SJC_DisableJointActuators();
		break;
	}

//	SJC_DebugOutput_UsbPrintf("\rStamp: %lu, Vin: %.2f, Vout: %.2f, Iout: %.2f, Dp: %.3f, Rpm: %.2f\n", stmp, (PsuVoltage/24.0f), (ActuatorVoltage/24.0f), ActuatorCurrent, ds, (vel/7600));
	SJC_DebugOutput_UsbPrintf("\rStamp, Vin, Vout, Iout, Rpm: \n");
//	SJC_DebugOutput_UsbPrintf("%.2f, %.2f, %.2f, %.4f\n", (PsuVoltage/24.0f), (ActuatorVoltage/24.0f), (ActuatorCurrent), (vel/7600));
	SJC_DebugOutput_UsbPrintf("\r%lu, %.2f, %.2f, %.2f, %.4f\n", stmp, (PsuVoltage), (ActuatorVoltage), (ActuatorCurrent), (vel));
}

// ------------------------------------------------------------------- //
