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
#include "task_common.h"

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

// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
void ControllerFunc(void);
void ControllerTaskApp(void *argument);

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //


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

		if (osOK == SJC_TaskCommon_SharedDataMutexTake(CONTROLLER_SHARED_DATA_MUTEX_WAIT_TIME_MS))
		{
			//			bMeasurementDone = false;
			//			tick = osKernelGetTickCount();

			ControllerFunc();
			SJC_TaskCommon_SharedDataMutexGive();

			osDelayUntil(tick);
		}

	}
}


void ControllerFunc(void)
{
	eSjcControlMode_t ControlMode;
	float Ref, PsuVoltage = 0, ActuatorVoltage = 0, ActuatorCurrent = 0, dt = 0,
			vel = 0, error = 0, ControlAction = 0, ds = 0, rpm = 0;
	static float Displacement = 0;
	static uint32_t stmp;

	SJC_GetJointRelativeAngularDisplacement(SJC_ACTUATOR_JOINT, &ds, Displacement);
	Displacement += ds;

	dt = -1e-3*stmp;
	SJC_GetTime(SJC_TIME_MS, &stmp);
	dt += 1e-3*stmp;

	vel = ds/dt;
	rpm = vel*(60)/(2*M_PI);

	SJC_GetControlMode(&ControlMode);
	SJC_GetControlReference(&Ref);

	switch (ControlMode) {
	case SJC_SPEED_CONTROL:
		error = Ref - vel;
		SJC_ComputeControlAction(error, dt, &ControlAction);
		SJC_SetJointActuatorVoltage(ControlAction);
		break;
		//	case SJC_TORQUE_CONTROL:
		//	case SJC_PRBS_EXCITATION:

	case SJC_OPEN_LOOP_CONTROL:
		SJC_EnableJointActuators();
		SJC_SetJointActuatorVoltage(Ref);
		break;

	default: // SJC_CONTROL_DISABLED
		SJC_DisableJointActuators();
		break;
	}

	SJC_GetJointActuatorPowerSupplyVoltage(&PsuVoltage);
	SJC_GetJointActuatorVoltage(&ActuatorVoltage);
	SJC_GetJointActuatorCurrent(&ActuatorCurrent);

	//	SJC_DebugOutput_UsbPrintf("\rStamp: %lu, Vin: %.2f, Vout: %.2f, Iout: %.2f, Dp: %.3f, Rpm: %.2f\n", stmp, (PsuVoltage/24.0f), (ActuatorVoltage/24.0f), ActuatorCurrent, ds, (vel/7600));
//	SJC_DebugOutput_UsbPrintf("\rStamp, Vin, Vout, Iout, Vel, Rpm, Ref, Err \n");
	//	SJC_DebugOutput_UsbPrintf("%.2f, %.2f, %.2f, %.4f\n", (PsuVoltage/24.0f), (ActuatorVoltage/24.0f), (ActuatorCurrent), (vel/7600));
	SJC_DebugOutput_UsbPrintf("\r%lu, %.2f, %.2f, %.2f, %.4f, %.4f, %.4f, %.4f\n", stmp, (PsuVoltage), (ActuatorVoltage), (ActuatorCurrent), (vel), rpm, Ref, error);
}

// ------------------------------------------------------------------- //
