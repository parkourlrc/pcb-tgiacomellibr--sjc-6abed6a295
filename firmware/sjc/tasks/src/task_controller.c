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
#include "sjc.h"
#include "configs.h"
#include "debug_output.h"
#include "analog_interfaces.h"
// ------------------------------------------------------------------- //


// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#define CONTROLLER_SHARED_DATA_MUTEX_WAIT_TIME_MS		( (uint32_t) osWaitForever )

// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //
TaskHandle_t xControlTaskHandle;
eSjcControlMode_t ControlMode;
// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //
static uint32_t Timestamp = 0;
static uint32_t ControlPeriod;
static float ControllerReference;
static float ControllerError = 0;

static float Displacement = 0;
static float SystemVoltage = 0;
static float ActuatorVoltage = 0;
static float ActuatorCurrent = 0;
static float ElapsedTime = 0;
static float ActuatorAngularSpeed = 0;
static float ControlAction = 0;
static float RelativeDisplacement = 0;
static float ActuatorRpm = 0;
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
void PublishControllerData(void);
void ControllerParametersUpdate(void);
void TaskEventFromIsr(void);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void ControllerTaskApp(void *argument)
{
	/* USER CODE BEGIN ControllerTaskApp */
	osDelay(SJC_TASK_INIT_DELAY);

	xControlTaskHandle = xTaskGetCurrentTaskHandle();

	SJC_DebugOutput_printf("\r\n[OS] Controller task initiated!");


	uint32_t tick = osKernelGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		osDelayUntil(tick);

		ControllerParametersUpdate();

		PublishControllerData();

		tick += pdMS_TO_TICKS(ControlPeriod);

		switch (ControlMode) {
		case SJC_SPEED_CONTROL:
			SJC_EnableJointActuators();

			ControllerError = ControllerReference - ActuatorAngularSpeed;
			SJC_ComputeControlAction(ControllerError, ElapsedTime, &ControlAction);
			SJC_SetJointActuatorVoltage(ControlAction);
			break;

		case SJC_OPEN_LOOP_CONTROL:
			SJC_EnableJointActuators();
			SJC_SetJointActuatorVoltage(ControllerReference);
			break;

			//	case SJC_TORQUE_CONTROL:
			//	case SJC_PRBS_EXCITATION:
		default: // SJC_CONTROL_DISABLED
			SJC_DisableJointActuators();
			break;
		}
	}
}

void ControllerParametersUpdate(void)
{

	SJC_GetJointActuatorVoltage(&ActuatorVoltage);
	SJC_GetJointActuatorCurrent(&ActuatorCurrent);
	SJC_GetJointActuatorPowerSupplyVoltage(&SystemVoltage);
	SJC_GetControlReference(&ControllerReference);
	SJC_GetJointRelativeAngularDisplacement(SJC_ACTUATOR_JOINT, &RelativeDisplacement, Displacement);
	Displacement += RelativeDisplacement;

	ElapsedTime = -1e-3*Timestamp;
	SJC_GetTime(SJC_TIME_MS, &Timestamp);
	ElapsedTime += 1e-3*Timestamp;

	ActuatorAngularSpeed = RelativeDisplacement/ElapsedTime;
	ActuatorRpm = ActuatorAngularSpeed*(60)/(2*M_PI);

	SJC_GetControlUpdatePeriodMs(&ControlPeriod);
	SJC_GetControlMode(&ControlMode);
}

void PublishControllerData(void)
{
	//	SJC_DebugOutput_UsbPrintf("\rStamp: %lu, Vin: %.2f, Vout: %.2f, Iout: %.2f, Dp: %.3f, Rpm: %.2f\n", stmp, (PsuVoltage/24.0f), (ActuatorVoltage/24.0f), ActuatorCurrent, RelativeDisplacement, (ActuatorAngularSpeed/7600));
	//	SJC_DebugOutput_UsbPrintf("\rStamp, Vin, Vout, Iout, Vel, Rpm, ControllerReference, Err \n");
	//	SJC_DebugOutput_UsbPrintf("%.2f, %.2f, %.2f, %.4f\n", (SystemVoltage/24.0f), (ActuatorVoltage/24.0f), (ActuatorCurrent), (ActuatorAngularSpeed/7600));
	SJC_DebugOutput_UsbPrintf("\r%lu, %.2f, %.2f, %.2f, %.4f, %.4f, %.4f, %.4f\n", Timestamp, (SystemVoltage), (ActuatorVoltage), (ActuatorCurrent), (ActuatorAngularSpeed), ActuatorRpm, ControllerReference, ControllerError);
}


// ------------------------------------------------------------------- //
