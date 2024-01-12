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
#define CONTROLLER_SHARED_DATA_MUTEX_WAIT_TIME_MS		( (uint32_t) osWaitForever )

// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //
TaskHandle_t xControlTaskHandle;

// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //
static uint32_t stmp, ulNotificationValue;
static float Displacement = 0;

float Ref, PsuVoltage = 0, ActuatorVoltage = 0, ActuatorCurrent = 0, dt = 0,
		vel = 0, error = 0, ControlAction = 0, ds = 0, rpm = 0;
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
void SpeedControllerFunc(void);
void ControllerTaskApp(void *argument);
void DataLogger(void);
void UpdateVariables(void);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void ControllerTaskApp(void *argument)
{
	xControlTaskHandle = xTaskGetCurrentTaskHandle();

	/* USER CODE BEGIN ControllerTaskApp */

	osDelay(SJC_TASK_INIT_DELAY);
	SJC_AnalogInterfaces_MeasurementEnable();
	SJC_DebugOutput_printf("\r\n[OS] Controller task initiated!");

	uint32_t tick = osKernelGetTickCount();
	uint32_t ControlPeriod;

	/* Infinite loop */
	for(;;)
	{
		SJC_GetControlUpdatePeriodMs(&ControlPeriod);
		tick += pdMS_TO_TICKS(ControlPeriod);

		eSjcControlMode_t ControlMode;
		SJC_GetControlMode(&ControlMode);

		switch (ControlMode) {
		case SJC_SPEED_CONTROL:
			SJC_EnableJointActuators();

			SpeedControllerFunc();
			break;

		case SJC_OPEN_LOOP_CONTROL:
			if (osOK == SJC_TaskCommon_SharedDataSemaphoreTake(CONTROLLER_SHARED_DATA_MUTEX_WAIT_TIME_MS))
			{
				SJC_EnableJointActuators();
				UpdateVariables();
				SJC_SetJointActuatorVoltage(Ref);
				DataLogger();

				SJC_TaskCommon_SharedDataSemaphoreGive();

				osDelayUntil(tick);
			}
			break;

			//	case SJC_TORQUE_CONTROL:
			//	case SJC_PRBS_EXCITATION:
		default: // SJC_CONTROL_DISABLED
			SJC_DisableJointActuators();
			break;
		}

	}
}

/*
 * void AnalogTaskApp(void *argument)
{
	osDelay(SJC_TASK_INIT_DELAY);
	SJC_DebugOutput_printf("\r\n[OS] Analog task initiated!");

	xTaskToNotify = xTaskGetCurrentTaskHandle();
	uint32_t ulNotificationValue;

	for(;;)
	{
		ulNotificationValue = ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

		if( ulNotificationValue == 1 )
		{
			LL_GPIO_TogglePin(SYS_HEARTBEAT_GPIO_Port, SYS_HEARTBEAT_Pin);
			ComputeSampleTime();
			ComputeAdcMeasurements();
		}
		else
		{
			// The call to ulTaskNotifyTake() timed out.
}

}
}
 */


void SpeedControllerFunc(void)
{
	ulNotificationValue = ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

	if( ulNotificationValue == 1 )
	{
		if (osOK == SJC_TaskCommon_SharedDataSemaphoreTake(CONTROLLER_SHARED_DATA_MUTEX_WAIT_TIME_MS))
		{
			UpdateVariables();
			error = Ref - vel;
			SJC_ComputeControlAction(error, dt, &ControlAction);
			SJC_SetJointActuatorVoltage(ControlAction);
			DataLogger();

			SJC_TaskCommon_SharedDataSemaphoreGive();
		}

	}
	else
	{
		// The call to ulTaskNotifyTake() timed out.
	}
}

void UpdateVariables(void)
{
	SJC_GetControlReference(&Ref);
	SJC_GetJointRelativeAngularDisplacement(SJC_ACTUATOR_JOINT, &ds, Displacement);
	Displacement += ds;

	dt = -1e-3*stmp;
	SJC_GetTime(SJC_TIME_MS, &stmp);
	dt += 1e-3*stmp;

	vel = ds/dt;
	rpm = vel*(60)/(2*M_PI);
}

void DataLogger(void)
{
	SJC_GetJointActuatorVoltage(&ActuatorVoltage);
	SJC_GetJointActuatorCurrent(&ActuatorCurrent);
	SJC_GetJointActuatorPowerSupplyVoltage(&PsuVoltage);

	//	SJC_DebugOutput_UsbPrintf("\rStamp: %lu, Vin: %.2f, Vout: %.2f, Iout: %.2f, Dp: %.3f, Rpm: %.2f\n", stmp, (PsuVoltage/24.0f), (ActuatorVoltage/24.0f), ActuatorCurrent, ds, (vel/7600));
	//	SJC_DebugOutput_UsbPrintf("\rStamp, Vin, Vout, Iout, Vel, Rpm, Ref, Err \n");
	//	SJC_DebugOutput_UsbPrintf("%.2f, %.2f, %.2f, %.4f\n", (PsuVoltage/24.0f), (ActuatorVoltage/24.0f), (ActuatorCurrent), (vel/7600));
	SJC_DebugOutput_UsbPrintf("\r%lu, %.2f, %.2f, %.2f, %.4f, %.4f, %.4f, %.4f\n", stmp, (PsuVoltage), (ActuatorVoltage), (ActuatorCurrent), (vel), rpm, Ref, error);
}

// ------------------------------------------------------------------- //
