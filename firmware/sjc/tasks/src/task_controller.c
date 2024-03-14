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
#include "test_signals.h"
#include "analog_interfaces.h"
// ------------------------------------------------------------------- //


// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#define CONTROLLER_SHARED_DATA_MUTEX_WAIT_TIME_MS		( (uint32_t) osWaitForever )

// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //
typedef enum {
	SJC_MOVE_BW = 0,
	SJC_MOVE_FW
} eSjcMoveDirection_t;
// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //
static TaskHandle_t xControlTaskHandle;
static eSjcControlMode_t ControlMode;

static float ElapsedTime = 0;
static uint32_t Timestamp = 0;
static uint32_t ControlPeriod = 1000;

static float ControlAction = 0;
static float ControllerError = 0;
static float ControllerReference = 0;

static float DisplacementThetaX = 0;
static float DisplacementThetaY = 0;

static float ActuatorDisplacement = 0;
static float ActuatorRelativeDisplacement = 0;
static float ActuatorSpeedRpm = 0;
static float ActuatorSpeed = 0;
static float ActuatorVoltage = 0;
static float ActuatorCurrent = 0;

static float SystemVoltage = 0;

float xTestSignalValue;
eSjcInputSignalState_t xTestSignalState;

// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
void PublishLogData(void);
void RunTestSignal(eSjcInputType_t type);
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

	SJC_TestSignals_Init();

	SJC_DebugOutput_SystemPrintf("\r\n[OS] Controller task initiated!");

	uint32_t tick = osKernelGetTickCount();

	SJC_EnableJointActuators();

	/* Infinite loop */
	for(;;)
	{
		osDelayUntil(tick);

		ControllerParametersUpdate();

		PublishLogData();

		tick += pdMS_TO_TICKS(ControlPeriod);

		switch (ControlMode) {
		case SJC_PULSE_EXCITATION:
			ControlAction = ZERO_FLOAT;
			RunTestSignal(INPUT_TYPE_PULSE);
			break;

		case SJC_TRIANG_EXCITATION:
			ControlAction = ZERO_FLOAT;
			RunTestSignal(INPUT_TYPE_TRIANG);
			break;

		case SJC_PRBS_EXCITATION:
			ControlAction = ZERO_FLOAT;
			RunTestSignal(INPUT_TYPE_PRBS);
			break;

		case SJC_TORQUE_CONTROL:
			ControlAction = ZERO_FLOAT;
			break;

		case SJC_SPEED_CONTROL:
			ControllerError = ControllerReference - ActuatorSpeed;
			SJC_ComputeControlAction(ControllerError, ElapsedTime, &ControlAction);
			break;

		case SJC_OPEN_LOOP_CONTROL:
			ControlAction = ControllerReference;
			break;

		default: // SJC_CONTROL_DISABLED
			ControlAction = ZERO_FLOAT;
			break;
		}

		// End stop ---------------------------------------------------------- //
		// ------------------------------------------------------------------- //
		uint32_t states;
		SJC_ReadDigitalInputs(&states);
		bool bEndStopAPressed = (~states) & (1 << SJC_ISOLATED_INPUT_A);
		bool bEndStopBPressed = (~states) & (1 << SJC_ISOLATED_INPUT_B);
		bool bEndStopPressed = bEndStopAPressed | bEndStopBPressed;

		if (true == bEndStopAPressed)
		{
			SJC_ResetJointAngularDisplacement();
		}

		eSjcMoveDirection_t xActualMoveDirection = (ControlAction > 0) ? SJC_MOVE_FW : SJC_MOVE_BW;

		bool bNextMoveAllowed = (false == bEndStopPressed);
		bNextMoveAllowed |= ( (true == bEndStopAPressed) && (xActualMoveDirection == SJC_MOVE_FW) );
		bNextMoveAllowed |= ( (true == bEndStopBPressed) && (xActualMoveDirection == SJC_MOVE_BW) );

		if (true == bNextMoveAllowed)
		{
			SJC_SetJointActuatorVoltage(ControlAction);
		}
		else
		{
			SJC_SetJointActuatorVoltage(ZERO_FLOAT);
		}

	}

	SJC_SetJointActuatorVoltage(ZERO_FLOAT);
	SJC_DisableJointActuators();
}
void ControllerParametersUpdate(void)
{
	// Electric variables update
	SJC_GetJointActuatorVoltage(&ActuatorVoltage);
	SJC_GetJointActuatorCurrent(&ActuatorCurrent);
	SJC_GetJointActuatorPowerSupplyVoltage(&SystemVoltage);

	// Actuated joint calcs
	SJC_GetJointRelativeAngularDisplacement(SJC_ACTUATOR_JOINT, &ActuatorRelativeDisplacement, ActuatorDisplacement);
	ActuatorDisplacement += ActuatorRelativeDisplacement;

	uint32_t Dt;
	SJC_GetElapsedTime(SJC_TIME_MS, Timestamp, &Dt);
	SJC_GetTime(SJC_TIME_MS, &Timestamp);

	ElapsedTime = ((float) ElapsedTime)*1.0e-3; // Convert miliseconds to seconds

	ActuatorSpeed = ActuatorRelativeDisplacement/ElapsedTime;
	ActuatorSpeedRpm = ActuatorSpeed*(30.0f/M_PI);

	// Passive joints calcs
	SJC_GetJointAngularDisplacement(SJC_AUX_JOINT_A, &DisplacementThetaX);
	SJC_GetJointAngularDisplacement(SJC_AUX_JOINT_B, &DisplacementThetaY);

	// Controller configs update
	SJC_GetControlMode(&ControlMode);
	SJC_GetControlUpdatePeriodMs(&ControlPeriod);
	SJC_GetControlReference(&ControllerReference);
}

void PublishLogData(void)
{
	SJC_DebugOutput_LoggerPrintf("\r%lu, %.2f, %.2f, %.2f, %.4f, %.4f, %.4ff\n", Timestamp, SystemVoltage, ControllerReference, ControllerError, ActuatorVoltage, ActuatorCurrent, ActuatorDisplacement, ActuatorSpeedRpm);
}

void RunTestSignal(eSjcInputType_t type)
{
	SJC_TestSignals_GetState(&xTestSignalState);

	if (INPUT_SIGNAL_STATE_CONFIG == xTestSignalState)
	{
		SJC_TestSignals_SetType(type);
	}
	else if (INPUT_SIGNAL_STATE_RUNNING == xTestSignalState)
	{
		SJC_TestSignals_GetValue(&xTestSignalValue);
		ControlAction = 15*xTestSignalValue;
	}
	else if (INPUT_SIGNAL_STATE_FINISHED == xTestSignalState)
	{
		SJC_TestSignals_Reset();
	}
	else
	{

	}
}


// ------------------------------------------------------------------- //
