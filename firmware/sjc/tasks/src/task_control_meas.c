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
#include "data_logger.h"
#include "task_common.h"
#include "test_signals.h"
#include "debug_output.h"
#include "network_configs.h"
#include "analog_interfaces.h"

// ------------------------------------------------------------------- //


// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#define TASK_NOTIFICATION_MAX_WAIT_TIME_MS				( (uint32_t ) 2U)
#define CURRENT_SAMPLES_FOR_BIAS_MEASUREMENT			( (uint32_t ) 3000UL)
// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //
typedef enum {
	TASK_MEASUREMENT_STATE_UNINITIALIZED = 0,
	TASK_MEASUREMENT_STATE_RUNNING
} eSjcTaskMeasurementState_t;

typedef enum {
	SJC_MOVE_BW = 0,
	SJC_MOVE_FW
} eSjcMoveDirection_t;

extern QueueHandle_t xDataLoggerQueueHandle;

static osTimerId_t xHeartbeatTimerId;
// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //
TaskHandle_t xControlAndMeasurementsTaskHandle;
static eSjcTaskMeasurementState_t xTaskState = TASK_MEASUREMENT_STATE_UNINITIALIZED;

static xSjcJointControllerMeasurements_t JointMeasurements = xSjcJointControllerMeasurementsEmptyObj;

static eSjcControlMode_t ControlMode;
static uint32_t ControllerPeriodSpec = 1000;
static float ControlTs = 0;
static uint32_t DeltaT = 0;

static float ActuatorSpeed = 0;
static float ActuatorRelativeDisplacement = 0;

float xTestSignalValue;
eSjcInputSignalState_t xTestSignalState;

static uint32_t TaskTic, TaskToc;
static uint32_t ControlTaskTic, ControlTaskToc;
static uint32_t ControllerCycleCounter;

static float CurrentBias = 0.0f;
static float CurrentMeasurementAux = 0.0f;
static uint32_t CurrentBiasMeasurementIdx = 0;
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
static void ControllerParametersUpdate(void);
static void EndstopsCheck(void);
static void MeasurementsUpdate(void);
static void EnqueueLogData(void);
static void RunTestSignal(eSjcInputType_t type);
static void RunControlLoop(void);
static bool CheckControlLoopCycle(void);

void vHeartbeatTimerCallback(void *argument);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void ControllerMeasTaskApp(void *argument)
{
	/* USER CODE BEGIN MeasurementsTaskApp */

	osDelay(SJC_TASK_INIT_DELAY);

	xControlAndMeasurementsTaskHandle = xTaskGetCurrentTaskHandle();

	SJC_TestSignals_Init();

	SJC_EnableJointActuators();

	SJC_DebugOutput_SystemPrintf("\r\n[OS] Controller and measurements task initiated!");


	xHeartbeatTimerId = osTimerNew(vHeartbeatTimerCallback, osTimerPeriodic, (void *)0, NULL);
	configASSERT(xHeartbeatTimerId != NULL);
	osTimerStart(xHeartbeatTimerId, SJC_HEARTBEAT_CALIBRATION_PERIOD_MS);


	uint32_t ulNotificationValue = 0;

	SJC_AnalogInterfaces_MeasurementEnable();

	xTaskState = TASK_MEASUREMENT_STATE_RUNNING;

	/* Infinite loop */
	for (;;) {

		SJC_GetTime(SJC_TIME_MS, &TaskTic);

		ulNotificationValue = ulTaskNotifyTake( pdTRUE, TASK_NOTIFICATION_MAX_WAIT_TIME_MS );

		if( ulNotificationValue == 1 )
		{
			SJC_AnalogInterfaces_UpdateConversionValues();

			RunControlLoop();
		}
		else
		{
			if (LL_TIM_IsEnabledCounter(SJC_PWM_INSTANCE))
			{
				SJC_DebugOutput_SystemPrintf("\n\n\n\r[OS FAULT] ADC Task notification error");
			}
		}

		SJC_GetTime(SJC_TIME_MS, &TaskToc);

		SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_MEASUREMENTS_RUNTIME, (TaskToc-TaskTic));
	}

	SJC_AnalogInterfaces_MeasurementDisable();
	SJC_SetJointActuatorVoltage(ZERO_FLOAT);
	SJC_DisableJointActuators();
	/* USER CODE END MainTaskApp */
}

static void RunControlLoop(void)
{
	if (true == CheckControlLoopCycle())
	{
		SJC_GetTime(SJC_TIME_MS, &ControlTaskTic);

		ControllerParametersUpdate();

		MeasurementsUpdate();

		EnqueueLogData();

		switch (ControlMode) {
		case SJC_PULSE_FAST_EXCITATION:
			JointMeasurements.ControlAction = ZERO_FLOAT;
			RunTestSignal(INPUT_TYPE_PULSE_FAST);
			break;

		case SJC_PULSE_SLOW_EXCITATION:
			JointMeasurements.ControlAction = ZERO_FLOAT;
			RunTestSignal(INPUT_TYPE_PULSE_SLOW);
			break;

		case SJC_TRIANG_EXCITATION:
			JointMeasurements.ControlAction = ZERO_FLOAT;
			RunTestSignal(INPUT_TYPE_TRIANG);
			break;

		case SJC_PRBS_EXCITATION:
			JointMeasurements.ControlAction = ZERO_FLOAT;
			RunTestSignal(INPUT_TYPE_PRBS);
			break;

		case SJC_TORQUE_CONTROL:
			JointMeasurements.ControllerError = JointMeasurements.ControllerReference - JointMeasurements.ActuatorCurrent;
			SJC_ComputeControlAction(JointMeasurements.ControllerError, ControlTs, &JointMeasurements.ControlAction);
			break;

		case SJC_SPEED_CONTROL:
			JointMeasurements.ControllerError = JointMeasurements.ControllerReference - ActuatorSpeed;
			SJC_ComputeControlAction(JointMeasurements.ControllerError, ControlTs, &JointMeasurements.ControlAction);
			break;

		case SJC_OPEN_LOOP_CONTROL:
			JointMeasurements.ControlAction = JointMeasurements.ControllerReference;
			break;

		case SJC_CONTROL_DISABLED:
			JointMeasurements.ControlAction = ZERO_FLOAT;
			break;

		default: // SJC_CURRENT_CALIBRATION
			SJC_GetJointActuatorCurrent(&CurrentMeasurementAux);
			CurrentBias += CurrentMeasurementAux;
			CurrentBiasMeasurementIdx++;
			if (CURRENT_SAMPLES_FOR_BIAS_MEASUREMENT == CurrentBiasMeasurementIdx)
			{
				CurrentBias /= CurrentBiasMeasurementIdx;
				SJC_SetJointActuatorCurrentBias(CurrentBias);
				SJC_SetControlMode(SJC_CONTROL_DISABLED);
				SJC_SetControlUpdatePeriodMs(SJC_CONTROLLER_MAX_UPDATE_PERIOD_MS);
				osTimerStop(xHeartbeatTimerId);
				osTimerStart(xHeartbeatTimerId, SJC_HEARTBEAT_CALLBACK_PERIOD_MS);
			}
			break;
		}

		// End stop ---------------------------------------------------------- //
		// ------------------------------------------------------------------- //
		EndstopsCheck();

		SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_CONTROLLER_SAMPLE_TIME, DeltaT);
		SJC_GetTime(SJC_TIME_MS, &ControlTaskToc);
		SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_CONTROLLER_RUNTIME, (ControlTaskToc-ControlTaskTic));
	}
}

static bool CheckControlLoopCycle(void)
{
	if ((++ControllerCycleCounter) >= ControllerPeriodSpec)
	{
		ControllerCycleCounter = 0;
		return true;
	}
	else
	{
		return false;
	}
}

void ADC_IRQHandler(void)
{
	if(LL_ADC_IsActiveFlag_JEOS(SJC_ADC_INSTANCE) != 0)
	{
		/* Clear flag ADC group injected end of sequence conversions */
		LL_ADC_ClearFlag_JEOS(SJC_ADC_INSTANCE);

		switch (xTaskState) {
		case TASK_MEASUREMENT_STATE_RUNNING:
			/* Call interruption treatment function */
			SJC_AnalogInterfaces_AdcConversionIsr();

			/* Dispatch conversion and filtering */
			if (SJC_RET_OK == SJC_AnalogInterfaces_CheckUpdateConversionTime())
			{
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				configASSERT( xControlAndMeasurementsTaskHandle != NULL );
				vTaskNotifyGiveFromISR( xControlAndMeasurementsTaskHandle, &xHigherPriorityTaskWoken );
				portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
			}
			break;
		default:
			break;
		}
	}

	/* Check whether ADC group regular overrun caused the ADC interruption */
	if(LL_ADC_IsActiveFlag_OVR(SJC_ADC_INSTANCE) != 0)
	{
		/* Clear flag ADC group regular overrun */
		LL_ADC_ClearFlag_OVR(SJC_ADC_INSTANCE);

		/* Call interruption treatment function */
		SJC_DebugOutput_SystemPrintf("\r\n[HW FAULT] ADC Overrun!\n");
		Error_Handler();
	}
}

static void ControllerParametersUpdate(void)
{
	// Controller configs update
	SJC_GetControlMode(&ControlMode);
	SJC_GetControlUpdatePeriodMs(&ControllerPeriodSpec);
	SJC_GetControlReference(&JointMeasurements.ControllerReference);
}

static void MeasurementsUpdate(void) {
	// Electric variables update
	SJC_GetJointActuatorVoltage(&JointMeasurements.ActuatorVoltage);
	SJC_GetJointActuatorCurrentUnbiased(&JointMeasurements.ActuatorCurrent);
	SJC_GetJointActuatorPowerSupplyVoltage(&JointMeasurements.SystemVoltage);

	// Actuated joint calcs
	SJC_GetJointRelativeAngularDisplacement(SJC_ACTUATOR_JOINT, &ActuatorRelativeDisplacement, JointMeasurements.ActuatorDisplacement);
	JointMeasurements.ActuatorDisplacement += ActuatorRelativeDisplacement;

	SJC_GetElapsedTime(SJC_TIME_MS, JointMeasurements.Timestamp, &DeltaT);
	JointMeasurements.Timestamp += DeltaT;

	ControlTs = DeltaT * 1.0e-3; // Convert miliseconds to seconds

	ActuatorSpeed = ActuatorRelativeDisplacement / ControlTs;
	JointMeasurements.ActuatorSpeedRpm = ActuatorSpeed * DEF_RAD_PER_SEC_TO_RPM;

	// Passive joints calcs
	//	SJC_GetJointAngularDisplacement(SJC_AUX_JOINT_A, &DisplacementThetaX);
	//	SJC_GetJointAngularDisplacement(SJC_AUX_JOINT_B, &DisplacementThetaY);
}

static void EndstopsCheck(void) {
	uint32_t states;
	SJC_ReadDigitalInputs(&states);
	bool bEndStopAPressed = (~states) & (1 << SJC_ISOLATED_INPUT_A);
	bool bEndStopBPressed = (~states) & (1 << SJC_ISOLATED_INPUT_B);
	bool bEndStopPressed = bEndStopAPressed | bEndStopBPressed;

	if (true == bEndStopAPressed) {
		SJC_ResetJointAngularDisplacement();
	}

	eSjcMoveDirection_t xActualMoveDirection = (JointMeasurements.ControlAction > 0) ? SJC_MOVE_FW : SJC_MOVE_BW;

	bool bNextMoveAllowed = (false == bEndStopPressed);
	bNextMoveAllowed |= ((true == bEndStopAPressed) && (xActualMoveDirection == SJC_MOVE_FW));
	bNextMoveAllowed |= ((true == bEndStopBPressed) && (xActualMoveDirection == SJC_MOVE_BW));

	if (true == bNextMoveAllowed)
	{
		SJC_SetJointActuatorVoltage(JointMeasurements.ControlAction);
	}
	else
	{
		SJC_SetJointActuatorVoltage(ZERO_FLOAT);
	}
}

static void RunTestSignal(eSjcInputType_t type)
{
	SJC_TestSignals_GetState(&xTestSignalState);

	if (INPUT_SIGNAL_STATE_INIT_CONFIG == xTestSignalState)
	{
		float time_factor = (4.0f/ControllerPeriodSpec); // From 4ms to 1ms
		SJC_TestSignals_SetConfig(type, time_factor);
	}
	else if (INPUT_SIGNAL_STATE_RUNNING == xTestSignalState)
	{
		SJC_TestSignals_GetValue(&xTestSignalValue);
		JointMeasurements.ControlAction = JointMeasurements.ControllerReference*xTestSignalValue;
	}
	else
	{
		SJC_TestSignals_Reset();
		SJC_SetControlMode(SJC_CONTROL_DISABLED);
	}
}

void vHeartbeatTimerCallback(void *argument)
{
	SJC_ToggleDigitalOutput(SJC_HEARTBEAT_OUTPUT);
}


static void EnqueueLogData(void)
{
	xQueueSend( xDataLoggerQueueHandle,
				( void * ) &JointMeasurements,
				( TickType_t ) 0 );
}

// ------------------------------------------------------------------- //
