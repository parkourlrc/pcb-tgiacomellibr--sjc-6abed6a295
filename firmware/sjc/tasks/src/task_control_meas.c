/*
 * network_task.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "cmsis_os.h"
#include "main.h"
// ------------------------------------------------------------------- //

// NETWORK includes -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Task includes ----------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "base/include/configs.h"
#include "base/include/network_configs.h"
#include "base/include/sjc.h"
#include "modules/include/analog_interfaces.h"
#include "modules/include/data_logger.h"
#include "modules/include/debug_output.h"
#include "modules/include/test_signals.h"
#include "tasks/include/task_common.h"

#include "modules/include/discrete_controller.h"
#include "modules/include/lqi_controller.h"

// ------------------------------------------------------------------- //

// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#define TASK_NOTIFICATION_MAX_WAIT_TIME_MS ((uint32_t)2U)
#define CURRENT_SAMPLES_FOR_BIAS_MEASUREMENT ((uint32_t)3000UL)
// ------------------------------------------------------------------- //

// Task data structures ---------------------------------------------- //
// ------------------------------------------------------------------- //

typedef enum
{
	SJC_MOVE_BW = 0,
	SJC_MOVE_FW
} eSjcMoveDirection_t;

extern QueueHandle_t xDataLoggerQueueHandle;

static osTimerId_t xHeartbeatTimerId;
// ------------------------------------------------------------------- //

// Task variables ---------------------------------------------------- //
// ------------------------------------------------------------------- //
TaskHandle_t xControlAndMeasurementsTaskHandle;

static xSjcServoPidParameters_t JointControllerParameters = xSjcServoPidParametersEmptyObj;
static xSjcJointControllerMeasurements_t JointMeasurements = xSjcJointControllerMeasurementsEmptyObj;

static eSjcControlMode_t ControlMode;
static eSjcControlMode_t LastControlMode;
static uint32_t ControllerPeriodSpec = 1000;
static float ControlTs = 0;
static uint32_t DeltaT = 0;
static bool control_mode_updated_ = false;

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

	TestSignals_init();

	SJC_enableJointActuatorDriver();
	SJC_enableJointActuatorOutput();

	xHeartbeatTimerId = osTimerNew(vHeartbeatTimerCallback, osTimerPeriodic, (void *)0, NULL);
	configASSERT(xHeartbeatTimerId != NULL);
	osTimerStart(xHeartbeatTimerId, SJC_HEARTBEAT_CALIBRATION_PERIOD_MS);

	osDelay(SJC_TASK_INIT_DELAY);

	uint32_t ulNotificationValue = 0;

	AnalogInterfaces_registerUpdateTaskHandle(&xControlAndMeasurementsTaskHandle);

	AnalogInterfaces_enable();

	SJC_setControlMode(SJC_ANALOG_CALIBRATION);
	LastControlMode = SJC_CONTROL_DISABLED;

	DebugOutput_systemPrintf("\r\n[OS] Controller and measurements task initiated!");

	/*
	 * Discrete controller tests
	 */

	float Kw = 5.1100f;
	DiscreteController_init();

	DiscreteController_setOutputLimits(-24.0f, 24.0f);

	DiscreteController_setFilterCutoffFrequency(100);

	DiscreteController_setAntiWindupGain(Kw);  // ou 1.0f / B0, se quiser experimentar

	/*
	 * LQI controller tests
	 */
//	float Kx[2] = {-7604.1397f, 104.8081f};
//	float Ki = -0.1864f;
//	float Kw = 1.54f;

	float Kx[2] = {-2.5100f, 4.3708f};
	float Ki = -0.1284f;
//	float Kw = 5.1100f;
	float cutoff_freq = 500;

	float Ad[2][2] = { {0.0f, 1.0f}, {-0.8039f, 1.7949f} };
	float Bd[2] = {0.1980f, 0.1583f};
	float Cd[2] = {1.0f, 0.0f};
//	float L[2] = {0.6077f, 0.6227f};
	float L[2] = {1.056964f, 1.206019f};

	CurrentObserver_t current_observer;
	CurrentObserver_init(&current_observer, Ad, Bd, Cd, L);


	LqiControllerParams_t params = {
			.Ad = { {0.0f, 1.0f}, {-0.8039f, 1.7949f} },
			.Bd = {0.1980f, 0.1583f},
			.Cd = {1.0f, 0.0f},
			.Kx[0] = Kx[0],
			.Kx[1] = Kx[1],
			.Ki = Ki,
			.Kw = Kw,

			.alpha = 1.0f - expf(-2.0f * 3.1415926f * cutoff_freq * 0.001f), // fc = 80 Hz, Ts = 1 ms
			.u_min = -24.0f,
			.u_max = 24.0f,
			.use_observer = true,
			.observer = &current_observer};

	LqiController_init(&params);
	LqiController_setSaturation(-24.0f, 24.0f);


	/* Infinite loop */
	for (;;)
	{
		SJC_getTime(SJC_TIME_MS, &TaskTic);

		ulNotificationValue = ulTaskNotifyTake(pdTRUE, TASK_NOTIFICATION_MAX_WAIT_TIME_MS);

		if (ulNotificationValue == 1)
		{
			AnalogInterfaces_updateConversionValues();

			RunControlLoop();
		}
		else
		{
			if (LL_TIM_IsEnabledCounter(SJC_PWM_INSTANCE))
			{
				DebugOutput_systemPrintf("\n\n\n\r[OS FAULT] ADC Task notification error");
			}
		}

		SJC_getTime(SJC_TIME_MS, &TaskToc);

		SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_MEASUREMENTS_RUNTIME, (TaskToc - TaskTic));
	}

	AnalogInterfaces_disable();
	SJC_setJointActuatorVoltage(ZERO_FLOAT);
	SJC_disableJointActuatorOutput();
	SJC_disableJointActuatorDriver();
	/* USER CODE END MainTaskApp */
}

static void RunControlLoop(void)
{
	if (true == CheckControlLoopCycle())
	{
		SJC_getTime(SJC_TIME_MS, &ControlTaskTic);

		ControllerParametersUpdate();

		MeasurementsUpdate();

		EnqueueLogData();

		switch (ControlMode)
		{
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

		case SJC_CURRENT_CONTROL:
			JointMeasurements.ControllerError = JointMeasurements.ControllerReference - JointMeasurements.ActuatorCurrent;
			//			SJC_computeControlAction(JointMeasurements.ControllerError, ControlTs, &JointMeasurements.ControlAction);

//			JointMeasurements.ControlAction = DiscreteController_compute(JointMeasurements.ControllerReference,JointMeasurements.ActuatorCurrent);
			JointMeasurements.ControlAction = LqiController_compute(JointMeasurements.ControllerReference, JointMeasurements.ActuatorCurrent);
			break;

		case SJC_SPEED_CONTROL:
			JointMeasurements.ControllerError = JointMeasurements.ControllerReference - ActuatorSpeed;
			SJC_computeControlAction(JointMeasurements.ControllerError, ControlTs, &JointMeasurements.ControlAction);
			break;

		case SJC_OPEN_LOOP_CONTROL:
			JointMeasurements.ControlAction = JointMeasurements.ControllerReference;
			break;

		case SJC_CONTROL_DISABLED:
			JointMeasurements.ControlAction = ZERO_FLOAT;
			break;

		default: // SJC_CURRENT_CALIBRATION
		{
			JointMeasurements.ControlAction = ZERO_FLOAT;

			if ((control_mode_updated_) && (CURRENT_SAMPLES_FOR_BIAS_MEASUREMENT == CurrentBiasMeasurementIdx))
			{
				CurrentBiasMeasurementIdx = 0;
				osTimerStop(xHeartbeatTimerId);
				osTimerStart(xHeartbeatTimerId, SJC_HEARTBEAT_CALIBRATION_PERIOD_MS);
			}
			SJC_getJointActuatorCurrent(&CurrentMeasurementAux);
			CurrentBias += CurrentMeasurementAux;
			CurrentBiasMeasurementIdx++;

			if (CURRENT_SAMPLES_FOR_BIAS_MEASUREMENT == CurrentBiasMeasurementIdx)
			{
				SJC_setControlMode(SJC_CONTROL_DISABLED);
				CurrentBias /= CurrentBiasMeasurementIdx;
				SJC_setJointActuatorCurrentBias(CurrentBias);
				DataLogger_transmit("\n\n\n\r[SJC] Estimated current bias: %.3f\n\n", CurrentBias);
				DebugOutput_systemPrintf("\n\n\n\r[SJC] Estimated current bias: %.3f\n\n", CurrentBias);
				SJC_setControlUpdatePeriodMs(SJC_CONTROLLER_MAX_UPDATE_PERIOD_MS);
				osTimerStop(xHeartbeatTimerId);
				osTimerStart(xHeartbeatTimerId, SJC_HEARTBEAT_CALLBACK_PERIOD_MS);
			}
		}
		break;
		}

		// End stop ---------------------------------------------------------- //
		// ------------------------------------------------------------------- //
		EndstopsCheck();

		SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_CONTROLLER_SAMPLE_TIME, DeltaT);
		SJC_getTime(SJC_TIME_MS, &ControlTaskToc);
		SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_CONTROLLER_RUNTIME, (ControlTaskToc - ControlTaskTic));
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

static void ControllerParametersUpdate(void)
{
	// Controller configs update
	SJC_getControlMode(&ControlMode);
	if (LastControlMode == ControlMode)
	{
		control_mode_updated_ = false;
	}
	else
	{
		control_mode_updated_ = true;
	}
	LastControlMode = ControlMode;

	SJC_getControlUpdatePeriodMs(&ControllerPeriodSpec);
	SJC_getControlParameters(&JointControllerParameters);
	SJC_getControlReference(&JointMeasurements.ControllerReference);
}

static void MeasurementsUpdate(void)
{
	// Electric variables update
	SJC_getJointActuatorVoltage(&JointMeasurements.ActuatorVoltage);
	SJC_getJointActuatorCurrentUnbiased(&JointMeasurements.ActuatorCurrent);
	SJC_getJointActuatorPowerSupplyVoltage(&JointMeasurements.SystemVoltage);

	// Actuated joint calcs
	SJC_getJointRelativeAngularDisplacement(SJC_ACTUATOR_JOINT, &ActuatorRelativeDisplacement, JointMeasurements.ActuatorDisplacement);
	JointMeasurements.ActuatorDisplacement += ActuatorRelativeDisplacement;

	SJC_getElapsedTime(SJC_TIME_MS, JointMeasurements.Timestamp, &DeltaT);
	JointMeasurements.Timestamp += DeltaT;

	ControlTs = DeltaT * 1.0e-3; // Convert miliseconds to seconds

	ActuatorSpeed = ActuatorRelativeDisplacement / ControlTs;
	JointMeasurements.ActuatorSpeedRpm = ActuatorSpeed * DEF_RAD_PER_SEC_TO_RPM;

	// Passive joints calcs
	//	SJC_GetJointAngularDisplacement(SJC_AUX_JOINT_A, &DisplacementThetaX);
	//	SJC_GetJointAngularDisplacement(SJC_AUX_JOINT_B, &DisplacementThetaY);
}

static void EndstopsCheck(void)
{
	uint32_t states;
	SJC_readDigitalInputs(&states);
	bool bEndStopAPressed = (~states) & (1 << SJC_ISOLATED_INPUT_A);
	bool bEndStopBPressed = (~states) & (1 << SJC_ISOLATED_INPUT_B);
	bool bEndStopPressed = bEndStopAPressed | bEndStopBPressed;

	if (true == bEndStopAPressed)
	{
		SJC_resetJointAngularDisplacement();
	}

	eSjcMoveDirection_t xActualMoveDirection = (JointMeasurements.ControlAction > 0) ? SJC_MOVE_FW : SJC_MOVE_BW;

	bool bNextMoveAllowed = (false == bEndStopPressed);
	bNextMoveAllowed |= ((true == bEndStopAPressed) && (xActualMoveDirection == SJC_MOVE_FW));
	bNextMoveAllowed |= ((true == bEndStopBPressed) && (xActualMoveDirection == SJC_MOVE_BW));

	if (true == bNextMoveAllowed)
	{
		SJC_setJointActuatorVoltage(JointMeasurements.ControlAction);
	}
	else
	{
		SJC_setJointActuatorVoltage(ZERO_FLOAT);
	}
}

static void RunTestSignal(eSjcInputType_t type)
{
	TestSignals_getState(&xTestSignalState);

	if (INPUT_SIGNAL_STATE_INIT_CONFIG == xTestSignalState)
	{
		float time_factor = (4.0f / ControllerPeriodSpec); // From 4ms to 1ms
		TestSignals_setConfig(type, time_factor);
	}
	else if (INPUT_SIGNAL_STATE_RUNNING == xTestSignalState)
	{
		TestSignals_getValue(&xTestSignalValue);
		JointMeasurements.ControlAction = JointMeasurements.ControllerReference * xTestSignalValue;
	}
	else
	{
		TestSignals_reset();
		SJC_setControlMode(SJC_CONTROL_DISABLED);
	}
}

void vHeartbeatTimerCallback(void *argument)
{
	SJC_toggleDigitalOutput(SJC_HEARTBEAT_OUTPUT);
}

static void EnqueueLogData(void)
{
	xQueueSend(xDataLoggerQueueHandle,
			   (void *)&JointMeasurements,
			   (TickType_t)0);
}

// ------------------------------------------------------------------- //
