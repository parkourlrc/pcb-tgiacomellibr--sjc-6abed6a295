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

#define IS_ISOLATED_INPUT_ACTIVE(states, pin) (((~(states)) >> (pin)) & 0x01)
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
static float dt = 0;
static uint32_t task_dt_ms = 0;
static bool control_mode_updated_ = false;

static float ActuatorAngularRate = 0;
static float ActuatorDisplacement = 0;

float SwingPositionThx = 0;
float SwingPositionThy = 0;
float SwingDisplacementThx = 0;
float SwingDisplacementThy = 0;

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
static bool IsMoveAllowed(float command, bool stop_a_pressed, bool stop_b_pressed);

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

	//	float Kw = 5.1100f;
	//	DiscreteController_init();
	//
	//	DiscreteController_setOutputLimits(-24.0f, 24.0f);
	//
	//	DiscreteController_setFilterCutoffFrequency(100);
	//
	//	DiscreteController_setAntiWindupGain(Kw);  // ou 1.0f / B0, se quiser experimentar

	/*
	 * LQI controller tests
	 */

#ifdef ROPE_AXIS
	//	Rope Motor
	// ----------------------------------------------------------------------------- //
	float Ad[2][2] = {{0.0000f, 1.0000f}, {-0.7864f, 1.7740f}};
	float Bd[2] = {0.1464f, 0.1139f};
	float Cd[2] = {1.0000f, 0.0000f};
	float L[2] = {1.159315, 1.360635};
	CurrentObserver_t current_observer;
	CurrentObserver_init(&current_observer, Ad, Bd, Cd, L);

	LqiControllerParams_t params = {
			.Ad = {{0.0000f, 1.0000f}, {-0.7864f, 1.7740f}},
			.Bd = {0.1464f, 0.1139f},
			.Cd = {1.0000f, 0.0000f},
			.Kx = {-2.9096f, 6.5371f},
			.Ki = -0.6049f,
			.Kw = 7.1767f,
			.alpha = 0.95678608f,
			.u_min = -24.0f,
			.u_max = 24.0f,
			.use_observer = true,
			.observer = &current_observer};
	// ----------------------------------------------------------------------------- //
#endif

#ifdef TROLLEY_AXIS
	//	Trolley Motor
	// ----------------------------------------------------------------------------- //
	float Ad[2][2] = {{0.0000f, 1.0000f}, {-0.7161f, 1.7032f}};
	float Bd[2] = {0.2938f, 0.2082f};
	float Cd[2] = {1.0000f, 0.0000f};
	float L[2] = {1.058990, 1.123099};
	CurrentObserver_t current_observer;
	CurrentObserver_init(&current_observer, Ad, Bd, Cd, L);

	LqiControllerParams_t params = {
			.Ad = {{0.0000f, 1.0000f}, {-0.7161f, 1.7032f}},
			.Bd = {0.2938f, 0.2082f},
			.Cd = {1.0000f, 0.0000f},
			.Kx = {-1.6827f, 3.0950f},
			.Ki = -0.2003f,
			.Kw = 9.9539f,
			.alpha = 0.95678608f,
			.u_min = -24.0f,
			.u_max = 24.0f,
			.use_observer = true,
			.observer = &current_observer};
	// ----------------------------------------------------------------------------- //
#endif

#ifdef BRIDGE_AXIS
	//	Bridge Motor
	// ----------------------------------------------------------------------------- //
	float Ad[2][2] = {{0.0000f, 1.0000f}, {-0.7456f, 1.7410f}};
	float Bd[2] = {0.7591f, 0.5642f};
	float Cd[2] = {1.0000f, 0.0000f};
	float L[2] = {0.855842, 0.797464};
	CurrentObserver_t current_observer;
	CurrentObserver_init(&current_observer, Ad, Bd, Cd, L);

	LqiControllerParams_t params = {
			.Ad = {{0.0000f, 1.0000f}, {-0.7456f, 1.7410f}},
			.Bd = {0.7591f, 0.5642f},
			.Cd = {1.0000f, 0.0000f},
			.Kx = {-0.0847f, 1.3151f},
			.Ki = -0.2241f,
			.Kw = 4.6308f,
			.alpha = 0.71539046f,
			.u_min = -24.0f,
			.u_max = 24.0f,
			.use_observer = true,
			.observer = &current_observer};
	// ----------------------------------------------------------------------------- //
#endif
	LqiController_init(&params);
	LqiController_setSaturation(-24.0f, 24.0f);

	SJC_getJointAngularDisplacement(SJC_ACTUATOR_JOINT, &ActuatorDisplacement, JointMeasurements.ActuatorAngularPosition);
	SJC_getJointAngularPosition(SJC_ACTUATOR_JOINT, &JointMeasurements.ActuatorAngularPosition);
	SJC_getJointAngularPosition(SJC_AUX_JOINT_TH_X, &SwingPositionThx);
	SJC_getJointAngularPosition(SJC_AUX_JOINT_TH_Y, &SwingPositionThy);

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
			//			SJC_computeControlAction(JointMeasurements.ControllerError, dt, &JointMeasurements.ControlAction);

			//			JointMeasurements.ControlAction = DiscreteController_compute(JointMeasurements.ControllerReference,JointMeasurements.ActuatorCurrent);
			JointMeasurements.ControlAction = LqiController_compute(JointMeasurements.ControllerReference, JointMeasurements.ActuatorCurrent);
			break;

		case SJC_SPEED_CONTROL:
			JointMeasurements.ControllerError = JointMeasurements.ControllerReference - ActuatorAngularRate;
			SJC_computeControlAction(JointMeasurements.ControllerError, dt, &JointMeasurements.ControlAction);
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
				if (CurrentBiasMeasurementIdx != 0)
				{
					CurrentBias /= CurrentBiasMeasurementIdx;
				}
				else
				{
					CurrentBias = 0;
				}

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

		SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_CONTROLLER_SAMPLE_TIME, task_dt_ms);
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
	SJC_getJointAngularDisplacement(SJC_ACTUATOR_JOINT, &ActuatorDisplacement, JointMeasurements.ActuatorAngularPosition);
	JointMeasurements.ActuatorAngularPosition += ActuatorDisplacement;

	SJC_getElapsedTime(SJC_TIME_MS, JointMeasurements.Timestamp_ms, &task_dt_ms);
	JointMeasurements.Timestamp_ms += task_dt_ms;

	dt = CONVERT_MS_TO_SEC(task_dt_ms); // Convert miliseconds to seconds

	ActuatorAngularRate = ActuatorDisplacement / dt;
	SJC_updateJointVelocity(SJC_ACTUATOR_JOINT, ActuatorAngularRate);

	JointMeasurements.ActuatorAngularRate = ActuatorAngularRate;
	JointMeasurements.ActuatorSpeedRpm = ActuatorAngularRate * DEF_RAD_PER_SEC_TO_RPM;

	// Passive joints calcs	
	SJC_getJointAngularDisplacement(SJC_AUX_JOINT_TH_X, &SwingDisplacementThx, SwingPositionThx);
	SJC_getJointAngularDisplacement(SJC_AUX_JOINT_TH_Y, &SwingDisplacementThy, SwingPositionThy);
	
	SJC_updateJointVelocity(SJC_AUX_JOINT_TH_X, (SwingDisplacementThx/dt));
	SJC_updateJointVelocity(SJC_AUX_JOINT_TH_Y, (SwingDisplacementThy/dt));

	SwingPositionThx += SwingDisplacementThx;
	SwingPositionThy += SwingDisplacementThy;

	JointMeasurements.AuxPositionMeas01 = SwingPositionThx;
	JointMeasurements.AuxPositionMeas02 = SwingPositionThy;
}

static void EndstopsCheck(void)
{
	uint32_t states;
	SJC_readDigitalInputs(&states);

#ifdef ROPE_AXIS
	bool bEndStopAPressed = IS_ISOLATED_INPUT_ACTIVE(states, SJC_ISOLATED_INPUT_A);
	bool bEndStopBPressed = IS_ISOLATED_INPUT_ACTIVE(states, SJC_ISOLATED_INPUT_B);
#else
	bool bEndStopAPressed = IS_ISOLATED_INPUT_ACTIVE(states, SJC_ISOLATED_INPUT_B);
	bool bEndStopBPressed = IS_ISOLATED_INPUT_ACTIVE(states, SJC_ISOLATED_INPUT_A);
#endif

	if (true == bEndStopAPressed)
	{
		SJC_resetJointAngularPosition();
	}

	bool bNextMoveAllowed = IsMoveAllowed(JointMeasurements.ControlAction, bEndStopAPressed, bEndStopBPressed);

	SJC_setJointActuatorVoltage(bNextMoveAllowed ? JointMeasurements.ControlAction : ZERO_FLOAT);
}

static bool IsMoveAllowed(float command, bool endstop_a_pressed, bool endstop_b_pressed)
{
	if (endstop_a_pressed && command < 0)
		return false;
	if (endstop_b_pressed && command > 0)
		return false;
	return true;
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
	BaseType_t status = xQueueSend(xDataLoggerQueueHandle, (void *)&JointMeasurements, (TickType_t)0);
	if (pdPASS != status)
	{
		DebugOutput_systemPrintf("[WARNING] Failed to enqueue log data.");
	}
}

// ------------------------------------------------------------------- //
