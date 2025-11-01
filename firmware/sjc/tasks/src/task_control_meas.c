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
#include "base/include/sjc.h"
#include "common/include/network_configs.h"
#include "modules/include/analog_interfaces.h"
#include "modules/include/data_logger.h"
#include "modules/include/debug_output.h"
#include "modules/include/discrete_controller.h"
#include "modules/include/lqr_controller.h"
#include "modules/include/pid_controller.h"
#include "modules/include/test_signals.h"
#include "tasks/include/task_common.h"

// ------------------------------------------------------------------- //

// Task defines ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#define TASK_NOTIFICATION_MAX_WAIT_TIME_MS ((uint32_t)2U)
#define CURRENT_SAMPLES_FOR_BIAS_MEASUREMENT ((uint32_t)5000UL)

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

static xSjcPidParameters_t JointControllerParameters = xSjcPidParametersEmptyObj;
static xSjcJointControllerMeasurements_t JointMeasurements = xSjcJointControllerMeasurementsEmptyObj;

static eSjcControlMode_t ControlMode;
static uint32_t ControllerPeriodSpec_ms = 1000;
static float control_dt_sec = 0;
static uint32_t control_dt_ms = 0;

static float ActuatorAngularRate = 0;
static float ActuatorDisplacement = 0;

float SwingPositionThx = 0;
float SwingPositionThy = 0;
float SwingDisplacementThx = 0;
float SwingDisplacementThy = 0;

float xTestSignalValue;
eSjcInputSignalState_t xTestSignalState;

static uint32_t TaskTic, TaskToc, measurement_last_wake_tick;
static uint32_t ControlTaskTic, ControlTaskToc, last_control_wake_tick;

static uint32_t ControllerCycleCounter;

static float CurrentBias = 0.0f;
static float CurrentMeasurementAux = 0.0f;
static uint32_t CurrentBiasMeasurementIdx = 0;

// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
static void check_endstops(void);
static bool check_control_update_cycle(void);
static bool check_move_allowed(float command, bool stop_a_pressed, bool stop_b_pressed);

static void run_control_loop(void);
static void get_controller_measurements(void);
static void update_controller_parameters(void);

/**
 * @brief Run input test signal generator.
 *
 * Manages the test signal state machine:
 * - INIT_CONFIG: ensures actuator is disabled, converts control period
 *   from ms to seconds, and configures the generator.
 * - RUNNING: retrieves the next signal sample and applies it
 *   to the actuator reference.
 * - FINISHED: resets the generator, disables control mode,
 *   and forces actuator voltage to zero.
 *
 * @param type Test signal type (PRBS, triangular, fast pulse, slow pulse, ...)
 */
static void run_test_signals(eSjcInputType_t type);

static void enqueue_log_data(void);
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
	osDelay(SJC_TASK_INIT_DELAY);

	xControlAndMeasurementsTaskHandle = xTaskGetCurrentTaskHandle();
	AnalogInterfaces_registerUpdateTaskHandle(&xControlAndMeasurementsTaskHandle);
	uint32_t ulNotificationValue = 0;

	SJC_setControlMode(SJC_ANALOG_CALIBRATION);

	// Init joint controllers ------------------------------------------------------------------ //
	// ----------------------------------------------------------------------------------------- //
	/*
	 * Test signals
	 */
	TestSignals_init();
	/*
	 * LQI controller
	 */
	LQRObserver_init();
	LQRController_init();
	LQRController_setSaturation(-23, 23);
	/*
	 * PID controller
	 */
	PidController_init();
	xSjcRange_t PidControllerSaturation;
	PidControllerSaturation.Min = -23;
	PidControllerSaturation.Max = 23;
	PidController_set_saturation(PidControllerSaturation);
	/*
	 * Discrete controller
	 */
	DiscreteController_init();
	DiscreteController_setSaturation(-23, 23);
	// ----------------------------------------------------------------------------------------- //

	xHeartbeatTimerId = osTimerNew(vHeartbeatTimerCallback, osTimerPeriodic, (void *)0, NULL);
	configASSERT(xHeartbeatTimerId != NULL);

	AnalogInterfaces_enable();

	SJC_enableJointActuatorDriver();
	SJC_enableJointActuatorOutput();

	DebugOutput_systemPrintf("\r\n[OS] Controller and measurements task initiated!");

	SJC_getJointAngularDisplacement(SJC_ACTUATOR_JOINT, &ActuatorDisplacement, JointMeasurements.ActuatorAngularPosition);
	SJC_getJointAngularPosition(SJC_ACTUATOR_JOINT, &JointMeasurements.ActuatorAngularPosition);
	SJC_getJointAngularPosition(SJC_AUX_JOINT_TH_X, &SwingPositionThx);
	SJC_getJointAngularPosition(SJC_AUX_JOINT_TH_Y, &SwingPositionThy);

	/* Infinite loop */
	for (;;)
	{

		ulNotificationValue = ulTaskNotifyTake(pdTRUE, osWaitForever);

		if (ulNotificationValue == 1)
		{
			SJC_getTime(SJC_TIME_MS, &TaskTic);
			SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_MEASUREMENTS_TASK_PERIOD_MS, (TaskTic - measurement_last_wake_tick));
			measurement_last_wake_tick = TaskTic;

			AnalogInterfaces_updateConversionValues();

			if (true == check_control_update_cycle())
			{
				SJC_getTime(SJC_TIME_MS, &ControlTaskTic);
				SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_CONTROLLER_TASK_PERIOD_MS, (ControlTaskTic - last_control_wake_tick));
				last_control_wake_tick = ControlTaskTic;

				get_controller_measurements();

				update_controller_parameters();

				enqueue_log_data();

				run_control_loop();

				SJC_getTime(SJC_TIME_MS, &ControlTaskToc);
				SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_CONTROLLER_TASK_EXEC_MS, (ControlTaskToc - ControlTaskTic));
			}

			// End stop ---------------------------------------------------------- //
			// ------------------------------------------------------------------- //
			check_endstops();

			SJC_getTime(SJC_TIME_MS, &TaskToc);
			SJC_TaskCommon_SetTimeMeasurement(TIME_LOG_MEASUREMENTS_TASK_EXEC_MS, (TaskToc - TaskTic));
		}
	}

	AnalogInterfaces_disable();
	SJC_setJointActuatorVoltage(ZERO_FLOAT);
	SJC_disableJointActuatorOutput();
	SJC_disableJointActuatorDriver();
	/* USER CODE END MainTaskApp */
}

static void run_control_loop(void)
{
	switch (ControlMode)
	{
	case SJC_PULSE_FAST_EXCITATION:
		run_test_signals(INPUT_TYPE_PULSE_FAST);
		break;

	case SJC_PULSE_SLOW_EXCITATION:
		run_test_signals(INPUT_TYPE_PULSE_SLOW);
		break;

	case SJC_SINE_PRBS_EXCITATION:
		run_test_signals(INPUT_TYPE_SINE_PRBS);
		break;

	case SJC_SLOW_PRBS_EXCITATION:
		run_test_signals(INPUT_TYPE_LOW_FREQ_PRBS);
		break;

	case SJC_MID_PRBS_EXCITATION:
		run_test_signals(INPUT_TYPE_MID_FREQ_PRBS);
		break;

	case SJC_FAST_PRBS_EXCITATION:
		run_test_signals(INPUT_TYPE_FAST_FREQ_PRBS);
		break;

	case SJC_LQR_CURRENT_CONTROL:
		JointMeasurements.ControlAction = LQRController_compute(JointMeasurements.ControllerReference, JointMeasurements.ActuatorCurrent);
		break;

	case SJC_DISCRETE_CURRENT_CONTROL:
		JointMeasurements.ControlAction = DiscreteController_compute(JointMeasurements.ControllerReference, JointMeasurements.ActuatorCurrent);
		break;

	case SJC_PID_CURRENT_CONTROL:
		JointMeasurements.ControlAction = PidController_compute(JointMeasurements.ControllerReference, JointMeasurements.ActuatorCurrent, control_dt_sec);
		break;

	case SJC_PID_SPEED_CONTROL:
		JointMeasurements.ControlAction = PidController_compute(JointMeasurements.ControllerReference, JointMeasurements.ActuatorAngularRate, control_dt_sec);
		break;

	case SJC_OPEN_LOOP_CONTROL:
		JointMeasurements.ControlAction = JointMeasurements.ControllerReference;
		break;

	case SJC_CONTROL_DISABLED:
		JointMeasurements.ControlAction = ZERO_FLOAT;
		break;

	default: // SJC_CURRENT_CALIBRATION
	{
		SJC_setJointActuatorVoltage(ZERO_FLOAT);

		// Primeira amostra da calibração
		if (CurrentBiasMeasurementIdx == 0)
		{
			CurrentBias = 0.0f; // zera acumulador
			osTimerStop(xHeartbeatTimerId);
			osTimerStart(xHeartbeatTimerId, SJC_HEARTBEAT_CALIBRATION_PERIOD_MS);
		}

		// Acumula amostras
		SJC_getJointActuatorCurrent(&CurrentMeasurementAux);
		CurrentBias += CurrentMeasurementAux;
		CurrentBiasMeasurementIdx++;

		// Finalização da calibração
		if (CurrentBiasMeasurementIdx >= CURRENT_SAMPLES_FOR_BIAS_MEASUREMENT)
		{
			SJC_setControlMode(SJC_CONTROL_DISABLED);

			if (CurrentBiasMeasurementIdx > 0)
			{
				CurrentBias /= (float)CurrentBiasMeasurementIdx;
			}
			else
			{
				CurrentBias = 0.0f;
			}

			SJC_setJointActuatorCurrentBias(CurrentBias);
			DataLogger_transmit("\n\n\n\r[SJC] Estimated current bias: %.3f\n\n", CurrentBias);
			DebugOutput_systemPrintf("\n\n\n\r[SJC] Estimated current bias: %.3f\n\n", CurrentBias);

			// Reconfigura timer para operação normal
			SJC_setControlUpdatePeriodMs(SJC_CONTROLLER_MAX_UPDATE_PERIOD_MS);
			osTimerStop(xHeartbeatTimerId);
			osTimerStart(xHeartbeatTimerId, SJC_HEARTBEAT_CALLBACK_PERIOD_MS);

			// Prepara para nova calibração futura
			CurrentBiasMeasurementIdx = 0;
		}
	}
	break;
	}
}

static bool check_control_update_cycle(void)
{
	if ((++ControllerCycleCounter) >= ControllerPeriodSpec_ms)
	{
		ControllerCycleCounter = 0;
		return true;
	}
	else
	{
		return false;
	}
}

static void update_controller_parameters(void)
{
	// Controller configs update
	SJC_getControlMode(&ControlMode);

	SJC_getControlUpdatePeriodMs(&ControllerPeriodSpec_ms);

	SJC_getControlReference(&JointMeasurements.ControllerReference);
}

static void get_controller_measurements(void)
{
	// Electric variables update
	SJC_getJointActuatorVoltage(&JointMeasurements.ActuatorVoltage);
	SJC_getJointActuatorCurrentUnbiased(&JointMeasurements.ActuatorCurrent);
	SJC_getJointActuatorPowerSupplyVoltage(&JointMeasurements.SystemVoltage);

	// Actuated joint calcs
	SJC_getJointAngularDisplacement(SJC_ACTUATOR_JOINT, &ActuatorDisplacement, JointMeasurements.ActuatorAngularPosition);
	JointMeasurements.ActuatorAngularPosition += ActuatorDisplacement;

	SJC_getElapsedTime(SJC_TIME_MS, JointMeasurements.Timestamp_ms, &control_dt_ms);
	JointMeasurements.Timestamp_ms += control_dt_ms;

	control_dt_sec = CONVERT_MS_TO_SEC(control_dt_ms); // Convert miliseconds to seconds

	ActuatorAngularRate = ActuatorDisplacement / control_dt_sec;
	SJC_updateJointVelocity(SJC_ACTUATOR_JOINT, ActuatorAngularRate);

	JointMeasurements.ActuatorAngularRate = ActuatorAngularRate;
	JointMeasurements.ActuatorSpeedRpm = ActuatorAngularRate * DEF_RAD_PER_SEC_TO_RPM;

	// Passive joints calcs
	SJC_getJointAngularDisplacement(SJC_AUX_JOINT_TH_X, &SwingDisplacementThx, SwingPositionThx);
	SJC_getJointAngularDisplacement(SJC_AUX_JOINT_TH_Y, &SwingDisplacementThy, SwingPositionThy);

	SJC_updateJointVelocity(SJC_AUX_JOINT_TH_X, (SwingDisplacementThx / control_dt_sec));
	SJC_updateJointVelocity(SJC_AUX_JOINT_TH_Y, (SwingDisplacementThy / control_dt_sec));

	SwingPositionThx += SwingDisplacementThx;
	SwingPositionThy += SwingDisplacementThy;

	JointMeasurements.AuxPositionMeas01 = SwingPositionThx;
	JointMeasurements.AuxPositionMeas02 = SwingPositionThy;
}

static void check_endstops(void)
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

	bool bNextMoveAllowed = check_move_allowed(JointMeasurements.ControlAction, bEndStopAPressed, bEndStopBPressed);

	SJC_setJointActuatorVoltage(bNextMoveAllowed ? JointMeasurements.ControlAction : ZERO_FLOAT);
}

static bool check_move_allowed(float command, bool endstop_a_pressed, bool endstop_b_pressed)
{
	if (endstop_a_pressed && command < 0)
		return false;
	if (endstop_b_pressed && command > 0)
		return false;
	return true;
}

static void run_test_signals(eSjcInputType_t type)
{
	TestSignals_getState(&xTestSignalState);

	if (INPUT_SIGNAL_STATE_INIT_CONFIG == xTestSignalState)
	{
		// Ensure actuator is off before starting
		SJC_setJointActuatorVoltage(ZERO_FLOAT);

		// Convert control period (ms) to seconds
		float Ts_sec = (float)ControllerPeriodSpec_ms / 1000.0f;

		// Configure test signal generator
		TestSignals_setConfig(type, Ts_sec);
	}
	else if (INPUT_SIGNAL_STATE_RUNNING == xTestSignalState)
	{
		TestSignals_getValue(&xTestSignalValue);
		JointMeasurements.ControlAction =
			JointMeasurements.ControllerReference * xTestSignalValue;
	}
	else // INPUT_SIGNAL_STATE_FINISHED
	{
		TestSignals_reset();
		SJC_setControlMode(SJC_CONTROL_DISABLED);
		SJC_setJointActuatorVoltage(ZERO_FLOAT);
	}
}

void vHeartbeatTimerCallback(void *argument)
{
	SJC_toggleDigitalOutput(SJC_HEARTBEAT_OUTPUT);
}

static void enqueue_log_data(void)
{
	BaseType_t status = xQueueSend(xDataLoggerQueueHandle, (void *)&JointMeasurements, (TickType_t)0);
	if (pdPASS != status)
	{
		DebugOutput_systemPrintf("[WARNING] Failed to enqueue log data.");
	}
}

// ------------------------------------------------------------------- //
