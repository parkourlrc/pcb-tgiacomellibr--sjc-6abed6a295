/*
 * sjc.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "dma.h"
#include "gpio.h"
#include "usb_device.h"
// ------------------------------------------------------------------- //

// NETWORK includes -------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "mb.h"
#include "port.h"
// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "base/include/sjc.h"
#include "system_types/include/types.h"

#include "base/include/configs.h"
#include "modules/include/analog_interfaces.h"
#include "modules/include/data_logger.h"
#include "modules/include/debug_output.h"
#include "modules/include/digital_interfaces.h"
#include "modules/include/discrete_controller.h"
#include "modules/include/encoder.h"
#include "modules/include/lqr_controller.h"
#include "modules/include/modbus_network.h"
#include "modules/include/motor_drive.h"
#include "modules/include/motor_output.h"
#include "modules/include/pid_controller.h"
#include "modules/include/pwm_signal.h"
#include "modules/include/timeout_timer.h"
#include "tasks/include/task_common.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
extern volatile USHORT usRegInputBuf[INPUT_REG_NREGS];
extern volatile USHORT usRegHoldingBuf[HOLDING_REG_NREGS];

static xSjcControlConfigs_t xSjcControlConfigs = xSjcControlConfigsEmptyObj;

static float fBiasCurrentValue = 0;
static float fJointVelocityMeasurementFiltered[SJC_JOINTS_SIZE];
static ExponentialFilter_t xVelocityFilter[SJC_JOINTS_SIZE];
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //

// Input registers update process ------------------------------------ //
static void Net_UpdateBoardInfoRegisters(void);
static void Net_UpdateNetworkInfoRegisters(void);
static void Net_UpdateDigitalIOInfoRegisters(void);
static void Net_UpdateControlMeasInfoRegisters(void);
static void Net_UpdateActuatorParametersRegisters(void);
static void Net_UpdatePidControllerRegisters(void);
static void Net_UpdateLQRControllerRegisters(void);
static void Net_UpdateDigitalControllerRegisters(void);
static void Net_UpdateSensorsParametersRegisters(void);

// Holding registers update process ---------------------------------- //
static void Net_ExecuteSystemConfigsUpdateCmd(void);
static void Net_ExecuteDigitalOutputsUpdateCmd(void);
static void Net_ExecuteControllerModeUpdateCmd(void);
static void Net_ExecuteControllerFrequencyUpdateCmd(void);
static void Net_ExecuteControllerRefUpdateCmd(void);
static void Net_ExecuteActuatorConfigsUpdateCmd(void);
static void Net_ExecuteNetworkConfigsUpdateCmd(void);
static void Net_ExecuteSensorsConfigsUpdateCmd(void);
static void Net_ExecutePIDControllerParametersUpdateCmd(void);
static void Net_ExecuteLQRControllerParametersUpdateCmd(void);
static void Net_ExecuteDigitalControllerParametersUpdateCmd(void);

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void SJC_init(void)
{
	// SJC Peripheral initialization

	// Init all board GPIOs
	MX_GPIO_Init();

	// Init DMA controller
	MX_DMA_Init();

	// Init USB and Serial debug devices
	MX_USART3_UART_Init();
	MX_USB_DEVICE_Init();

#if SJC_HW_VERSION_MAJOR > 1
	// Init Serial datalogger device
	MX_UART5_Init();
#endif

	// Init Joint PWM and Encoders
	MX_TIM1_Init();

	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM8_Init();

	// Init Serial communication device
	MX_USART2_UART_Init();

	// Init ADC and Sample time check timer
	MX_TIM4_Init();
	MX_TIM10_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	fBiasCurrentValue = 0;

	HAL_Delay(1000);

	// SJC OS initialization
	SJC_initOsKernel();
	SJC_initOsObjects();

	// SJC version setup
	SJC_setVersions();

	// SJC Modules initialization
	DataLogger_init();
	DebugOutput_init();
	AnalogInterfaces_init();
	DigitalInterfaces_init();

	Encoder_init();
	Encoder_enable(SJC_AUX_JOINT_TH_X);
	Encoder_enable(SJC_AUX_JOINT_TH_Y);
	Encoder_enable(SJC_ACTUATOR_JOINT);

	PidController_init();

	for (size_t i = 0; i < SJC_JOINTS_SIZE; i++)
	{
		ExponentialFilter_init(&xVelocityFilter[i], 100.0, 1000.0);
	}

	xSjcControlConfigs.Mode = SJC_CONTROL_DISABLED;
	xSjcControlConfigs.UpdateRate = SJC_CONTROLLER_MAX_UPDATE_RATE_HZ;

	MotorOutput_init();
	MotorDrive_init(PWM_FREQ_20kHz, 24.0);

	// SJC O.S. tasks initialization
	SJC_initOsThreads();
}

void SJC_initOsKernel(void)
{
	/* Init scheduler */
	osKernelInitialize(); /* Call init function for freertos objects (in freertos.c) */
}

void SJC_initOsObjects(void)
{
	SJC_TaskCommon_initObjects();
}

void SJC_initOsThreads(void)
{
	/* Init Startup Tasks */
	SJC_TaskCommon_initThreads();

	/* Start scheduler */
	osKernelStart();

	while (1)
	{
	}
}

void SJC_presentSystemInfo(void)
{
	DebugOutput_presentSystemInfo();
}

eSjcReturnType_t SJC_setVersions(void)
{
	usRegInputBuf[INPUT_REG_HW_VERSION] = SJC_HW_VERSION_MAJOR << 8 | SJC_HW_VERSION_MINOR;

	usRegInputBuf[INPUT_REG_FW_VERSION_MAJOR_MINOR] = SJC_FW_VERSION_MAJOR << 8 | SJC_FW_VERSION_MINOR;
	usRegInputBuf[INPUT_REG_FW_VERSION_PATCH + 1] = SJC_FW_VERSION_PATCH;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_delay(const eSjcTimeType_t type, const uint32_t value)
{
	eSjcReturnType_t ret = SJC_RET_OK;

	if (value == 0)
	{
		return SJC_RET_NOK;
	}

	switch (type)
	{
	case SJC_TIME_MS:
		TimeoutTimer_delayMs(value);
		break;
	case SJC_TIME_SEC:
		TimeoutTimer_delaySec(value);
		break;
	default:
		ret = SJC_RET_NOK;
		break;
	}
	return ret;
}

eSjcReturnType_t SJC_getTime(const eSjcTimeType_t type, uint32_t *value)
{
	switch (type)
	{
	case SJC_TIME_MS:
		*value = TimeoutTimer_getTimeMs();
		break;
	case SJC_TIME_SEC:
		*value = TimeoutTimer_getTimeSec();
		break;
	default:
		*value = 0;
		return SJC_RET_NOK;
		break;
	}
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getElapsedTime(const eSjcTimeType_t type, const uint32_t timepoint, uint32_t *value)
{
	if (!value)
	{
		return SJC_RET_NOK;
	}

	uint32_t now;
	switch (type)
	{
	case SJC_TIME_MS:
		now = TimeoutTimer_getTimeMs();
		break;
	case SJC_TIME_SEC:
		now = TimeoutTimer_getTimeSec();
		break;
	default:
		return SJC_RET_NOK;
	}

	*value = (uint32_t)(now - timepoint);
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getControlMode(eSjcControlMode_t *config)
{
	*config = xSjcControlConfigs.Mode;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_setControlMode(const eSjcControlMode_t config)
{
	if (config < SJC_CONTROL_MODE_SIZE)
	{
		if (config == SJC_ANALOG_CALIBRATION)
		{
			SJC_setJointActuatorVoltage(ZERO_FLOAT);
		}
		xSjcControlConfigs.Mode = config;

		return SJC_RET_OK;
	}

	return SJC_RET_NOK;
}

eSjcReturnType_t SJC_getControlUpdateRateHz(float *config)
{
	if (!config)
	{
		return SJC_RET_NOK;
	}

	*config = xSjcControlConfigs.UpdateRate;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_setControlUpdateRateHz(const float config)
{
	xSjcControlConfigs.UpdateRate = SJC_Helpers_RealClamp(config, SJC_CONTROLLER_MIN_UPDATE_RATE_HZ, SJC_CONTROLLER_MAX_UPDATE_RATE_HZ);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getControlUpdatePeriodMs(uint32_t *period_ms)
{
	if (!period_ms || xSjcControlConfigs.UpdateRate <= 0.0f)
	{
		return SJC_RET_NOK;
	}

	*period_ms = ((uint32_t)(1000.0f / xSjcControlConfigs.UpdateRate));
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_setControlUpdatePeriodMs(const uint32_t period_ms)
{
	if (period_ms == 0)
		return SJC_RET_NOK;
	float clamped = (float)SJC_Helpers_UintClamp(period_ms, SJC_CONTROLLER_MIN_UPDATE_PERIOD_MS, SJC_CONTROLLER_MAX_UPDATE_PERIOD_MS);
	xSjcControlConfigs.UpdateRate = 1000.0f / clamped;
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getControlReference(float *reference)
{
	if (!reference)
	{
		return SJC_RET_NOK;
	}

	*reference = xSjcControlConfigs.Reference;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_setControlReference(const float reference)
{
	xSjcControlConfigs.Reference = reference;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_resetController(void)
{
	switch (xSjcControlConfigs.Mode)
	{
	case SJC_PID_SPEED_CONTROL:
	case SJC_PID_CURRENT_CONTROL:
		PidController_reset();
		break;

	case SJC_LQR_CURRENT_CONTROL:
		LQRController_reset();
		break;

	case SJC_DISCRETE_CURRENT_CONTROL:
		DiscreteController_reset();
		break;

	default:
		break;
	}
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getControlError(float *controller_error)
{
	if (!controller_error)
	{
		return SJC_RET_NOK;
	}

	switch (xSjcControlConfigs.Mode)
	{
	case SJC_PID_SPEED_CONTROL:
	case SJC_PID_CURRENT_CONTROL:
		*controller_error = PidController_get_error();
		break;

	case SJC_LQR_CURRENT_CONTROL:
		*controller_error = LQRController_get_error();
		break;

	case SJC_DISCRETE_CURRENT_CONTROL:
		*controller_error = DiscreteController_get_error();
		break;

	default:
		*controller_error = 0.0f;
		break;
	}

	return SJC_RET_OK;
}

void SJC_updateHoldingRegisters(void)
{
	Net_ExecuteSystemConfigsUpdateCmd();
	Net_ExecuteDigitalOutputsUpdateCmd();
	Net_ExecuteControllerModeUpdateCmd();
	Net_ExecuteControllerFrequencyUpdateCmd();
	Net_ExecuteControllerRefUpdateCmd();
	Net_ExecuteActuatorConfigsUpdateCmd();
	Net_ExecuteNetworkConfigsUpdateCmd();
	Net_ExecuteSensorsConfigsUpdateCmd();
	Net_ExecutePIDControllerParametersUpdateCmd();
	Net_ExecuteLQRControllerParametersUpdateCmd();
	Net_ExecuteDigitalControllerParametersUpdateCmd();
}

void SJC_updateInputRegisters(void)
{
	Net_UpdateBoardInfoRegisters();
	Net_UpdateNetworkInfoRegisters();
	Net_UpdateDigitalIOInfoRegisters();
	Net_UpdateControlMeasInfoRegisters();
	Net_UpdateActuatorParametersRegisters();
	Net_UpdatePidControllerRegisters();
	Net_UpdateLQRControllerRegisters();
	Net_UpdateDigitalControllerRegisters();
	Net_UpdateSensorsParametersRegisters();
}

void SJC_executeNetworkDaemon(void)
{
	ModbusNetwork_update();
}

eSjcReturnType_t SJC_enableAdc(void)
{
	AnalogInterfaces_enable();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_disableAdc(void)
{
	AnalogInterfaces_disable();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_enableJointActuatorDriver(void)
{
	MotorDrive_enable();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_disableJointActuatorDriver(void)
{
	MotorDrive_disable();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getJointActuatorDriverState(bool *state)
{
	return MotorDrive_getState(state);
}

eSjcReturnType_t SJC_enableJointActuatorOutput(void)
{
	MotorOutput_enable();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_disableJointActuatorOutput(void)
{
	MotorOutput_disable();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getJointActuatorOutputState(bool *state)
{
	return MotorOutput_getState(state);
}

eSjcReturnType_t SJC_getJointActuatorVoltage(float *voltage)
{
	return MotorDrive_getOutputVoltage(voltage);
}

eSjcReturnType_t SJC_setJointActuatorVoltage(const float voltage)
{
	MotorDrive_setOutputVoltage(voltage);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_resetJointAngularPosition(void)
{
	Encoder_reset(SJC_ACTUATOR_JOINT);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_resetAuxJointsAngularPosition(void)
{
	Encoder_reset(SJC_AUX_JOINT_TH_X);
	Encoder_reset(SJC_AUX_JOINT_TH_Y);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getJointAngularPosition(const eSjcJoints_t joint, float *disp)
{
	if (!disp || joint >= SJC_JOINTS_SIZE)
	{
		return SJC_RET_NOK;
	}
	return Encoder_getAngularDisplacement(joint, disp);
}

eSjcReturnType_t SJC_getJointAngularDisplacement(const eSjcJoints_t joint, float *disp, const float prev_position)
{
	if (!disp || joint >= SJC_JOINTS_SIZE)
	{
		return SJC_RET_NOK;
	}
	eSjcReturnType_t ret = Encoder_getAngularDisplacement(joint, disp);

	if (ret == SJC_RET_OK)
	{
		*disp -= prev_position;
	}

	return ret;
}

eSjcReturnType_t SJC_resetJointVelocity(const eSjcJoints_t joint)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	if (joint < SJC_JOINTS_SIZE)
	{
		ExponentialFilter_reset(&xVelocityFilter[joint]);
		fJointVelocityMeasurementFiltered[joint] = 0.0f;
		ret = SJC_RET_OK;
	}

	return ret;
}

eSjcReturnType_t SJC_getJointAngularVelocity(const eSjcJoints_t joint, float *vel)
{
	if (!vel || joint >= SJC_JOINTS_SIZE)
	{
		return SJC_RET_NOK;
	}

	*vel = fJointVelocityMeasurementFiltered[joint];
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_updateJointVelocity(const eSjcJoints_t joint, const float vel)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	if (joint < SJC_JOINTS_SIZE)
	{
		fJointVelocityMeasurementFiltered[joint] = ExponentialFilter_update(&xVelocityFilter[joint], vel);
		ret = SJC_RET_OK;
	}

	return ret;
}

eSjcReturnType_t SJC_getJointActuatorPwmFrequency(uint32_t *freq)
{
	if (!freq)
	{
		return SJC_RET_NOK;
	}

	*freq = PwmSignal_getFrequency();
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_setJointActuatorCurrentBias(const float bias)
{
	fBiasCurrentValue = bias;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getJointActuatorCurrent(float *value)
{
	return MotorDrive_getOutputCurrent(value);
}

eSjcReturnType_t SJC_getJointActuatorCurrentUnbiased(float *value)
{
	if (!value)
	{
		return SJC_RET_NOK;
	}

	eSjcReturnType_t ret = MotorDrive_getOutputCurrent(value);
	if (ret == SJC_RET_OK)
	{
		*value -= fBiasCurrentValue;
	}
	return ret;
}

eSjcReturnType_t SJC_getJointActuatorPowerSupplyVoltage(float *value)
{
	return MotorDrive_getPowerSupplyVoltage(value);
}

eSjcReturnType_t SJC_readDigitalInputs(uint32_t *states)
{
	return DigitalInterfaces_readInputStates(states);
}

eSjcReturnType_t SJC_readDigitalOutputs(uint32_t *states)
{
	return DigitalInterfaces_readOutputStates(states);
}

eSjcReturnType_t SJC_writeDigitalOutputs(const uint8_t states)
{
	return DigitalInterfaces_writeOutputStates(states);
}

eSjcReturnType_t SJC_toggleDigitalOutput(const eSjcDigitalOutputs_t Output)
{
	return DigitalInterfaces_toggleOutputState(Output);
}

eSjcReturnType_t SJC_readDigitalInput(const eSjcDigitalInputs_t Input, bool *state)
{
	return DigitalInterfaces_readInputState(Input, state);
}

eSjcReturnType_t SJC_readDigitalOutput(const eSjcDigitalOutputs_t Output, bool *state)
{
	return DigitalInterfaces_readOutputState(Output, state);
}

eSjcReturnType_t SJC_writeDigitalOutput(const eSjcDigitalOutputs_t Output, const bool state)
{
	return DigitalInterfaces_writeOutputState(Output, state);
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// Input registers update process ------------------------------------ //
static void Net_UpdateBoardInfoRegisters(void)
{
	uint16_t w[2];
	float fAux;
	eSjcControlMode_t control_mode;

	// Control mode
	SJC_getControlMode(&control_mode);
	usRegInputBuf[INPUT_REG_CONTROL_MODE] = (uint16_t)control_mode;

	SJC_getControlUpdateRateHz(&fAux);
	compose_ui16_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_CONTROL_FREQUENCY] = w[0];
	usRegInputBuf[INPUT_REG_CONTROL_FREQUENCY + 1] = w[1];

	// #TODO: Error flags
	usRegInputBuf[INPUT_REG_SYSTEM_ERROR_FLAGS] = 0x0000;
}

static void Net_UpdateNetworkInfoRegisters(void)
{
	// Network info
	uint16_t addr;
	uint32_t baud;
	uint16_t w[2];
	char parity;

	ModbusNetwork_getParameters(&addr, &baud, &parity);
	compose_ui16_array_from_ui32(baud, w);

	usRegInputBuf[INPUT_REG_NETWORK_ADDRESS] = addr;
	usRegInputBuf[INPUT_REG_NETWORK_PARITY] = (uint16_t)parity;
	usRegInputBuf[INPUT_REG_NETWORK_BAUD_RATE] = w[0];
	usRegInputBuf[INPUT_REG_NETWORK_BAUD_RATE + 1] = w[1];
}

static void Net_UpdateDigitalIOInfoRegisters(void)
{
	// Digital IOs
	uint32_t u32 = 0;
	SJC_readDigitalInputs(&u32);
	usRegInputBuf[INPUT_REG_DIGITAL_INPUTS] = (uint16_t)u32;

	u32 = 0;
	SJC_readDigitalOutputs(&u32);
	usRegInputBuf[INPUT_REG_DIGITAL_OUTPUTS] = (uint16_t)u32;
}

static void Net_UpdateControlMeasInfoRegisters(void)
{
	float fAux = 0.0f;
	uint16_t w[2];

	// Uptime (u32 -> 2x u16)
	uint32_t uptime = TimeoutTimer_getTimeMs();
	compose_ui16_array_from_ui32(uptime, w);
	usRegInputBuf[INPUT_REG_SYSTEM_UPTIME] = w[0];
	usRegInputBuf[INPUT_REG_SYSTEM_UPTIME + 1] = w[1];

	// Control error (f32 -> 2x u16)
	fAux = 0.0f;
	SJC_getControlError(&fAux);
	compose_ui16_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_CONTROL_ERROR] = w[0];
	usRegInputBuf[INPUT_REG_CONTROL_ERROR + 1] = w[1];

	// Control reference
	fAux = 0.0f;
	SJC_getControlReference(&fAux);
	compose_ui16_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_CONTROL_REFERENCE] = w[0];
	usRegInputBuf[INPUT_REG_CONTROL_REFERENCE + 1] = w[1];

	// Actuator current (unbiased)
	fAux = 0.0f;
	SJC_getJointActuatorCurrentUnbiased(&fAux);
	compose_ui16_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_ACTUATOR_CURRENT] = w[0];
	usRegInputBuf[INPUT_REG_ACTUATOR_CURRENT + 1] = w[1];

	// Actuator voltage
	fAux = 0.0f;
	SJC_getJointActuatorVoltage(&fAux);
	compose_ui16_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_ACTUATOR_VOLTAGE] = w[0];
	usRegInputBuf[INPUT_REG_ACTUATOR_VOLTAGE + 1] = w[1];

	// Actuator position
	fAux = 0.0f;
	SJC_getJointAngularPosition(SJC_ACTUATOR_JOINT, &fAux);
	compose_ui16_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_ACTUATOR_POSITION] = w[0];
	usRegInputBuf[INPUT_REG_ACTUATOR_POSITION + 1] = w[1];

	// Aux THX position
	fAux = 0.0f;
	SJC_getJointAngularPosition(SJC_AUX_JOINT_TH_X, &fAux);
	compose_ui16_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_AUX_SWING_POSITION_THX] = w[0];
	usRegInputBuf[INPUT_REG_AUX_SWING_POSITION_THX + 1] = w[1];

	// Aux THY position
	fAux = 0.0f;
	SJC_getJointAngularPosition(SJC_AUX_JOINT_TH_Y, &fAux);
	compose_ui16_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_AUX_SWING_POSITION_THY] = w[0];
	usRegInputBuf[INPUT_REG_AUX_SWING_POSITION_THY + 1] = w[1];

	// Actuator velocity
	fAux = 0.0f;
	SJC_getJointAngularVelocity(SJC_ACTUATOR_JOINT, &fAux);
	compose_ui16_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_ACTUATOR_VELOCITY] = w[0];
	usRegInputBuf[INPUT_REG_ACTUATOR_VELOCITY + 1] = w[1];

	// Swing velocities
	fAux = 0.0f;
	SJC_getJointAngularVelocity(SJC_AUX_JOINT_TH_X, &fAux);
	compose_ui16_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_SWING_VELOCITY_THX] = w[0];
	usRegInputBuf[INPUT_REG_SWING_VELOCITY_THX + 1] = w[1];

	fAux = 0.0f;
	SJC_getJointAngularVelocity(SJC_AUX_JOINT_TH_Y, &fAux);
	compose_ui16_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_SWING_VELOCITY_THY] = w[0];
	usRegInputBuf[INPUT_REG_SWING_VELOCITY_THY + 1] = w[1];
}

static void Net_UpdateActuatorParametersRegisters(void)
{
	float fAux;
	uint16_t w[2];

	// Actuator output state
	bool state = false;
	SJC_getJointActuatorOutputState(&state);
	usRegInputBuf[INPUT_REG_ACTUATOR_STATE] = (uint16_t)(state ? 1u : 0u);

	// System voltage
	fAux = 0.0f;
	SJC_getJointActuatorPowerSupplyVoltage(&fAux);
	compose_ui16_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_SYSTEM_VOLTAGE] = w[0];
	usRegInputBuf[INPUT_REG_SYSTEM_VOLTAGE + 1] = w[1];

	// Actuator base voltage
	fAux = 0.0f;
	MotorDrive_getBaseVoltage(&fAux);
	compose_ui16_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_ACTUATOR_BASE_VOLTAGE] = w[0];
	usRegInputBuf[INPUT_REG_ACTUATOR_BASE_VOLTAGE + 1] = w[1];

	/* Reserved for future implementation
	 TODO: Actuator limits (voltage, speed, current...)
	 */
}

static void Net_UpdatePidControllerRegisters(void)
{
	xSjcPidParameters_t p = PidController_get_parameters();

	uint16_t w[2];

	compose_ui16_array_from_float(p.Kp, w);
	usRegInputBuf[INPUT_REG_PID_CONTROLLER_PARAMS_KP] = w[0];
	usRegInputBuf[INPUT_REG_PID_CONTROLLER_PARAMS_KP + 1] = w[1];

	compose_ui16_array_from_float(p.Ki, w);
	usRegInputBuf[INPUT_REG_PID_CONTROLLER_PARAMS_KI] = w[0];
	usRegInputBuf[INPUT_REG_PID_CONTROLLER_PARAMS_KI + 1] = w[1];

	compose_ui16_array_from_float(p.Kd, w);
	usRegInputBuf[INPUT_REG_PID_CONTROLLER_PARAMS_KD] = w[0];
	usRegInputBuf[INPUT_REG_PID_CONTROLLER_PARAMS_KD + 1] = w[1];

	compose_ui16_array_from_float(p.K_aw, w);
	usRegInputBuf[INPUT_REG_PID_CONTROLLER_PARAMS_K_AW] = w[0];
	usRegInputBuf[INPUT_REG_PID_CONTROLLER_PARAMS_K_AW + 1] = w[1];

	compose_ui16_array_from_float(p.u_min, w);
	usRegInputBuf[INPUT_REG_PID_CONTROLLER_PARAMS_U_MIN] = w[0];
	usRegInputBuf[INPUT_REG_PID_CONTROLLER_PARAMS_U_MIN + 1] = w[1];

	compose_ui16_array_from_float(p.u_max, w);
	usRegInputBuf[INPUT_REG_PID_CONTROLLER_PARAMS_U_MAX] = w[0];
	usRegInputBuf[INPUT_REG_PID_CONTROLLER_PARAMS_U_MAX + 1] = w[1];
}

static void Net_UpdateLQRControllerRegisters(void)
{
	xSjcLQRControllerParameters_t ctrl;
	xSjcLQRObserverParameters_t obs;

	LQRController_get_parameters(&ctrl, &obs);

	uint16_t w[2];

	// Configs
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_ORDER] = ctrl.order;
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_USE_OBS] = ctrl.use_observer;

	// ---- Controller gains ----
	compose_ui16_array_from_float(ctrl.Kx[0], w);
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_KX0] = w[0];
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_KX0 + 1] = w[1];

	compose_ui16_array_from_float(ctrl.Kx[1], w);
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_KX1] = w[0];
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_KX1 + 1] = w[1];

	compose_ui16_array_from_float(ctrl.Ki, w);
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_KI] = w[0];
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_KI + 1] = w[1];

	compose_ui16_array_from_float(ctrl.Kw, w);
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_KW] = w[0];
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_KW + 1] = w[1];

	compose_ui16_array_from_float(ctrl.cutoff_freq, w);
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_CUTOFF_FREQ] = w[0];
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_CUTOFF_FREQ + 1] = w[1];

	compose_ui16_array_from_float(ctrl.u_min, w);
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_U_MIN] = w[0];
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_U_MIN + 1] = w[1];

	compose_ui16_array_from_float(ctrl.u_max, w);
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_U_MAX] = w[0];
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_U_MAX + 1] = w[1];

	// ---- Discrete model ----
	compose_ui16_array_from_float(ctrl.Ad[0][0], w);
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_AD00] = w[0];
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_AD00 + 1] = w[1];

	compose_ui16_array_from_float(ctrl.Ad[0][1], w);
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_AD01] = w[0];
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_AD01 + 1] = w[1];

	compose_ui16_array_from_float(ctrl.Ad[1][0], w);
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_AD10] = w[0];
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_AD10 + 1] = w[1];

	compose_ui16_array_from_float(ctrl.Ad[1][1], w);
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_AD11] = w[0];
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_AD11 + 1] = w[1];

	compose_ui16_array_from_float(ctrl.Bd[0], w);
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_BD0] = w[0];
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_BD0 + 1] = w[1];

	compose_ui16_array_from_float(ctrl.Bd[1], w);
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_BD1] = w[0];
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_BD1 + 1] = w[1];

	compose_ui16_array_from_float(ctrl.Cd[0], w);
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_CD0] = w[0];
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_CD0 + 1] = w[1];

	compose_ui16_array_from_float(ctrl.Cd[1], w);
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_CD1] = w[0];
	usRegInputBuf[INPUT_REG_LQR_CONTROLLER_PARAMS_CD1 + 1] = w[1];

	// ---- Observer ----
	compose_ui16_array_from_float(obs.Ao[0][0], w);
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_AO00] = w[0];
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_AO00 + 1] = w[1];

	compose_ui16_array_from_float(obs.Ao[0][1], w);
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_AO01] = w[0];
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_AO01 + 1] = w[1];

	compose_ui16_array_from_float(obs.Ao[1][0], w);
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_AO10] = w[0];
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_AO10 + 1] = w[1];

	compose_ui16_array_from_float(obs.Ao[1][1], w);
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_AO11] = w[0];
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_AO11 + 1] = w[1];

	compose_ui16_array_from_float(obs.Bo[0], w);
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_BO0] = w[0];
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_BO0 + 1] = w[1];

	compose_ui16_array_from_float(obs.Bo[1], w);
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_BO1] = w[0];
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_BO1 + 1] = w[1];

	compose_ui16_array_from_float(obs.Co[0], w);
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_CO0] = w[0];
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_CO0 + 1] = w[1];

	compose_ui16_array_from_float(obs.Co[1], w);
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_CO1] = w[0];
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_CO1 + 1] = w[1];

	compose_ui16_array_from_float(obs.L[0], w);
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_L0] = w[0];
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_L0 + 1] = w[1];

	compose_ui16_array_from_float(obs.L[1], w);
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_L1] = w[0];
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_L1 + 1] = w[1];

	compose_ui16_array_from_float(obs.obs_cutoff_freq, w);
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_CUTOFF_FREQ] = w[0];
	usRegInputBuf[INPUT_REG_LQR_OBSERVER_PARAMS_CUTOFF_FREQ + 1] = w[1];
}

static void Net_UpdateDigitalControllerRegisters(void)
{
	xSjcDiscreteControllerParameters_t p;
	DiscreteController_get_parameters(&p);

	uint16_t w[2];

	// ---- Denominator (A) ----
	compose_ui16_array_from_float(p.A[0], w);
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_A0] = w[0];
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_A0 + 1] = w[1];

	compose_ui16_array_from_float(p.A[1], w);
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_A1] = w[0];
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_A1 + 1] = w[1];

	compose_ui16_array_from_float(p.A[2], w);
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_A2] = w[0];
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_A2 + 1] = w[1];

	// ---- Numerator (B) ----
	compose_ui16_array_from_float(p.B[0], w);
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_B0] = w[0];
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_B0 + 1] = w[1];

	compose_ui16_array_from_float(p.B[1], w);
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_B1] = w[0];
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_B1 + 1] = w[1];

	compose_ui16_array_from_float(p.B[2], w);
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_B2] = w[0];
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_B2 + 1] = w[1];

	// ---- Limits ----
	compose_ui16_array_from_float(p.u_min, w);
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_U_MIN] = w[0];
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_U_MIN + 1] = w[1];

	compose_ui16_array_from_float(p.u_max, w);
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_U_MAX] = w[0];
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_U_MAX + 1] = w[1];

	// ---- Anti-windup ----
	compose_ui16_array_from_float(p.K_aw, w);
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_K_AW] = w[0];
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_K_AW + 1] = w[1];

	// ---- Cutoff frequency ----
	compose_ui16_array_from_float(p.cutoff_freq, w);
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_CFREQ] = w[0];
	usRegHoldingBuf[INPUT_REG_DIGITAL_CONTROLLER_PARAMS_CFREQ + 1] = w[1];
}

static void Net_UpdateSensorsParametersRegisters(void)
{
	uint16_t aux;

	Encoder_getResolution(SJC_ACTUATOR_JOINT, &aux);
	usRegInputBuf[INPUT_REG_ACTUATOR_ENCODER_RESOLUTION_CPR] = aux;

	Encoder_getResolution(SJC_AUX_JOINT_TH_X, &aux);
	usRegInputBuf[INPUT_REG_SWING_THX_ENCODER_RESOLUTION_CPR] = aux;

	Encoder_getResolution(SJC_AUX_JOINT_TH_Y, &aux);
	usRegInputBuf[INPUT_REG_SWING_THY_ENCODER_RESOLUTION_CPR] = aux;
}

// Holding registers update process ---------------------------------- //

static void Net_ExecuteSystemConfigsUpdateCmd(void)
{
	if (usRegHoldingBuf[HOLDING_REG_SYSTEM_COMMANDS] & SYS_CMD_REBOOT)
	{

		// Define digital outputs states
		usRegHoldingBuf[HOLDING_REG_SYSTEM_COMMANDS] &= (~SYS_CMD_REBOOT);

		// Apply system reboot
		SJC_SYSTEM_RESET();
	}

	if (usRegHoldingBuf[HOLDING_REG_SYSTEM_COMMANDS] & SYS_CMD_RESET_PRIMARY_ENCODER_COUNTER)
	{
		usRegHoldingBuf[HOLDING_REG_SYSTEM_COMMANDS] &= (~SYS_CMD_RESET_PRIMARY_ENCODER_COUNTER);

		SJC_resetJointAngularPosition();
	}

	if (usRegHoldingBuf[HOLDING_REG_SYSTEM_COMMANDS] & SYS_CMD_RESET_AUX_ENCODER_COUNTER)
	{
		usRegHoldingBuf[HOLDING_REG_SYSTEM_COMMANDS] &= (~SYS_CMD_RESET_AUX_ENCODER_COUNTER);

		SJC_resetAuxJointsAngularPosition();
	}

	if (usRegHoldingBuf[HOLDING_REG_SYSTEM_COMMANDS] & SYS_CMD_RESET_PRIMARY_JOINT_VELOCITY)
	{
		usRegHoldingBuf[HOLDING_REG_SYSTEM_COMMANDS] &= (~SYS_CMD_RESET_PRIMARY_JOINT_VELOCITY);

		SJC_resetJointVelocity(SJC_ACTUATOR_JOINT);
	}

	if (usRegHoldingBuf[HOLDING_REG_SYSTEM_COMMANDS] & SYS_CMD_RESET_AUX_JOINT_VELOCITY)
	{
		usRegHoldingBuf[HOLDING_REG_SYSTEM_COMMANDS] &= (~SYS_CMD_RESET_AUX_JOINT_VELOCITY);

		SJC_resetJointVelocity(SJC_AUX_JOINT_TH_X);
		SJC_resetJointVelocity(SJC_AUX_JOINT_TH_Y);
	}

	if (usRegHoldingBuf[HOLDING_REG_SYSTEM_COMMANDS] & SYS_CMD_RESET_CONTROLLER)
	{
		usRegHoldingBuf[HOLDING_REG_SYSTEM_COMMANDS] &= ~SYS_CMD_RESET_CONTROLLER;

		SJC_resetController();
	}
}

static void Net_ExecuteDigitalOutputsUpdateCmd(void)
{
	if (usRegHoldingBuf[HOLDING_REG_DIGITAL_OUTPUTS_STATE_UPDATE] & HOLDING_REG_APPLY_MSK)
	{

		// Define digital outputs states
		usRegHoldingBuf[HOLDING_REG_DIGITAL_OUTPUTS_STATE_UPDATE] &= (~HOLDING_REG_APPLY_MSK);

		SJC_writeDigitalOutputs(usRegHoldingBuf[HOLDING_REG_DIGITAL_OUTPUTS_STATE_UPDATE]);
	}
}

static void Net_ExecuteControllerModeUpdateCmd(void)
{
	if (usRegHoldingBuf[HOLDING_REG_CONTROL_MODE_UPDATE] & HOLDING_REG_APPLY_MSK)
	{
		eSjcControlMode_t control_mode;

		// Clear update mask
		usRegHoldingBuf[HOLDING_REG_CONTROL_MODE_UPDATE] &= (~HOLDING_REG_APPLY_MSK);

		// Controller config
		control_mode = (eSjcControlMode_t)usRegHoldingBuf[HOLDING_REG_CONTROL_MODE];
		SJC_setControlMode(control_mode);

#ifdef SJC_DEBUG_PRINT_NET_CMDS
		SJC_getControlMode(&control_mode);

		switch (control_mode)
		{
		case SJC_CONTROL_DISABLED:
			DebugOutput_systemPrintf("\r\n[NetCmd] Control disabled (%u)", (unsigned int)control_mode);
			break;

		case SJC_OPEN_LOOP_CONTROL:
			DebugOutput_systemPrintf("\r\n[NetCmd] Open-loop control mode (%u)", (unsigned int)control_mode);
			break;

		case SJC_PID_SPEED_CONTROL:
			DebugOutput_systemPrintf("\r\n[NetCmd] PID speed control mode (%u)", (unsigned int)control_mode);
			break;

		case SJC_PID_CURRENT_CONTROL:
			DebugOutput_systemPrintf("\r\n[NetCmd] PID current control mode (%u)", (unsigned int)control_mode);
			break;

		case SJC_LQR_CURRENT_CONTROL:
			DebugOutput_systemPrintf("\r\n[NetCmd] LQR current control mode (%u)", (unsigned int)control_mode);
			break;

		case SJC_DISCRETE_CURRENT_CONTROL:
			DebugOutput_systemPrintf("\r\n[NetCmd] Discrete current control mode (%u)", (unsigned int)control_mode);
			break;

		case SJC_SLOW_PRBS_EXCITATION:
			DebugOutput_systemPrintf("\r\n[NetCmd] PRBS excitation mode - Slow (5 Hz) (%u)", (unsigned int)control_mode);
			break;

		case SJC_MID_PRBS_EXCITATION:
			DebugOutput_systemPrintf("\r\n[NetCmd] PRBS excitation mode - Mid (10 Hz) (%u)", (unsigned int)control_mode);
			break;

		case SJC_FAST_PRBS_EXCITATION:
			DebugOutput_systemPrintf("\r\n[NetCmd] PRBS excitation mode - Fast (15 Hz) (%u)", (unsigned int)control_mode);
			break;

		case SJC_SINE_PRBS_EXCITATION:
			DebugOutput_systemPrintf("\r\n[NetCmd] Sine+PRBS excitation mode (sine 1, 2.5, 5 Hz + PRBS 20 Hz) (%u)", (unsigned int)control_mode);
			break;

		case SJC_PULSE_FAST_EXCITATION:
			DebugOutput_systemPrintf("\r\n[NetCmd] Pulse fast excitation mode (4 Hz) (%u)", (unsigned int)control_mode);
			break;

		case SJC_PULSE_SLOW_EXCITATION:
			DebugOutput_systemPrintf("\r\n[NetCmd] Pulse slow excitation mode (2 Hz) (%u)", (unsigned int)control_mode);
			break;

		case SJC_ANALOG_CALIBRATION:
			DebugOutput_systemPrintf("\r\n[NetCmd] Current sense analog calibration (%u)", (unsigned int)control_mode);
			break;

		default:
			DebugOutput_systemPrintf("\r\n[NetCmd] Unknown control mode! (%u)", (unsigned int)control_mode);
			break;
		}
#endif

		// Reset controller after config update
		SJC_resetController();
	}
}

static void Net_ExecuteControllerFrequencyUpdateCmd(void)
{
	if (usRegHoldingBuf[HOLDING_REG_CONTROL_FREQUENCY_UPDATE] & HOLDING_REG_APPLY_MSK)
	{
		uint16_t w[2];
		float fAux;

		// Clear update mask
		usRegHoldingBuf[HOLDING_REG_CONTROL_FREQUENCY_UPDATE] &= (~HOLDING_REG_APPLY_MSK);

		// Controller frequency config
		w[0] = usRegHoldingBuf[HOLDING_REG_CONTROL_FREQUENCY];
		w[1] = usRegHoldingBuf[HOLDING_REG_CONTROL_FREQUENCY + 1];
		compose_float_from_ui16_array(w, &fAux);
		SJC_setControlUpdateRateHz(fAux);

#ifdef SJC_DEBUG_PRINT_NET_CMDS
		DebugOutput_systemPrintf("\r\n[NetCmd] New control update rate: %.3f Hz", fAux);
#endif
		// Reset controller after config update
		SJC_resetController();
	}
}

static void Net_ExecuteControllerRefUpdateCmd(void)
{
	if (usRegHoldingBuf[HOLDING_REG_CONTROLLER_REFERENCE_UPDATE] & HOLDING_REG_APPLY_MSK)
	{
		usRegHoldingBuf[HOLDING_REG_CONTROLLER_REFERENCE_UPDATE] &= (~HOLDING_REG_APPLY_MSK);

		uint16_t w[2];
		w[0] = usRegHoldingBuf[HOLDING_REG_CONTROLLER_REFERENCE_VALUE];
		w[1] = usRegHoldingBuf[HOLDING_REG_CONTROLLER_REFERENCE_VALUE + 1];

		float reference = 0.0f;
		compose_float_from_ui16_array(w, &reference);
		SJC_setControlReference(reference);

#ifdef SJC_DEBUG_PRINT_NET_CMDS
		DebugOutput_systemPrintf("\r\n[NetCmd] New control reference: %.3f\r\n", reference);
#endif
	}
}

static void Net_ExecuteActuatorConfigsUpdateCmd(void)
{
	if (usRegHoldingBuf[HOLDING_REG_ACTUATOR_SETUP_UPDATE] & HOLDING_REG_APPLY_MSK)
	{
		uint16_t w[2];
		float fAux = 0.0f;

		usRegHoldingBuf[HOLDING_REG_ACTUATOR_SETUP_UPDATE] &= (~HOLDING_REG_APPLY_MSK);

		w[0] = usRegHoldingBuf[HOLDING_REG_ACTUATOR_BASE_VOLTAGE];
		w[1] = usRegHoldingBuf[HOLDING_REG_ACTUATOR_BASE_VOLTAGE + 1];
		compose_float_from_ui16_array(w, &fAux);
		MotorDrive_setBaseVoltage(fAux);

		//		w[0] = usRegHoldingBuf[HOLDING_REG_ACTUATOR_LIMITS_VOLTAGE];
		//		w[1] = usRegHoldingBuf[HOLDING_REG_ACTUATOR_LIMITS_VOLTAGE + 1];
		//		compose_float_from_ui16_array(w, &fAux);
		//		xSjcRange_t EffortLimits = {.Min = -fAux, .Max = fAux};
		//		PidController_setActuatorEffortLimits(EffortLimits);

		/* Reserved for future implementation
		 *
		 * float current_limit = (1.0f*usRegHoldingBuf[HOLDING_REG_ACTUATOR_LIMITS_CURRENT]/100.0f);
		 * float voltage_limit = (1.0f*usRegHoldingBuf[HOLDING_REG_ACTUATOR_LIMITS_VOLTAGE]/100.0f);
		 * float speed_limit;

		 * uint16_t *ptr = (uint16_t*) (&speed_limit);
		 * ptr[0] = usRegHoldingBuf[HOLDING_REG_ACTUATOR_LIMITS_SPEED + 1];
		 * ptr[1] = usRegHoldingBuf[HOLDING_REG_ACTUATOR_LIMITS_SPEED];
		 */
	}
}

static void Net_ExecuteNetworkConfigsUpdateCmd(void)
{
	if (usRegHoldingBuf[HOLDING_REG_NETWORK_CONFIG_UPDATE] & HOLDING_REG_APPLY_MSK)
	{
		usRegHoldingBuf[HOLDING_REG_NETWORK_CONFIG_UPDATE] &= (~HOLDING_REG_APPLY_MSK);

		xSjcNetworkConfig_t config;
		uint16_t w[2];

		w[0] = usRegHoldingBuf[HOLDING_REG_NETWORK_CONFIG_BAUD_RATE];
		w[1] = usRegHoldingBuf[HOLDING_REG_NETWORK_CONFIG_BAUD_RATE + 1];
		compose_ui32_from_ui16_array(w, &config.Baudrate);

		config.Parity = (char)usRegHoldingBuf[HOLDING_REG_NETWORK_CONFIG_PARITY];
		config.Address = usRegHoldingBuf[HOLDING_REG_NETWORK_CONFIG_ADDRESS];

#ifdef SJC_DEBUG_PRINT_NET_CMDS
		DebugOutput_printf("\r\n[NetCmd] Network address: %u", config.Address);
		DebugOutput_printf("\r\n[NetCmd] Network parity: %c", config.Parity);
		DebugOutput_printf("\r\n[NetCmd] Network baudrate: %lu bps", (unsigned long)config.Baudrate);
#endif

		// TODO: persist to flash + reboot se necessário
	}
}

static void Net_ExecuteSensorsConfigsUpdateCmd(void)
{
	if (usRegHoldingBuf[HOLDING_REG_SENSORS_SETUP_UPDATE] & HOLDING_REG_APPLY_MSK)
	{
		// Encoders parameters
		usRegHoldingBuf[HOLDING_REG_SENSORS_SETUP_UPDATE] &= (~HOLDING_REG_APPLY_MSK);

		Encoder_setResolution(SJC_AUX_JOINT_TH_X, usRegHoldingBuf[HOLDING_REG_SWING_THX_ENCODER_RESOLUTION_CPR]);
		Encoder_setResolution(SJC_AUX_JOINT_TH_Y, usRegHoldingBuf[HOLDING_REG_SWING_THY_ENCODER_RESOLUTION_CPR]);
		Encoder_setResolution(SJC_ACTUATOR_JOINT, usRegHoldingBuf[HOLDING_REG_ACTUATOR_ENCODER_RESOLUTION_CPR]);
	}
}

static void Net_ExecutePIDControllerParametersUpdateCmd(void)
{
	if (usRegHoldingBuf[HOLDING_REG_PID_CONTROLLER_UPDATE] & HOLDING_REG_APPLY_MSK)
	{
		usRegHoldingBuf[HOLDING_REG_PID_CONTROLLER_UPDATE] &= (~HOLDING_REG_APPLY_MSK);

		xSjcPidParameters_t params;
		uint16_t w[2];

		w[0] = usRegHoldingBuf[HOLDING_REG_PID_CONTROLLER_PARAMS_KP];
		w[1] = usRegHoldingBuf[HOLDING_REG_PID_CONTROLLER_PARAMS_KP + 1];
		compose_float_from_ui16_array(w, &params.Kp);

		w[0] = usRegHoldingBuf[HOLDING_REG_PID_CONTROLLER_PARAMS_KI];
		w[1] = usRegHoldingBuf[HOLDING_REG_PID_CONTROLLER_PARAMS_KI + 1];
		compose_float_from_ui16_array(w, &params.Ki);

		w[0] = usRegHoldingBuf[HOLDING_REG_PID_CONTROLLER_PARAMS_KD];
		w[1] = usRegHoldingBuf[HOLDING_REG_PID_CONTROLLER_PARAMS_KD + 1];
		compose_float_from_ui16_array(w, &params.Kd);

		w[0] = usRegHoldingBuf[HOLDING_REG_PID_CONTROLLER_PARAMS_K_AW];
		w[1] = usRegHoldingBuf[HOLDING_REG_PID_CONTROLLER_PARAMS_K_AW + 1];
		compose_float_from_ui16_array(w, &params.K_aw);

		w[0] = usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_U_MIN];
		w[1] = usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_U_MIN + 1];
		compose_float_from_ui16_array(w, &params.u_min);

		w[0] = usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_U_MAX];
		w[1] = usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_U_MAX + 1];
		compose_float_from_ui16_array(w, &params.u_max);

		params.configured = true;

		PidController_set_parameters(params);
		PidController_reset();

#ifdef SJC_DEBUG_PRINT_NET_CMDS
		DebugOutput_printf("\r\n[NetCmd] Controller Kp: %f", params.Kp);
		DebugOutput_printf("\r\n[NetCmd] Controller Ki: %f", params.Ki);
		DebugOutput_printf("\r\n[NetCmd] Controller Kd: %f", params.Kd);
		DebugOutput_printf("\r\n[NetCmd] Controller K_aw: %f", params.K_aw);
		DebugOutput_printf("\r\n[NetCmd] Controller U_min: %f", params.u_min);
		DebugOutput_printf("\r\n[NetCmd] Controller U_max: %f", params.u_max);
#endif
	}
}

static void Net_ExecuteLQRControllerParametersUpdateCmd(void)
{
	if (usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_UPDATE] & HOLDING_REG_APPLY_MSK)
	{
		usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_UPDATE] &= ~HOLDING_REG_APPLY_MSK;

		xSjcLQRControllerParameters_t ctrl = xSjcLQRControllerParametersEmptyObj;
		xSjcLQRObserverParameters_t obs = xSjcLQRObserverParametersEmptyObj;

		// ---- Controller configs ----
		ctrl.order = usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_ORDER];
		ctrl.use_observer = usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_USE_OBS];

		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_KX0], &ctrl.Kx[0]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_KX1], &ctrl.Kx[1]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_KI], &ctrl.Ki);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_KW], &ctrl.Kw);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_CUTOFF_FREQ], &ctrl.cutoff_freq);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_U_MIN], &ctrl.u_min);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_U_MAX], &ctrl.u_max);

		// ---- Discrete model (Ad, Bd, Cd) ----
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_AD00], &ctrl.Ad[0][0]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_AD01], &ctrl.Ad[0][1]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_AD10], &ctrl.Ad[1][0]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_AD11], &ctrl.Ad[1][1]);

		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_BD0], &ctrl.Bd[0]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_BD1], &ctrl.Bd[1]);

		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_CD0], &ctrl.Cd[0]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_CONTROLLER_PARAMS_CD1], &ctrl.Cd[1]);

		// ---- Observer matrices ----
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_OBSERVER_PARAMS_AO00], &obs.Ao[0][0]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_OBSERVER_PARAMS_AO01], &obs.Ao[0][1]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_OBSERVER_PARAMS_AO10], &obs.Ao[1][0]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_OBSERVER_PARAMS_AO11], &obs.Ao[1][1]);

		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_OBSERVER_PARAMS_BO0], &obs.Bo[0]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_OBSERVER_PARAMS_BO1], &obs.Bo[1]);

		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_OBSERVER_PARAMS_CO0], &obs.Co[0]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_OBSERVER_PARAMS_CO1], &obs.Co[1]);

		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_OBSERVER_PARAMS_L0], &obs.L[0]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_OBSERVER_PARAMS_L1], &obs.L[1]);

		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_LQR_OBSERVER_PARAMS_CUTOFF_FREQ], &obs.obs_cutoff_freq);

		// ---- Aplicar ----
		if (ctrl.order < 2)
		{
			ctrl.use_observer = false;
		}

		ctrl.configured = true;

		LQRController_set_parameters(ctrl, obs);
		LQRController_reset();

#ifdef SJC_DEBUG_PRINT_NET_CMDS
		DebugOutput_printf("\r\n[NetCmd][LQR] Order: %u", ctrl.order);
		DebugOutput_printf("\r\n[NetCmd][LQR] Use Observer: %u", ctrl.use_observer);

		// ---- Gains ----
		DebugOutput_printf("\r\n[NetCmd][LQR] Kx0: %f", ctrl.Kx[0]);
		DebugOutput_printf("\r\n[NetCmd][LQR] Kx1: %f", ctrl.Kx[1]);
		DebugOutput_printf("\r\n[NetCmd][LQR] Ki: %f", ctrl.Ki);
		DebugOutput_printf("\r\n[NetCmd][LQR] Kw: %f", ctrl.Kw);
		DebugOutput_printf("\r\n[NetCmd][LQR] Cutoff: %f", ctrl.cutoff_freq);
		DebugOutput_printf("\r\n[NetCmd][LQR] U_min: %f", ctrl.u_min);
		DebugOutput_printf("\r\n[NetCmd][LQR] U_max: %f", ctrl.u_max);

		// ---- Discrete Model ----
		DebugOutput_printf("\r\n[NetCmd][LQR] Ad = [[%.4f, %.4f], [%.4f, %.4f]]",
						   ctrl.Ad[0][0], ctrl.Ad[0][1],
						   ctrl.Ad[1][0], ctrl.Ad[1][1]);
		DebugOutput_printf("\r\n[NetCmd][LQR] Bd = [%.4f, %.4f]",
						   ctrl.Bd[0], ctrl.Bd[1]);
		DebugOutput_printf("\r\n[NetCmd][LQR] Cd = [%.4f, %.4f]",
						   ctrl.Cd[0], ctrl.Cd[1]);

		// ---- Observer ----
		DebugOutput_printf("\r\n[NetCmd][LQR] Ao = [[%.4f, %.4f], [%.4f, %.4f]]",
						   obs.Ao[0][0], obs.Ao[0][1],
						   obs.Ao[1][0], obs.Ao[1][1]);
		DebugOutput_printf("\r\n[NetCmd][LQR] Bo = [%.4f, %.4f]",
						   obs.Bo[0], obs.Bo[1]);
		DebugOutput_printf("\r\n[NetCmd][LQR] Co = [%.4f, %.4f]",
						   obs.Co[0], obs.Co[1]);
		DebugOutput_printf("\r\n[NetCmd][LQR] L  = [%.4f, %.4f]",
						   obs.L[0], obs.L[1]);
		DebugOutput_printf("\r\n[NetCmd][LQR] Obs Cutoff: %f",
						   obs.obs_cutoff_freq);
#endif
	}
}

static void Net_ExecuteDigitalControllerParametersUpdateCmd(void)
{
	if (usRegHoldingBuf[HOLDING_REG_DIGITAL_CONTROLLER_UPDATE] & HOLDING_REG_APPLY_MSK)
	{
		usRegHoldingBuf[HOLDING_REG_DIGITAL_CONTROLLER_UPDATE] &= ~HOLDING_REG_APPLY_MSK;

		xSjcDiscreteControllerParameters_t params = {0};

		// --- Denominator A ---
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_A0], &params.A[0]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_A1], &params.A[1]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_A2], &params.A[2]);

		// --- Numerator B ---
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_B0], &params.B[0]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_B1], &params.B[1]);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_B2], &params.B[2]);

		// --- Limits and gains ---
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_U_MIN], &params.u_min);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_U_MAX], &params.u_max);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_K_AW], &params.K_aw);
		compose_float_from_ui16_array(&usRegHoldingBuf[HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_CFREQ], &params.cutoff_freq);

		params.configured = true;

		// --- Apply parameters ---
		DiscreteController_set_parameters(params);
		DiscreteController_reset();

#ifdef SJC_DEBUG_PRINT_NET_CMDS
		// --- Denominator A ---
		DebugOutput_printf("\r\n[NetCmd][DIG] A = [%.6f, %.6f, %.6f]", params.A[0], params.A[1], params.A[2]);

		// --- Numerator B ---
		DebugOutput_printf("\r\n[NetCmd][DIG] B = [%.6f, %.6f, %.6f]", params.B[0], params.B[1], params.B[2]);

		// --- Limits ---
		DebugOutput_printf("\r\n[NetCmd][DIG] u_min = %.6f, u_max = %.6f", params.u_min, params.u_max);

		// --- Gains ---
		DebugOutput_printf("\r\n[NetCmd][DIG] K_aw = %.6f, cutoff = %.6f", params.K_aw, params.cutoff_freq);
#endif
	}
}

// ------------------------------------------------------------------- //
