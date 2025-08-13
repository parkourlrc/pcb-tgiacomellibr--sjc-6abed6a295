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
#include "modules/include/modbus_network.h"
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
#include "modules/include/encoder.h"
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
static void Net_UpdateControlMeasInfoRegisters(void);
static void Net_UpdateDigitalIOInfoRegisters(void);
static void Net_UpdateSensorsParametersRegisters(void);
static void Net_UpdateActuatorParametersRegisters(void);
static void Net_UpdateControllerParametersRegisters(void);

// Holding registers update process ---------------------------------- //
static void Net_ExecuteSystemConfigsUpdateCmd(void);
static void Net_ExecuteNetworkConfigsUpdateCmd(void);
static void Net_ExecuteSensorsConfigsUpdateCmd(void);
static void Net_ExecuteActuatorConfigsUpdateCmd(void);
static void Net_ExecuteControllerModeUpdateCmd(void);
static void Net_ExecuteControllerRefUpdateCmd(void);
static void Net_ExecuteDigitalOutputsUpdateCmd(void);
static void Net_ExecuteControllerParametersUpdateCmd(void);
static void Net_ExecuteAdcFilterModeUpdateCmd(void);

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
	MX_ADC1_Init();
	MX_TIM4_Init();
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
	xSjcControlConfigs.Rate = SJC_CONTROLLER_MAX_UPDATE_RATE_HZ;

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

	usRegInputBuf[INPUT_REG_FW_VERSION] = SJC_FW_VERSION_MAJOR << 8 | SJC_FW_VERSION_MINOR;
	usRegInputBuf[INPUT_REG_FW_VERSION + 1] = SJC_FW_VERSION_PATCH;

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
		return SJC_RET_NOK;

	*config = xSjcControlConfigs.Rate;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_setControlUpdateRateHz(const float config)
{
	xSjcControlConfigs.Rate = SJC_Helpers_RealClamp(config, SJC_CONTROLLER_MIN_UPDATE_RATE_HZ, SJC_CONTROLLER_MAX_UPDATE_RATE_HZ);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getControlUpdatePeriodMs(uint32_t *period_ms)
{
	if (!period_ms || xSjcControlConfigs.Rate <= 0.0f)
		return SJC_RET_NOK;
	*period_ms = ((uint32_t)(1000.0f / xSjcControlConfigs.Rate));
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_setControlUpdatePeriodMs(const uint32_t period_ms)
{
	if (period_ms == 0)
		return SJC_RET_NOK;
	float clamped = (float)SJC_Helpers_UintClamp(period_ms, SJC_CONTROLLER_MIN_UPDATE_PERIOD_MS, SJC_CONTROLLER_MAX_UPDATE_PERIOD_MS);
	xSjcControlConfigs.Rate = 1000.0f / clamped;
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getControlReference(float *reference)
{
	if (!reference)
	{
		return SJC_RET_NOK;
	}

	*reference = PidController_getReference();
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_setControlReference(const float reference)
{
	PidController_setReference(reference);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getControlError(float *controller_error)
{
	if (!controller_error)
	{
		return SJC_RET_NOK;
	}

	*controller_error = PidController_getError();
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getControlParameters(xSjcServoPidParameters_t *Parameters)
{
	if (!Parameters)
	{
		return SJC_RET_NOK;
	}

	*Parameters = PidController_getControlParameters();
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_setControlParameters(const xSjcServoPidParameters_t Parameters)
{
	PidController_setControlParameters(Parameters);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_computeControlAction(const float ControlError, const float dt, float *ControlAction)
{
	if (!ControlAction)
	{
		return SJC_RET_NOK;
	}

	*ControlAction = PidController_compute(ControlError, dt);
	return SJC_RET_OK;
}

void SJC_updateHoldingRegisters(void)
{
	Net_ExecuteSystemConfigsUpdateCmd();
	Net_ExecuteDigitalOutputsUpdateCmd();
	Net_ExecuteNetworkConfigsUpdateCmd();
	Net_ExecuteSensorsConfigsUpdateCmd();
	Net_ExecuteActuatorConfigsUpdateCmd();
	Net_ExecuteControllerModeUpdateCmd();
	Net_ExecuteControllerRefUpdateCmd();
	Net_ExecuteControllerParametersUpdateCmd();
	Net_ExecuteAdcFilterModeUpdateCmd();
}

void SJC_updateInputRegisters(void)
{
	Net_UpdateBoardInfoRegisters();
	Net_UpdateControlMeasInfoRegisters();
	Net_UpdateDigitalIOInfoRegisters();
	Net_UpdateSensorsParametersRegisters();
	Net_UpdateActuatorParametersRegisters();
	Net_UpdateControllerParametersRegisters();
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

eSjcReturnType_t SJC_setAdcFilterMode(const eSjcAdcFilterMode_t mode, const float fc)
{
	AnalogInterfaces_setFilterMode(mode, fc);

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
		return SJC_RET_NOK;
	return Encoder_getAngularDisplacement(joint, disp);
}

eSjcReturnType_t SJC_getJointAngularDisplacement(const eSjcJoints_t joint, float *disp, const float prev_position)
{
	if (!disp || joint >= SJC_JOINTS_SIZE)
		return SJC_RET_NOK;
	eSjcReturnType_t ret = Encoder_getAngularDisplacement(joint, disp);
	if (ret == SJC_RET_OK)
		*disp -= prev_position;
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
		return SJC_RET_NOK;
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
	uint16_t addr;
	uint32_t baud;
	char parity;
	ModbusNetwork_getParameters(&addr, &baud, &parity);

	usRegInputBuf[INPUT_REG_NETWORK_ADDRESS] = addr;
	usRegInputBuf[INPUT_REG_NETWORK_PARITY] = (uint16_t)(uint8_t)parity;

	uint16_t w[2];
	compose_byte_array_from_ui32(baud, w);
	usRegInputBuf[INPUT_REG_NETWORK_BAUD_RATE] = w[0];
	usRegInputBuf[INPUT_REG_NETWORK_BAUD_RATE + 1] = w[1];

	// Control mode
	eSjcControlMode_t mode;
	SJC_getControlMode(&mode);
	usRegInputBuf[INPUT_REG_CONTROL_MODE] = (uint16_t)mode;

	// #TODO: Error flags
	usRegInputBuf[INPUT_REG_SYSTEM_ERROR_FLAGS] = 0x0000;
}

static void Net_UpdateControlMeasInfoRegisters(void)
{
	float fAux = 0.0f;
	uint16_t w[2];

	// Uptime (u32 -> 2x u16)
	uint32_t uptime = TimeoutTimer_getTimeMs();
	compose_byte_array_from_ui32(uptime, w);
	usRegInputBuf[INPUT_REG_SYSTEM_UPTIME] = w[0];
	usRegInputBuf[INPUT_REG_SYSTEM_UPTIME + 1] = w[1];

	// Control error (f32 -> 2x u16)
	fAux = 0.0f;
	SJC_getControlError(&fAux);
	compose_byte_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_CONTROL_ERROR] = w[0];
	usRegInputBuf[INPUT_REG_CONTROL_ERROR + 1] = w[1];

	// Control reference
	fAux = 0.0f;
	SJC_getControlReference(&fAux);
	compose_byte_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_CONTROL_REFERENCE] = w[0];
	usRegInputBuf[INPUT_REG_CONTROL_REFERENCE + 1] = w[1];

	// Actuator current (unbiased)
	fAux = 0.0f;
	SJC_getJointActuatorCurrentUnbiased(&fAux);
	compose_byte_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_ACTUATOR_CURRENT] = w[0];
	usRegInputBuf[INPUT_REG_ACTUATOR_CURRENT + 1] = w[1];

	// Actuator voltage
	fAux = 0.0f;
	SJC_getJointActuatorVoltage(&fAux);
	compose_byte_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_ACTUATOR_VOLTAGE] = w[0];
	usRegInputBuf[INPUT_REG_ACTUATOR_VOLTAGE + 1] = w[1];

	// Actuator position
	fAux = 0.0f;
	SJC_getJointAngularPosition(SJC_ACTUATOR_JOINT, &fAux);
	compose_byte_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_ACTUATOR_POSITION] = w[0];
	usRegInputBuf[INPUT_REG_ACTUATOR_POSITION + 1] = w[1];

	// Aux THX position
	fAux = 0.0f;
	SJC_getJointAngularPosition(SJC_AUX_JOINT_TH_X, &fAux);
	compose_byte_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_AUX_SWING_POSITION_THX] = w[0];
	usRegInputBuf[INPUT_REG_AUX_SWING_POSITION_THX + 1] = w[1];

	// Aux THY position
	fAux = 0.0f;
	SJC_getJointAngularPosition(SJC_AUX_JOINT_TH_Y, &fAux);
	compose_byte_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_AUX_SWING_POSITION_THY] = w[0];
	usRegInputBuf[INPUT_REG_AUX_SWING_POSITION_THY + 1] = w[1];

	// Actuator velocity
	fAux = 0.0f;
	SJC_getJointAngularVelocity(SJC_ACTUATOR_JOINT, &fAux);
	compose_byte_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_ACTUATOR_VELOCITY] = w[0];
	usRegInputBuf[INPUT_REG_ACTUATOR_VELOCITY + 1] = w[1];

	// Swing velocities
	fAux = 0.0f;
	SJC_getJointAngularVelocity(SJC_AUX_JOINT_TH_X, &fAux);
	compose_byte_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_SWING_VELOCITY_THX] = w[0];
	usRegInputBuf[INPUT_REG_SWING_VELOCITY_THX + 1] = w[1];

	fAux = 0.0f;
	SJC_getJointAngularVelocity(SJC_AUX_JOINT_TH_Y, &fAux);
	compose_byte_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_SWING_VELOCITY_THY] = w[0];
	usRegInputBuf[INPUT_REG_SWING_VELOCITY_THY + 1] = w[1];
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

static void Net_UpdateSensorsParametersRegisters(void)
{
	uint16_t aux;

	Encoder_getResolution(SJC_ACTUATOR_JOINT, &aux);
	usRegInputBuf[INPUT_REG_ACTUATOR_ENCODER_RESOLUTION_PPR] = aux;

	Encoder_getResolution(SJC_AUX_JOINT_TH_X, &aux);
	usRegInputBuf[INPUT_REG_SWING_THX_ENCODER_RESOLUTION_PPR] = aux;

	Encoder_getResolution(SJC_AUX_JOINT_TH_Y, &aux);
	usRegInputBuf[INPUT_REG_SWING_THY_ENCODER_RESOLUTION_PPR] = aux;
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
	compose_byte_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_SYSTEM_VOLTAGE] = w[0];
	usRegInputBuf[INPUT_REG_SYSTEM_VOLTAGE + 1] = w[1];

	// Actuator base voltage
	fAux = 0.0f;
	MotorDrive_getBaseVoltage(&fAux);
	compose_byte_array_from_float(fAux, w);
	usRegInputBuf[INPUT_REG_ACTUATOR_BASE_VOLTAGE] = w[0];
	usRegInputBuf[INPUT_REG_ACTUATOR_BASE_VOLTAGE + 1] = w[1];


	/* Reserved for future implementation
	 TODO: Actuator limits (voltage, speed, current...)
	 */
}

static void Net_UpdateControllerParametersRegisters(void)
{
	xSjcServoPidParameters_t p;
	SJC_getControlParameters(&p);

	uint16_t w[2];

	compose_byte_array_from_float(p.Kp, w);
	usRegInputBuf[INPUT_REG_CONTROLLER_PARAMS_KP] = w[0];
	usRegInputBuf[INPUT_REG_CONTROLLER_PARAMS_KP + 1] = w[1];

	compose_byte_array_from_float(p.Ki, w);
	usRegInputBuf[INPUT_REG_CONTROLLER_PARAMS_KI] = w[0];
	usRegInputBuf[INPUT_REG_CONTROLLER_PARAMS_KI + 1] = w[1];

	compose_byte_array_from_float(p.Kd, w);
	usRegInputBuf[INPUT_REG_CONTROLLER_PARAMS_KD] = w[0];
	usRegInputBuf[INPUT_REG_CONTROLLER_PARAMS_KD + 1] = w[1];

	compose_byte_array_from_float(p.KT, w);
	usRegInputBuf[INPUT_REG_CONTROLLER_PARAMS_KT] = w[0];
	usRegInputBuf[INPUT_REG_CONTROLLER_PARAMS_KT + 1] = w[1];

	compose_byte_array_from_float(p.KA, w);
	usRegInputBuf[INPUT_REG_CONTROLLER_PARAMS_KA] = w[0];
	usRegInputBuf[INPUT_REG_CONTROLLER_PARAMS_KA + 1] = w[1];

	float rate = 0.0f;
	SJC_getControlUpdateRateHz(&rate);
	compose_byte_array_from_float(rate, w);
	usRegInputBuf[INPUT_REG_CONTROLLER_PARAMS_RATE] = w[0];
	usRegInputBuf[INPUT_REG_CONTROLLER_PARAMS_RATE + 1] = w[1];
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
	if (usRegHoldingBuf[HOLDING_REG_CONTROLLER_MODE_UPDATE] & HOLDING_REG_APPLY_MSK)
	{
		usRegHoldingBuf[HOLDING_REG_CONTROLLER_MODE_UPDATE] &= (~HOLDING_REG_APPLY_MSK);

		// Controller operating mode
		SJC_setControlMode(usRegHoldingBuf[HOLDING_REG_CONTROLLER_MODE_UPDATE]);
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
		compose_float_from_byte_array(w, &reference);
		SJC_setControlReference(reference);
	}
}

static void Net_ExecuteActuatorConfigsUpdateCmd(void)
{
	if (usRegHoldingBuf[HOLDING_REG_ACTUATOR_SETUP_UPDATE] & HOLDING_REG_APPLY_MSK)
	{
		usRegHoldingBuf[HOLDING_REG_ACTUATOR_SETUP_UPDATE] &= (~HOLDING_REG_APPLY_MSK);
		float base_voltage = (1.0f * usRegHoldingBuf[HOLDING_REG_ACTUATOR_BASE_VOLTAGE] / 100.0f);
		float voltage_limit = (1.0f * usRegHoldingBuf[HOLDING_REG_ACTUATOR_LIMITS_VOLTAGE] / 100.0f);

		MotorDrive_setBaseVoltage(base_voltage);
		xSjcRange_t EffortLimits = {.Min = -voltage_limit, .Max = voltage_limit};
		PidController_setActuatorEffortLimits(EffortLimits);

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
		compose_ui32_from_byte_array(w, &config.Baudrate);

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

		Encoder_setResolution(SJC_AUX_JOINT_TH_X, usRegHoldingBuf[HOLDING_REG_SWING_THX_ENCODER_RESOLUTION_PPR]);
		Encoder_setResolution(SJC_AUX_JOINT_TH_Y, usRegHoldingBuf[HOLDING_REG_SWING_THY_ENCODER_RESOLUTION_PPR]);
		Encoder_setResolution(SJC_ACTUATOR_JOINT, usRegHoldingBuf[HOLDING_REG_ACTUATOR_ENCODER_RESOLUTION_PPR]);
	}
}

static void Net_ExecuteControllerParametersUpdateCmd(void)
{
	if (usRegHoldingBuf[HOLDING_REG_CONTROLLER_PARAMS_UPDATE] & HOLDING_REG_APPLY_MSK)
	{
		usRegHoldingBuf[HOLDING_REG_CONTROLLER_PARAMS_UPDATE] &= (~HOLDING_REG_APPLY_MSK);

		xSjcServoPidParameters_t params;
		uint16_t w[2];

		w[0] = usRegHoldingBuf[HOLDING_REG_CONTROLLER_PARAMS_KP];
		w[1] = usRegHoldingBuf[HOLDING_REG_CONTROLLER_PARAMS_KP + 1];
		compose_float_from_byte_array(w, &params.Kp);

		w[0] = usRegHoldingBuf[HOLDING_REG_CONTROLLER_PARAMS_KI];
		w[1] = usRegHoldingBuf[HOLDING_REG_CONTROLLER_PARAMS_KI + 1];
		compose_float_from_byte_array(w, &params.Ki);

		w[0] = usRegHoldingBuf[HOLDING_REG_CONTROLLER_PARAMS_KD];
		w[1] = usRegHoldingBuf[HOLDING_REG_CONTROLLER_PARAMS_KD + 1];
		compose_float_from_byte_array(w, &params.Kd);

		w[0] = usRegHoldingBuf[HOLDING_REG_CONTROLLER_PARAMS_KT];
		w[1] = usRegHoldingBuf[HOLDING_REG_CONTROLLER_PARAMS_KT + 1];
		compose_float_from_byte_array(w, &params.KT);

		w[0] = usRegHoldingBuf[HOLDING_REG_CONTROLLER_PARAMS_KA];
		w[1] = usRegHoldingBuf[HOLDING_REG_CONTROLLER_PARAMS_KA + 1];
		compose_float_from_byte_array(w, &params.KA);

		SJC_setControlParameters(params);

		// Controller update rate
		float control_rate = 0.0f;
		w[0] = usRegHoldingBuf[HOLDING_REG_CONTROLLER_PARAMS_RATE];
		w[1] = usRegHoldingBuf[HOLDING_REG_CONTROLLER_PARAMS_RATE + 1];
		compose_float_from_byte_array(w, &control_rate);
		SJC_setControlUpdateRateHz(control_rate);

#ifdef SJC_DEBUG_PRINT_NET_CMDS
		DebugOutput_printf("\r\n[NetCmd] Controller Kp: %f", params.Kp);
		DebugOutput_printf("\r\n[NetCmd] Controller Ki: %f", params.Ki);
		DebugOutput_printf("\r\n[NetCmd] Controller Kd: %f", params.Kd);
		DebugOutput_printf("\r\n[NetCmd] Controller Aux1 (KT): %f", params.KT);
		DebugOutput_printf("\r\n[NetCmd] Controller Aux2 (KA): %f", params.KA);
		DebugOutput_printf("\r\n[NetCmd] Controller Update Rate: %f Hz", control_rate);
#endif
	}
}

static void Net_ExecuteAdcFilterModeUpdateCmd(void)
{
	if (usRegHoldingBuf[HOLDING_REG_ADC_FILTER_CONFIG_UPDATE] & HOLDING_REG_APPLY_MSK)
	{
		usRegHoldingBuf[HOLDING_REG_ADC_FILTER_CONFIG_UPDATE] &= (~HOLDING_REG_APPLY_MSK);

		uint16_t mode = usRegHoldingBuf[HOLDING_REG_ADC_FILTER_CONFIG_MODE];

		uint16_t w[2];
		w[0] = usRegHoldingBuf[HOLDING_REG_ADC_FILTER_CONFIG_CUTOFF_FREQUENCY];
		w[1] = usRegHoldingBuf[HOLDING_REG_ADC_FILTER_CONFIG_CUTOFF_FREQUENCY + 1];

		float fc = 0.0f;
		compose_float_from_byte_array(w, &fc);

		SJC_setAdcFilterMode((eSjcAdcFilterMode_t)mode, fc);
		SJC_setControlMode(SJC_ANALOG_CALIBRATION);
	}
}

// ------------------------------------------------------------------- //
