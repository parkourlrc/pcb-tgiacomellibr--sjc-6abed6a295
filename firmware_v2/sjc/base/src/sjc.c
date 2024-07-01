/*
 * motor.c
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
#include "sjc.h"

#include "configs.h"
#include "analog_interfaces.h"
#include "data_logger.h"
#include "debug_output.h"
#include "digital_interfaces.h"
#include "encoder.h"
#include "helpers.h"
#include "motor_drive.h"
#include "motor_output.h"
#include "network.h"
#include "timeout_timer.h"
#include "pid_controller.h"
#include "pwm.h"
#include "task_common.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
extern volatile USHORT usRegInputBuf[REG_INPUT_NREGS];
extern volatile USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

static xSjcControlConfigs_t xSjcControlConfigs = xSjcControlConfigsEmptyObj;

static float fBiasCurrentValue = 0;
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //

// Input registers update process ------------------------------------ //
void Net_UpdateRuntimeInfoRegisters(void);
void Net_UpdateNetworkParametersRegisters(void);
void Net_UpdateSensorsParametersRegisters(void);
void Net_UpdateActuatorParametersRegisters(void);
void Net_UpdateControllerParametersRegisters(void);

// Holding registers update process ---------------------------------- //
void Net_ExecuteSystemConfigsUpdateCmd(void);
void Net_ExecuteNetworkConfigsUpdateCmd(void);
void Net_ExecuteSensorsConfigsUpdateCmd(void);
void Net_ExecuteActuatorConfigsUpdateCmd(void);
void Net_ExecuteControllerConfigsUpdateCmd(void);
void Net_ExecuteDigitalOutputsUpdateCmd(void);
void Net_ExecuteControllerParametersUpdateCmd(void);

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
	MX_USB_DEVICE_Init();
	MX_USART3_UART_Init();

	// Init Serial datalogger device
	MX_UART5_Init();

	// Init Joint PWM and Encoders
	MX_TIM1_Init();

	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM8_Init();

	// Init Modbus Network Serial and Timer
	MX_TIM10_Init();
	MX_USART2_UART_Init();

	// Init ADC and Sample time check timer
	MX_ADC1_Init();
	MX_TIM4_Init();

	HAL_Delay(1000);

	// SJC O.S. initialization
	osKernelInitialize(); /* Call init function for freertos objects (in freertos.c) */
	SJC_initOsObjects();


	// SJC Modules initialization
	fBiasCurrentValue = 0;
	SJC_setVersions();

	DataLogger_init();
	DebugOutput_init();
	AnalogInterfaces_init();
	DigitalInterfaces_init();

	Encoder_init();
	Encoder_enable(SJC_AUX_JOINT_A);
	Encoder_enable(SJC_AUX_JOINT_B);
	Encoder_enable(SJC_ACTUATOR_JOINT);

	PidController_Init();

	xSjcControlConfigs.Mode = SJC_CURRENT_CALIBRATION;
	xSjcControlConfigs.Rate = SJC_CONTROLLER_MAX_UPDATE_RATE_HZ;

	MotorOutput_init();
	MotorDrive_init(PWM_FREQ_20kHz, 24.0);

	SJC_presentSystemInfo();

	// SJC Threads initialization
	SJC_initOsThreads();
}

void SJC_initOsObjects(void)
{
	SJC_TaskCommon_initCommonObjects();
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
	usRegInputBuf[REG_INPUT_HW_VERSION] = SJC_HW_VERSION_MAJOR << 8 | SJC_HW_VERSION_MINOR;

	usRegInputBuf[REG_INPUT_FW_VERSION] = SJC_FW_VERSION_MAJOR << 8 | SJC_FW_VERSION_MINOR;
	usRegInputBuf[REG_INPUT_FW_VERSION + 1] = SJC_FW_VERSION_PATCH;

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
		Timer_delayMs(value);
		break;
	case SJC_TIME_SEC:
		Timer_delaySec(value);
		break;
	default:
		ret = SJC_RET_NOK;
		break;
	}
	return ret;
}

eSjcReturnType_t SJC_getTime(const eSjcTimeType_t type, uint32_t *value)
{
	eSjcReturnType_t ret = SJC_RET_OK;

	switch (type)
	{
	case SJC_TIME_MS:
		*value = Timer_getTimeMs();
		break;
	case SJC_TIME_SEC:
		*value = Timer_getTimeSec();
		break;
	default:
		ret = SJC_RET_NOK;
		break;
	}
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getElapsedTime(const eSjcTimeType_t type, const uint32_t timepoint, uint32_t *value)
{
	eSjcReturnType_t ret = SJC_RET_OK;
	uint32_t now;

	switch (type)
	{
	case SJC_TIME_MS:
		now = Timer_getTimeMs();
		break;
	case SJC_TIME_SEC:
		now = Timer_getTimeSec();
		break;
	default:
		now = 0;
		ret = SJC_RET_NOK;
		break;
	}

	*value = (now > timepoint) ? (now - timepoint) : (1 + now + (UINT32_MAX - timepoint));
	return ret;
}

eSjcReturnType_t SJC_getControlMode(eSjcControlMode_t *config)
{
	*config = xSjcControlConfigs.Mode;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_setControlMode(const eSjcControlMode_t config)
{
	xSjcControlConfigs.Mode = config;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getControlUpdateRateHz(float *config)
{
	*config = xSjcControlConfigs.Mode;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_setControlUpdateRateHz(const float config)
{
	xSjcControlConfigs.Rate = SJC_Helpers_RealClamp(config, SJC_CONTROLLER_MIN_UPDATE_RATE_HZ, SJC_CONTROLLER_MAX_UPDATE_RATE_HZ);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getControlUpdatePeriodMs(uint32_t *config)
{
	*config = ((uint32_t)(1000 / xSjcControlConfigs.Rate));

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_setControlUpdatePeriodMs(const uint32_t config)
{
	xSjcControlConfigs.Rate = ( (uint32_t) (1000/SJC_Helpers_UintClamp(config, SJC_CONTROLLER_MIN_UPDATE_PERIOD_MS, SJC_CONTROLLER_MAX_UPDATE_PERIOD_MS)) );

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getControlReference(float *reference)
{
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
	*controller_error = PidController_getError();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getControlParameters(xSjcServoPidParameters_t *Parameters)
{
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
	*ControlAction = PidController_compute(ControlError, dt);

	return SJC_RET_OK;
}

void SJC_updateHoldingRegisters(void) {
	Net_ExecuteSystemConfigsUpdateCmd();
	Net_ExecuteDigitalOutputsUpdateCmd();
	Net_ExecuteNetworkConfigsUpdateCmd();
	Net_ExecuteSensorsConfigsUpdateCmd();
	Net_ExecuteActuatorConfigsUpdateCmd();
	Net_ExecuteControllerConfigsUpdateCmd();
	Net_ExecuteControllerParametersUpdateCmd();
}

void SJC_updateInputRegisters(void) {
	Net_UpdateRuntimeInfoRegisters();
	Net_UpdateNetworkParametersRegisters();
	Net_UpdateSensorsParametersRegisters();
	Net_UpdateActuatorParametersRegisters();
	Net_UpdateControllerParametersRegisters();
}

void SJC_executeNetworkDaemon(void)
{
	Network_update();
}

void SJC_processNetworkData(void)
{
	Network_processReceivedData();
}

eSjcReturnType_t SJC_enableAdc(void)
{
	AnalogInterfaces_measurementEnable();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_disableAdc(void)
{
	AnalogInterfaces_measurementDisable();

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

eSjcReturnType_t SJC_resetJointAngularDisplacement(void)
{
	Encoder_reset(SJC_ACTUATOR_JOINT);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_getJointAngularDisplacement(const eSjcJoints_t joint, float *disp)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	if (joint < SJC_JOINTS_SIZE)
	{
		ret = Encoder_getAngularDisplacement(joint, disp);
	}

	return ret;
}

eSjcReturnType_t SJC_getJointRelativeAngularDisplacement(const eSjcJoints_t joint, float *RelativeDisp, const float PrevDisp)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	if (joint < SJC_JOINTS_SIZE)
	{
		ret = Encoder_getAngularDisplacement(joint, RelativeDisp);
		*RelativeDisp -= PrevDisp;
	}

	return ret;
}

eSjcReturnType_t SJC_getJointActuatorPwmFrequency(uint32_t *freq)
{
	*freq = PWM_getFrequency();

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
	eSjcReturnType_t ret = MotorDrive_getOutputCurrent(value);
	*value -= fBiasCurrentValue;
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
void Net_UpdateRuntimeInfoRegisters(void)
{
	float fAux;
	uint16_t ui16Aux;
	uint32_t ui32Aux;
	uint32_t uptime = Timer_getTimeMs();
	uint16_t *ui16ptr = ((uint16_t *)(&uptime));
	usRegInputBuf[REG_INPUT_SYSTEM_UPTIME] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_SYSTEM_UPTIME + 1] = ui16ptr[0];

	eSjcControlMode_t mode;
	SJC_getControlMode(&mode);
	usRegInputBuf[REG_INPUT_CONTROL_MODE] = mode;

	fAux = 0;
	SJC_getControlError(&fAux);
	ui16ptr = ((uint16_t *)(&fAux));
	usRegInputBuf[REG_INPUT_CONTROL_ERROR] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROL_ERROR + 1] = ui16ptr[0];

	fAux = 0;
	SJC_getControlReference(&fAux);
	ui16ptr = ((uint16_t *)(&fAux));
	usRegInputBuf[REG_INPUT_CONTROL_REFERENCE] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROL_REFERENCE + 1] = ui16ptr[0];

	fAux = 0;
	SJC_getJointAngularDisplacement(SJC_ACTUATOR_JOINT, &fAux);
	ui16ptr = ((uint16_t *)(&fAux));
	usRegInputBuf[REG_INPUT_ACTUATOR_DISPLACEMENT] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_ACTUATOR_DISPLACEMENT + 1] = ui16ptr[0];

	fAux = 0;
	SJC_getJointAngularDisplacement(SJC_AUX_JOINT_A, &fAux);
	ui16ptr = ((uint16_t *)(&fAux));
	usRegInputBuf[REG_INPUT_AUX_ENCODER_A_DISPLACEMENT] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_AUX_ENCODER_A_DISPLACEMENT + 1] = ui16ptr[0];

	fAux = 0;
	SJC_getJointAngularDisplacement(SJC_AUX_JOINT_B, &fAux);
	ui16ptr = ((uint16_t *)(&fAux));
	usRegInputBuf[REG_INPUT_AUX_ENCODER_B_DISPLACEMENT] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_AUX_ENCODER_B_DISPLACEMENT + 1] = ui16ptr[0];

	fAux = 0;
	SJC_getJointActuatorVoltage(&fAux);
	ui16ptr = ((uint16_t *)(&fAux));
	usRegInputBuf[REG_INPUT_ACTUATOR_VOLTAGE] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_ACTUATOR_VOLTAGE + 1] = ui16ptr[0];

	fAux = 0;
//	SJC_getJointActuatorCurrent(&fAux);
	SJC_getJointActuatorCurrentUnbiased(&fAux);
	ui16ptr = ((uint16_t *)(&fAux));
	usRegInputBuf[REG_INPUT_ACTUATOR_CURRENT] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_ACTUATOR_CURRENT + 1] = ui16ptr[0];

	bool state;
	SJC_getJointActuatorDriverState(&state);
	usRegInputBuf[REG_INPUT_ACTUATOR_STATE] = state;

	ui32Aux = 0;
	SJC_readDigitalInputs(&ui32Aux);
	ui16Aux = (uint16_t)ui32Aux;
	usRegInputBuf[REG_INPUT_DIGITAL_INPUTS] = ui16Aux;

	ui32Aux = 0;
	SJC_readDigitalOutputs(&ui32Aux);
	ui16Aux = (uint16_t)ui32Aux;
	usRegInputBuf[REG_INPUT_DIGITAL_OUTPUTS] = ui16Aux;

	fAux = 0;
	SJC_getJointActuatorPowerSupplyVoltage(&fAux);
	ui16ptr = ((uint16_t *)(&fAux));
	usRegInputBuf[REG_INPUT_SYSTEM_VOLTAGE] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_SYSTEM_VOLTAGE + 1] = ui16ptr[0];

	usRegInputBuf[REG_INPUT_SYSTEM_ERROR_FLAGS] = 0x00;
}

void Net_UpdateNetworkParametersRegisters(void)
{
	uint16_t xSlaveAddress;
	uint32_t xBaudRate;
	char xParity;
	Network_getParameters(&xSlaveAddress, &xBaudRate, &xParity);

	usRegInputBuf[REG_INPUT_NETWORK_ADDRESS] = xSlaveAddress;
	usRegInputBuf[REG_INPUT_NETWORK_PARITY] = xParity;
	uint16_t *ptr = (uint16_t *)(&xBaudRate);
	usRegInputBuf[REG_INPUT_NETWORK_BAUD_RATE] = ptr[1];
	usRegInputBuf[REG_INPUT_NETWORK_BAUD_RATE + 1] = ptr[0];
}

void Net_UpdateSensorsParametersRegisters(void)
{
	uint16_t aux;

	Encoder_getResolution(SJC_ACTUATOR_JOINT, &aux);
	usRegInputBuf[REG_INPUT_ACTUATOR_ENCODER_RESOLUTION_PPR] = aux;

	Encoder_getResolution(SJC_AUX_JOINT_A, &aux);
	usRegInputBuf[REG_INPUT_AUX_A_ENCODER_RESOLUTION_PPR] = aux;

	Encoder_getResolution(SJC_AUX_JOINT_B, &aux);
	usRegInputBuf[REG_INPUT_AUX_B_ENCODER_RESOLUTION_PPR] = aux;
}

void Net_UpdateActuatorParametersRegisters(void)
{
	float base_voltage;
	MotorDrive_getBaseVoltage(&base_voltage);
	usRegInputBuf[REG_INPUT_ACTUATOR_BASE_VOLTAGE] = base_voltage * 100.0f;

	/* Reserved for future implementation
	 *
	 * uint16_t * ui16ptr = ((uint16_t*)(&xSjcJoint[SJC_ACTUATOR_JOINT].Limits.Speed.Max));
	 * usRegInputBuf[REG_INPUT_ACTUATOR_LIMITS_SPEED] = ui16ptr[1];
	 * usRegInputBuf[REG_INPUT_ACTUATOR_LIMITS_SPEED+1] = ui16ptr[0];
	 * usRegInputBuf[REG_INPUT_ACTUATOR_LIMITS_VOLTAGE] = xSjcJoint[SJC_ACTUATOR_JOINT].Limits.Voltage.Max*100.0f;
	 * usRegInputBuf[REG_INPUT_ACTUATOR_LIMITS_CURRENT] = xSjcJoint[SJC_ACTUATOR_JOINT].Limits.Current.Max*100.0f;
	 */
}

void Net_UpdateControllerParametersRegisters(void)
{
	xSjcServoPidParameters_t Parameters;
	SJC_getControlParameters(&Parameters);

	uint16_t *ui16ptr = (uint16_t *)(&Parameters.Kp);
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_KP] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_KP + 1] = ui16ptr[0];

	ui16ptr = (uint16_t *)(&Parameters.Ki);
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_KI] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_KI + 1] = ui16ptr[0];

	ui16ptr = (uint16_t *)(&Parameters.Kd);
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_KD] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_KD + 1] = ui16ptr[0];

	ui16ptr = (uint16_t *)(&Parameters.KT);
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_KT] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_KT + 1] = ui16ptr[0];

	ui16ptr = (uint16_t *)(&Parameters.KA);
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_KA] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_KA + 1] = ui16ptr[0];

	float rate = 0;
	SJC_getControlUpdateRateHz(&rate);
	ui16ptr = (uint16_t *)(&rate);
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_RATE] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_RATE + 1] = ui16ptr[0];
}

// Holding registers update process ---------------------------------- //

void Net_ExecuteSystemConfigsUpdateCmd(void)
{
	if (usRegHoldingBuf[REG_HOLDING_SYSTEM_REBOOT])
	{

		// Define digital outputs states
		usRegHoldingBuf[REG_HOLDING_DIGITAL_OUTPUTS_STATE_UPDATE] &= 0x0000;

		// Apply system reboot
	}
}

void Net_ExecuteDigitalOutputsUpdateCmd(void)
{
	if (usRegHoldingBuf[REG_HOLDING_DIGITAL_OUTPUTS_STATE_UPDATE] & DIGITAL_OUTPUTS_UPDATE_MSK)
	{

		// Define digital outputs states
		usRegHoldingBuf[REG_HOLDING_DIGITAL_OUTPUTS_STATE_UPDATE] &= (~DIGITAL_OUTPUTS_UPDATE_MSK);

		SJC_writeDigitalOutputs(usRegHoldingBuf[REG_HOLDING_DIGITAL_OUTPUTS_STATE_UPDATE]);
	}
}

void Net_ExecuteControllerConfigsUpdateCmd(void)
{
	if (usRegHoldingBuf[REG_HOLDING_CONTROLLER_MODE_UPDATE] & CONTROLLER_UPDATE_MSK)
	{
		usRegHoldingBuf[REG_HOLDING_CONTROLLER_MODE_UPDATE] &= (~CONTROLLER_UPDATE_MSK);

		// Controller reference
		float reference;
		uint16_t *ptr = (uint16_t *)(&reference);
		ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_REFERENCE + 1];
		ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_REFERENCE];
		SJC_setControlReference(reference);

		// Controller operating mode
		SJC_setControlMode(usRegHoldingBuf[REG_HOLDING_CONTROLLER_MODE_UPDATE]);

#ifdef SJC_DEBUG_PRINT_NET_CMDS
		SJC_DebugOutput_printf("\r\n[NetCmd] Controller reference: %f", reference);

		switch (xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Mode)
		{
		case SJC_OPEN_LOOP_CONTROL:
			SJC_DebugOutput_printf("\r\n[NetCmd] Controller mode: Open loop");
			break;
		case SJC_PRBS_EXCITATION:
			SJC_DebugOutput_printf("\r\n[NetCmd] Controller mode: PRBS Excitation");
			break;
		case SJC_SPEED_CONTROL:
			SJC_DebugOutput_printf("\r\n[NetCmd] Controller mode: Speed control");
			break;
		case SJC_CURRENT_CONTROL:
			SJC_DebugOutput_printf("\r\n[NetCmd] Controller mode: Torque control");
			break;

		default: // SJC_CONTROL_DISABLED
			SJC_DebugOutput_printf("\r\n[NetCmd] Controller mode: Control disabled");
			break;
		}
#endif
	}
}

void Net_ExecuteActuatorConfigsUpdateCmd(void)
{
	if (usRegHoldingBuf[REG_HOLDING_ACTUATOR_SETUP_UPDATE] & ACTUATOR_CONFIGS_UPDATE_MSK)
	{
		usRegHoldingBuf[REG_HOLDING_ACTUATOR_SETUP_UPDATE] &= (~ACTUATOR_CONFIGS_UPDATE_MSK);
		float base_voltage = (1.0f * usRegHoldingBuf[REG_HOLDING_ACTUATOR_BASE_VOLTAGE] / 100.0f);
		float voltage_limit = (1.0f*usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_VOLTAGE]/100.0f);

		MotorDrive_setBaseVoltage(base_voltage);
		xSjcRange_t EffortLimits = {.Min = -voltage_limit, .Max = voltage_limit};
		PidController_setActuatorEffortLimits(EffortLimits);

		/* Reserved for future implementation
		 *
		 * float current_limit = (1.0f*usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_CURRENT]/100.0f);
		 * float voltage_limit = (1.0f*usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_VOLTAGE]/100.0f);
		 * float speed_limit;

		 * uint16_t *ptr = (uint16_t*) (&speed_limit);
		 * ptr[0] = usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_SPEED + 1];
		 * ptr[1] = usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_SPEED];
		 */

#ifdef SJC_DEBUG_PRINT_NET_CMDS
		SJC_DebugOutput_printf("\r\n[NetCmd] Actuator base voltage: %f V", MotorDrive_getBaseVoltage(&xSjcJoint[SJC_ACTUATOR_JOINT].MotorDrive));
		SJC_DebugOutput_printf("\r\n[NetCmd] Actuator voltage limit: %f V", MotorDrive_getVoltageLimit(&xSjcJoint[SJC_ACTUATOR_JOINT].Limits));
		SJC_DebugOutput_printf("\r\n[NetCmd] Actuator current limit: %f A", MotorDrive_getCurrentLimit(&xSjcJoint[SJC_ACTUATOR_JOINT].Limits));
		SJC_DebugOutput_printf("\r\n[NetCmd] Actuator speed limit: %f rd/s", MotorDrive_getSpeedLimit(&xSjcJoint[SJC_ACTUATOR_JOINT].Limits));
#endif
	}
}

void Net_ExecuteNetworkConfigsUpdateCmd(void)
{
	if (usRegHoldingBuf[REG_HOLDING_NETWORK_CONFIG_UPDATE] & NETWORK_PARAMETERS_UPDATE_MSK)
	{

		// Network parameters
		usRegHoldingBuf[REG_HOLDING_NETWORK_CONFIG_UPDATE] &= (~NETWORK_PARAMETERS_UPDATE_MSK);

		xSjcNetworkConfig_t config;

		uint16_t *ptr = (uint16_t *)(&config.Baudrate);
		ptr[0] = usRegHoldingBuf[REG_HOLDING_NETWORK_CONFIG_BAUD_RATE + 1];
		ptr[1] = usRegHoldingBuf[REG_HOLDING_NETWORK_CONFIG_BAUD_RATE];

		config.Parity = usRegHoldingBuf[REG_HOLDING_NETWORK_CONFIG_PARITY];
		config.Address = usRegHoldingBuf[REG_HOLDING_NETWORK_CONFIG_ADDRESS];

#ifdef SJC_DEBUG_PRINT_NET_CMDS
		SJC_DebugOutput_printf("\r\n[NetCmd] Network address: %u", config.Address);
		SJC_DebugOutput_printf("\r\n[NetCmd] Network parity: %c", config.Parity);
		SJC_DebugOutput_printf("\r\n[NetCmd] Network baudrate: %lu bps", config.Baudrate);
#endif

		// Write to flash
		// Reboot
	}
}

void Net_ExecuteSensorsConfigsUpdateCmd(void)
{
	if (usRegHoldingBuf[REG_HOLDING_SENSORS_SETUP_UPDATE] & SENSORS_PARAMETERS_UPDATE_MSK)
	{

		// Encoders parameters
		usRegHoldingBuf[REG_HOLDING_SENSORS_SETUP_UPDATE] &= (~SENSORS_PARAMETERS_UPDATE_MSK);

		Encoder_setResolution(SJC_AUX_JOINT_A, usRegHoldingBuf[REG_HOLDING_AUX_ENCODER_A_RESOLUTION_PPR]);
		Encoder_setResolution(SJC_AUX_JOINT_B, usRegHoldingBuf[REG_HOLDING_AUX_ENCODER_B_RESOLUTION_PPR]);
		Encoder_setResolution(SJC_ACTUATOR_JOINT, usRegHoldingBuf[REG_HOLDING_ACTUATOR_ENCODER_RESOLUTION_PPR]);

#ifdef SJC_DEBUG_PRINT_NET_CMDS
		SJC_DebugOutput_printf("\r\n[NetCmd] AuxA encoder res.: %u ppr", xSjcJoint[SJC_AUX_JOINT_A].Encoder.Resolution_PPR);
		SJC_DebugOutput_printf("\r\n[NetCmd] AuxB encoder res.: %u ppr", xSjcJoint[SJC_AUX_JOINT_B].Encoder.Resolution_PPR);
		SJC_DebugOutput_printf("\r\n[NetCmd] Actuator encoder res.: %u ppr", xSjcJoint[SJC_ACTUATOR_JOINT].Encoder.Resolution_PPR);
#endif
	}
}

void Net_ExecuteControllerParametersUpdateCmd(void)
{
	if (usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_UPDATE] & CONTROLLER_PARAMETERS_UPDATE_MSK)
	{

		// Controller parameters
		usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_UPDATE] &= (~CONTROLLER_PARAMETERS_UPDATE_MSK);

		xSjcServoPidParameters_t params;
		uint16_t *ptr = (uint16_t *)(&params.Kp);
		ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_KP + 1];
		ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_KP];

		ptr = (uint16_t *)(&params.Ki);
		ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_KI + 1];
		ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_KI];

		ptr = (uint16_t *)(&params.Kd);
		ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_KD + 1];
		ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_KD];

		ptr = (uint16_t *)(&params.KT);
		ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_KT + 1];
		ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_KT];

		ptr = (uint16_t *)(&params.KA);
		ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_KA + 1];
		ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_KA];

		SJC_setControlParameters(params);

		// Controller update rate
		float control_rate;
		ptr = (uint16_t *)(&control_rate);
		ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_RATE + 1];
		ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_RATE];
		SJC_setControlUpdateRateHz(control_rate);

#ifdef SJC_DEBUG_PRINT_NET_CMDS
		SJC_DebugOutput_printf("\r\n[NetCmd] Controller Kp: %f", params.Kp);
		SJC_DebugOutput_printf("\r\n[NetCmd] Controller Ki: %f", params.Ki);
		SJC_DebugOutput_printf("\r\n[NetCmd] Controller Kd: %f", params.Kd);
		SJC_DebugOutput_printf("\r\n[NetCmd] Controller Aux1: %f", params.KT);
		SJC_DebugOutput_printf("\r\n[NetCmd] Controller Aux2: %f", params.KA);
		SJC_DebugOutput_printf("\r\n[NetCmd] Controller Update Rate: %f Hz", control_rate);
#endif
	}
}

// ------------------------------------------------------------------- //
