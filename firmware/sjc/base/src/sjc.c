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
#include "debug_output.h"
#include "digital_interfaces.h"
#include "encoder.h"
#include "helpers.h"
#include "motor_drive.h"
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
void SJC_Init(void)
{
	// SJC Peripheral initialization
	// Init all board GPIOs
	MX_GPIO_Init();

	// Init DMA controller
	MX_DMA_Init();

	// Init ADC and Sample time check timer
	MX_ADC1_Init();
	MX_TIM4_Init();

	// Init Joint PWM and Encoders
	MX_TIM1_Init();

	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM8_Init();

	// Init USB and Serial debug devices
	MX_USB_DEVICE_Init();
	MX_USART3_UART_Init();

	// SJC Modules initialization
	SJC_SetVersions();

	SJC_DebugOutput_Init();
	SJC_AnalogInterfaces_Init();
	SJC_DigitalInterfaces_init();

	SJC_Encoder_init();
	SJC_Encoder_Enable(SJC_AUX_JOINT_A);
	SJC_Encoder_Enable(SJC_AUX_JOINT_B);
	SJC_Encoder_Enable(SJC_ACTUATOR_JOINT);

	SJC_PidController_Init();

	xSjcControlConfigs.Mode = SJC_CONTROL_DISABLED;
	xSjcControlConfigs.Rate = SJC_CONTROLLER_MIN_UPDATE_RATE_HZ;

	SJC_MotorDrive_Init(PWM_FREQ_20kHz, 24.0);

	SJC_PresentSystemInfo();
	SJC_InitOsKernel();
}

void SJC_InitOsKernel(void)
{
	/* Init scheduler */
	osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */

	/* Init Startup Tasks */
	SJC_TaskCommon_Init();

	/* Start scheduler */
	osKernelStart();

	while (1)
	{

	}
}

void SJC_PresentSystemInfo(void)
{
	SJC_DebugOutput_PresentSystemInfo();
}

eSjcReturnType_t SJC_SetVersions(void)
{
	usRegInputBuf[REG_INPUT_HW_VERSION] = SJC_HW_VERSION_MAJOR << 8 | SJC_HW_VERSION_MINOR;

	usRegInputBuf[REG_INPUT_FW_VERSION] = SJC_FW_VERSION_MAJOR << 8 | SJC_FW_VERSION_MINOR;
	usRegInputBuf[REG_INPUT_FW_VERSION+1] = SJC_FW_VERSION_PATCH;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_Delay(const eSjcTimeType_t type, const uint32_t value)
{
	eSjcReturnType_t ret = SJC_RET_OK;

	if (value == 0)
	{
		return SJC_RET_NOK;
	}

	switch (type) {
	case SJC_TIME_MS:
		SJC_Timer_DelayMs(value);
		break;
	case SJC_TIME_SEC:
		SJC_Timer_DelaySec(value);
		break;
	default:
		ret = SJC_RET_NOK;
		break;
	}
	return ret;
}

eSjcReturnType_t SJC_GetTime(const eSjcTimeType_t type, uint32_t *value)
{
	eSjcReturnType_t ret = SJC_RET_OK;

	switch (type) {
	case SJC_TIME_MS:
		*value = SJC_Timer_GetTimeMs();
		break;
	case SJC_TIME_SEC:
		*value = SJC_Timer_GetTimeSec();
		break;
	default:
		ret = SJC_RET_NOK;
		break;
	}
	return ret;
}

eSjcReturnType_t SJC_GetControlMode(eSjcControlMode_t *config)
{
	*config = xSjcControlConfigs.Mode;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_SetControlMode(const eSjcControlMode_t config)
{
	xSjcControlConfigs.Mode = config;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_GetControlUpdateRateHz(float * config)
{
	*config = xSjcControlConfigs.Mode;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_SetControlUpdateRateHz(const float config)
{
	xSjcControlConfigs.Rate = SJC_Helpers_RealClamp(config, SJC_CONTROLLER_MIN_UPDATE_RATE_HZ, SJC_CONTROLLER_MAX_UPDATE_RATE_HZ);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_GetControlUpdatePeriodMs(uint32_t * config)
{
	*config = ((uint32_t) (1000/xSjcControlConfigs.Rate));

	return SJC_RET_OK;

}

eSjcReturnType_t SJC_GetControlReference(float * reference)
{
	*reference = SJC_PidController_GetReference();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_SetControlReference(const float reference)
{
	SJC_PidController_SetReference(reference);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_GetControlError(float * controller_error)
{
	*controller_error = SJC_PidController_GetError();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_GetControlParameters(xPidControllerParameters_t * Parameters)
{
	*Parameters = SJC_PidController_GetControlParameters();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_SetControlParameters(const xPidControllerParameters_t Parameters)
{
	SJC_PidController_SetControlParameters(Parameters);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_ComputeControlAction(const float ControlError, const float dt, float * ControlAction)
{
	*ControlAction = SJC_PidController_Compute(ControlError, dt);

	return SJC_RET_OK;
}


void SJC_ExecuteNetworkCommands(void)
{
	Net_ExecuteSystemConfigsUpdateCmd();
	Net_ExecuteDigitalOutputsUpdateCmd();
	Net_ExecuteNetworkConfigsUpdateCmd();
	Net_ExecuteSensorsConfigsUpdateCmd();
	Net_ExecuteActuatorConfigsUpdateCmd();
	Net_ExecuteControllerConfigsUpdateCmd();
	Net_ExecuteControllerParametersUpdateCmd();
}

void SJC_UpdateNetworkDataRegisters(void)
{
	Net_UpdateRuntimeInfoRegisters();
	Net_UpdateNetworkParametersRegisters();
	Net_UpdateSensorsParametersRegisters();
	Net_UpdateActuatorParametersRegisters();
	Net_UpdateControllerParametersRegisters();
}

eSjcReturnType_t SJC_EnableAdc(void)
{
	SJC_AnalogInterfaces_MeasurementEnable();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_DisableAdc(void)
{
	SJC_AnalogInterfaces_MeasurementDisable();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_EnableJointActuators(void)
{
	SJC_MotorDrive_Enable();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_DisableJointActuators(void)
{
	SJC_MotorDrive_Disable();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_GetJointActuatorState(bool * state)
{
	return SJC_MotorDrive_GetState(state);
}

eSjcReturnType_t SJC_GetJointActuatorVoltage(float * voltage)
{
	return SJC_MotorDrive_GetOutputVoltage(voltage);
}

eSjcReturnType_t SJC_SetJointActuatorVoltage(const float voltage)
{
	SJC_MotorDrive_SetOutputVoltage(voltage);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_GetJointAngularDisplacement(const eSjcJoints_t joint, float * disp)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	if (joint < SJC_JOINTS_SIZE)
	{
		ret = SJC_Encoder_GetAngularDisplacement(joint, disp);
	}

	return ret;
}

eSjcReturnType_t SJC_GetJointRelativeAngularDisplacement(const eSjcJoints_t joint, float * RelativeDisp, const float PrevDisp)
{
	eSjcReturnType_t ret = SJC_RET_NOK;

	if (joint < SJC_JOINTS_SIZE)
	{
		ret = SJC_Encoder_GetAngularDisplacement(joint, RelativeDisp);
		*RelativeDisp -= PrevDisp;
	}

	return ret;
}

eSjcReturnType_t SJC_GetJointActuatorPwmFrequency(uint32_t * freq)
{
	*freq = SJC_PWM_GetFrequency();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_GetJointActuatorCurrent(float * value)
{
	return SJC_MotorDrive_GetOutputCurrent(value);
}

eSjcReturnType_t SJC_GetJointActuatorPowerSupplyVoltage(float * value)
{
	return SJC_MotorDrive_GetPowerSupplyVoltage(value);}

eSjcReturnType_t SJC_ReadDigitalInputs(uint32_t * states)
{
	return SJC_DigitalInterfaces_ReadInputStates(states);
}

eSjcReturnType_t SJC_ReadDigitalOutputs(uint32_t * states)
{
	return SJC_DigitalInterfaces_ReadOutputStates(states);
}

eSjcReturnType_t SJC_WriteDigitalOutputs(const uint8_t states)
{
	return SJC_DigitalInterfaces_WriteOutputStates(states);
}

eSjcReturnType_t SJC_ToggleDigitalOutput(const eSjcDigitalOutputs_t Output)
{
	return SJC_DigitalInterfaces_ToggleOutputState(Output);
}

eSjcReturnType_t SJC_ReadDigitalInput(const eSjcDigitalInputs_t Input, bool * state)
{
	return SJC_DigitalInterfaces_ReadInputState(Input, state);
}

eSjcReturnType_t SJC_ReadDigitalOutput(const eSjcDigitalOutputs_t Output, bool *state)
{
	return SJC_DigitalInterfaces_ReadOutputState(Output, state);
}

eSjcReturnType_t SJC_WriteDigitalOutput(const eSjcDigitalOutputs_t Output, const bool state)
{
	return SJC_DigitalInterfaces_WriteOutputState(Output, state);
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
	uint32_t uptime = SJC_Timer_GetTimeMs();
	uint16_t * ui16ptr = ((uint16_t*)(&uptime));
	usRegInputBuf[REG_INPUT_SYSTEM_UPTIME] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_SYSTEM_UPTIME+1] = ui16ptr[0];

	eSjcControlMode_t mode;
	SJC_GetControlMode(&mode);
	usRegInputBuf[REG_INPUT_CONTROL_MODE] = mode;

	fAux = 0;
	SJC_GetControlError(&fAux);
	ui16ptr = ((uint16_t*)(&fAux));
	usRegInputBuf[REG_INPUT_CONTROL_ERROR] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROL_ERROR+1] = ui16ptr[0];

	fAux = 0;
	SJC_GetControlReference(&fAux);
	ui16ptr = ((uint16_t*)(&fAux));
	usRegInputBuf[REG_INPUT_CONTROL_REFERENCE] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROL_REFERENCE+1] = ui16ptr[0];

	fAux = 0;
	SJC_GetJointAngularDisplacement(SJC_ACTUATOR_JOINT, &fAux);
	ui16ptr = ((uint16_t*)(&fAux));
	usRegInputBuf[REG_INPUT_ACTUATOR_DISPLACEMENT] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_ACTUATOR_DISPLACEMENT+1] = ui16ptr[0];

	fAux = 0;
	SJC_GetJointAngularDisplacement(SJC_AUX_JOINT_A, &fAux);
	ui16ptr = ((uint16_t*)(&fAux));
	usRegInputBuf[REG_INPUT_AUX_ENCODER_A_DISPLACEMENT] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_AUX_ENCODER_A_DISPLACEMENT+1] = ui16ptr[0];

	fAux = 0;
	SJC_GetJointAngularDisplacement(SJC_AUX_JOINT_B, &fAux);
	ui16ptr = ((uint16_t*)(&fAux));
	usRegInputBuf[REG_INPUT_AUX_ENCODER_B_DISPLACEMENT] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_AUX_ENCODER_B_DISPLACEMENT+1] = ui16ptr[0];

	fAux = 0;
	SJC_GetJointActuatorVoltage(&fAux);
	ui16ptr = ((uint16_t*)(&fAux));
	usRegInputBuf[REG_INPUT_ACTUATOR_VOLTAGE] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_ACTUATOR_VOLTAGE+1] = ui16ptr[0];

	fAux = 0;
	SJC_GetJointActuatorCurrent(&fAux);
	ui16ptr = ((uint16_t*)(&fAux));
	usRegInputBuf[REG_INPUT_ACTUATOR_CURRENT] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_ACTUATOR_CURRENT+1] = ui16ptr[0];

	bool state;
	SJC_GetJointActuatorState(&state);
	usRegInputBuf[REG_INPUT_ACTUATOR_STATE] = state;

	ui32Aux = 0;
	SJC_ReadDigitalInputs(&ui32Aux);
	ui16Aux = (uint16_t) ui32Aux;
	usRegInputBuf[REG_INPUT_DIGITAL_INPUTS] = ui16Aux;

	ui32Aux = 0;
	SJC_ReadDigitalOutputs(&ui32Aux);
	ui16Aux = (uint16_t) ui32Aux;
	usRegInputBuf[REG_INPUT_DIGITAL_OUTPUTS] = ui16Aux;

	fAux = 0;
	SJC_GetJointActuatorPowerSupplyVoltage(&fAux);
	ui16ptr = ((uint16_t*)(&fAux));
	usRegInputBuf[REG_INPUT_SYSTEM_VOLTAGE] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_SYSTEM_VOLTAGE+1] = ui16ptr[0];

	usRegInputBuf[REG_INPUT_SYSTEM_ERROR_FLAGS] = 0x00;
}

void Net_UpdateNetworkParametersRegisters(void)
{
	uint16_t xSlaveAddress;
	uint32_t xBaudRate;
	char xParity;
	SJC_Network_GetParameters(&xSlaveAddress, &xBaudRate, &xParity);


	usRegInputBuf[REG_INPUT_NETWORK_ADDRESS] = xSlaveAddress;
	usRegInputBuf[REG_INPUT_NETWORK_PARITY] = xParity;
	uint16_t * ptr = (uint16_t *)(&xBaudRate);
	usRegInputBuf[REG_INPUT_NETWORK_BAUD_RATE] = ptr[1];
	usRegInputBuf[REG_INPUT_NETWORK_BAUD_RATE+1] = ptr[0];
}

void Net_UpdateSensorsParametersRegisters(void)
{
	uint16_t aux;

	SJC_Encoder_GetResolution(SJC_ACTUATOR_JOINT, &aux);
	usRegInputBuf[REG_INPUT_ACTUATOR_ENCODER_RESOLUTION_PPR] = aux;

	SJC_Encoder_GetResolution(SJC_AUX_JOINT_A, &aux);
	usRegInputBuf[REG_INPUT_AUX_A_ENCODER_RESOLUTION_PPR] = aux;

	SJC_Encoder_GetResolution(SJC_AUX_JOINT_B, &aux);
	usRegInputBuf[REG_INPUT_AUX_B_ENCODER_RESOLUTION_PPR] = aux;
}

void Net_UpdateActuatorParametersRegisters(void)
{
	float base_voltage;
	SJC_MotorDrive_GetBaseVoltage(&base_voltage);
	usRegInputBuf[REG_INPUT_ACTUATOR_BASE_VOLTAGE] = base_voltage*100.0f;

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
	xPidControllerParameters_t Parameters;
	SJC_GetControlParameters(&Parameters);

	uint16_t * ui16ptr = (uint16_t *)(&Parameters.Kp);
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_KP] 	  = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_KP+1]   = ui16ptr[0];

	ui16ptr = (uint16_t *)(&Parameters.Ki);
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_KI]     = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_KI+1]   = ui16ptr[0];

	ui16ptr = (uint16_t *)(&Parameters.Kd);
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_KD]     = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_KD+1]   = ui16ptr[0];

	ui16ptr = (uint16_t *)(&Parameters.Aux1);
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_AUX1]   = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_AUX1+1] = ui16ptr[0];

	ui16ptr = (uint16_t *)(&Parameters.Aux2);
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_AUX2]   = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_AUX2+1] = ui16ptr[0];

	float rate = 0;
	SJC_GetControlUpdateRateHz(&rate);
	ui16ptr = (uint16_t *)(&rate);
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_RATE]   = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROLLER_PARAMS_RATE+1] = ui16ptr[0];
}

// Holding registers update process ---------------------------------- //

void Net_ExecuteSystemConfigsUpdateCmd(void) {
	if (usRegHoldingBuf[REG_HOLDING_SYSTEM_REBOOT]) {

		// Define digital outputs states
		usRegHoldingBuf[REG_HOLDING_DIGITAL_OUTPUTS_STATE_UPDATE] &= 0x0000;

		// Apply system reboot
	}
}

void Net_ExecuteDigitalOutputsUpdateCmd(void)
{
	if (usRegHoldingBuf[REG_HOLDING_DIGITAL_OUTPUTS_STATE_UPDATE] & DIGITAL_OUTPUTS_UPDATE_MSK) {

		// Define digital outputs states
		usRegHoldingBuf[REG_HOLDING_DIGITAL_OUTPUTS_STATE_UPDATE] &= (~DIGITAL_OUTPUTS_UPDATE_MSK);

		SJC_WriteDigitalOutputs(usRegHoldingBuf[REG_HOLDING_DIGITAL_OUTPUTS_STATE_UPDATE]);
	}
}

void Net_ExecuteControllerConfigsUpdateCmd(void)
{
	if (usRegHoldingBuf[REG_HOLDING_CONTROLLER_MODE_UPDATE] & CONTROLLER_UPDATE_MSK)
	{
		usRegHoldingBuf[REG_HOLDING_CONTROLLER_MODE_UPDATE] &= (~CONTROLLER_UPDATE_MSK);

		// Controller reference
		float reference;
		uint16_t * ptr = (uint16_t*) (&reference);
		ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_REFERENCE + 1];
		ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_REFERENCE];
		SJC_SetControlReference(reference);

		// Controller operating mode
		SJC_SetControlMode(usRegHoldingBuf[REG_HOLDING_CONTROLLER_MODE_UPDATE]);

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
		case SJC_TORQUE_CONTROL:
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
		float base_voltage  = (1.0f*usRegHoldingBuf[REG_HOLDING_ACTUATOR_BASE_VOLTAGE]/100.0f);
		SJC_MotorDrive_SetBaseVoltage(base_voltage );

		/* Reserved for future implementation
		 *
		 * float voltage_limit = (1.0f*usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_VOLTAGE]/100.0f);
		 * float current_limit = (1.0f*usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_CURRENT]/100.0f);
		 * float speed_limit;

		 * uint16_t *ptr = (uint16_t*) (&speed_limit);
		 * ptr[0] = usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_SPEED + 1];
		 * ptr[1] = usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_SPEED];
		 */


#ifdef SJC_DEBUG_PRINT_NET_CMDS
		SJC_DebugOutput_printf("\r\n[NetCmd] Actuator base voltage: %f V", SJC_MotorDrive_GetBaseVoltage(&xSjcJoint[SJC_ACTUATOR_JOINT].MotorDrive));
		SJC_DebugOutput_printf("\r\n[NetCmd] Actuator voltage limit: %f V", SJC_MotorDrive_GetVoltageLimit(&xSjcJoint[SJC_ACTUATOR_JOINT].Limits));
		SJC_DebugOutput_printf("\r\n[NetCmd] Actuator current limit: %f A", SJC_MotorDrive_GetCurrentLimit(&xSjcJoint[SJC_ACTUATOR_JOINT].Limits));
		SJC_DebugOutput_printf("\r\n[NetCmd] Actuator speed limit: %f rd/s", SJC_MotorDrive_GetSpeedLimit(&xSjcJoint[SJC_ACTUATOR_JOINT].Limits));
#endif
	}
}

void Net_ExecuteNetworkConfigsUpdateCmd(void) {
	if (usRegHoldingBuf[REG_HOLDING_NETWORK_CONFIG_UPDATE] & NETWORK_PARAMETERS_UPDATE_MSK)
	{

		// Network parameters
		usRegHoldingBuf[REG_HOLDING_NETWORK_CONFIG_UPDATE] &= (~NETWORK_PARAMETERS_UPDATE_MSK);

		xSjcNetworkConfig_t config;

		uint16_t *ptr = (uint16_t*) (&config.Baudrate);
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

void Net_ExecuteSensorsConfigsUpdateCmd(void) {
	if (usRegHoldingBuf[REG_HOLDING_SENSORS_SETUP_UPDATE] & SENSORS_PARAMETERS_UPDATE_MSK) {

		// Encoders parameters
		usRegHoldingBuf[REG_HOLDING_SENSORS_SETUP_UPDATE] &= (~SENSORS_PARAMETERS_UPDATE_MSK);

		SJC_Encoder_SetResolution(SJC_AUX_JOINT_A, usRegHoldingBuf[REG_HOLDING_AUX_ENCODER_A_RESOLUTION_PPR]);
		SJC_Encoder_SetResolution(SJC_AUX_JOINT_B, usRegHoldingBuf[REG_HOLDING_AUX_ENCODER_B_RESOLUTION_PPR]);
		SJC_Encoder_SetResolution(SJC_ACTUATOR_JOINT, usRegHoldingBuf[REG_HOLDING_ACTUATOR_ENCODER_RESOLUTION_PPR]);

#ifdef SJC_DEBUG_PRINT_NET_CMDS
		SJC_DebugOutput_printf("\r\n[NetCmd] AuxA encoder res.: %u ppr", xSjcJoint[SJC_AUX_JOINT_A].Encoder.Resolution_PPR);
		SJC_DebugOutput_printf("\r\n[NetCmd] AuxB encoder res.: %u ppr", xSjcJoint[SJC_AUX_JOINT_B].Encoder.Resolution_PPR);
		SJC_DebugOutput_printf("\r\n[NetCmd] Actuator encoder res.: %u ppr", xSjcJoint[SJC_ACTUATOR_JOINT].Encoder.Resolution_PPR);
#endif
	}
}

void Net_ExecuteControllerParametersUpdateCmd(void)
{
	if (usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_UPDATE] & CONTROLLER_PARAMETERS_UPDATE_MSK) {

		// Controller parameters
		usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_UPDATE] &= (~CONTROLLER_PARAMETERS_UPDATE_MSK);

		xPidControllerParameters_t params;
		uint16_t *ptr = (uint16_t*) (&params.Kp);
		ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_KP + 1];
		ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_KP];

		ptr = (uint16_t*) (&params.Ki);
		ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_KI + 1];
		ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_KI];

		ptr = (uint16_t*) (&params.Kd);
		ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_KD + 1];
		ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_KD];

		ptr = (uint16_t*) (&params.Aux1);
		ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_AUX1 + 1];
		ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_AUX1];

		ptr = (uint16_t*) (&params.Aux2);
		ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_AUX2 + 1];
		ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_AUX2];

		SJC_SetControlParameters(params);

		// Controller update rate
		float control_rate;
		ptr = (uint16_t*) (&control_rate);
		ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_RATE + 1];
		ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_RATE];
		SJC_SetControlUpdateRateHz(control_rate);

#ifdef SJC_DEBUG_PRINT_NET_CMDS
		SJC_DebugOutput_printf("\r\n[NetCmd] Controller Kp: %f",   params.Kp);
		SJC_DebugOutput_printf("\r\n[NetCmd] Controller Ki: %f",   params.Ki);
		SJC_DebugOutput_printf("\r\n[NetCmd] Controller Kd: %f",   params.Kd);
		SJC_DebugOutput_printf("\r\n[NetCmd] Controller Aux1: %f", params.Aux1);
		SJC_DebugOutput_printf("\r\n[NetCmd] Controller Aux2: %f", params.Aux2);
		SJC_DebugOutput_printf("\r\n[NetCmd] Controller Update Rate: %f Hz", control_rate);
#endif
	}
}


// ------------------------------------------------------------------- //
