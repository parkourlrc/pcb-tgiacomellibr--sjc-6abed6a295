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
#include "actuator.h"
#include "analog_sensors.h"
#include "debug_output.h"
#include "digital_interfaces.h"
#include "encoder.h"
#include "helpers.h"
#include "network.h"
#include "pwm.h"
#include "timeout_timer.h"
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

static xSjcJoint_t xSjcJoint[SJC_JOINTS_SIZE];
// ------------------------------------------------------------------- //

// Private function prototypes --------------------------------------- //
// ------------------------------------------------------------------- //
void InitJointTimers(void);
void InitJointObject(const eSjcJoints_t Joint, const eSjcJointType_t Type, xSjcJoint_t * JointObject);

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
	MX_GPIO_Init();

	MX_DMA_Init();

	InitJointTimers();

	SJC_Timer_Init();

	SJC_DebugOutput_Init();

	InitJointObject(SJC_AUX_JOINT_A, SJC_PASSIVE_JOINT, &xSjcJoint[SJC_AUX_JOINT_A]);
	InitJointObject(SJC_AUX_JOINT_B, SJC_PASSIVE_JOINT, &xSjcJoint[SJC_AUX_JOINT_B]);
	InitJointObject(SJC_ACTUATOR_JOINT, SJC_ACTIVE_JOINT, &xSjcJoint[SJC_ACTUATOR_JOINT]);

	SJC_SetVersions();
}

eSjcReturnType_t SJC_InitOsKernel(void)
{
	/* Init scheduler */
	osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */

	/* Init Data structures */
	SJC_TaskCommon_InitThreads();
	/* Init Tasks */

	/* Start scheduler */
	osKernelStart();

	for (;;);

	return SJC_RET_OK;
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

eSjcReturnType_t SJC_GetControlMode(eSjcControlMode_t *config) {
	*config = xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Mode;
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_SetControlMode(const eSjcControlMode_t config)
{
	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Mode = config;

	switch (xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Mode) {
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

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_GetControlReference(float * reference)
{
	*reference = xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Reference;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_SetControlReference(const float reference)
{
	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Reference = reference;

	SJC_DebugOutput_printf("\r\n[NetCmd] Controller reference: %f", reference);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_GetControlError(float * controller_error)
{
	*controller_error = xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Errors.Error;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_SetControlError(const float controller_error)
{
	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Errors.Error = controller_error;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_GetControlUpdateRateHz(float * config)
{
	*config = xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Rate;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_SetControlUpdateRateHz(const float config)
{

	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Rate = SJC_Helpers_RealClamp(config, SJC_CONTROLLER_MIN_UPDATE_RATE_HZ, SJC_CONTROLLER_MAX_UPDATE_RATE_HZ);

	SJC_DebugOutput_printf("\r\n[NetCmd] Controller Update Rate: %f", xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Rate);

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_GetControlUpdatePeriodMs(uint32_t * config)
{
	*config = ((uint32_t) (1000/xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Rate));

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_GetControlParameters(xControllerParameters_t * Parameters)
{
	Parameters->Kp = xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Kp;
	Parameters->Ki = xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Ki;
	Parameters->Kd = xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Kd;
	Parameters->Aux1 = xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Aux1;
	Parameters->Aux2 = xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Aux2;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_SetControlParameters(const xControllerParameters_t Parameters)
{
	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Kp = Parameters.Kp;
	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Ki = Parameters.Ki;
	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Kd = Parameters.Kd;
	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Aux1 = Parameters.Aux1;
	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Aux2 = Parameters.Aux2;

	SJC_DebugOutput_printf("\r\n[NetCmd] Controller Kp: %f",   xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Kp);
	SJC_DebugOutput_printf("\r\n[NetCmd] Controller Ki: %f",   xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Ki);
	SJC_DebugOutput_printf("\r\n[NetCmd] Controller Kd: %f",   xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Kd);
	SJC_DebugOutput_printf("\r\n[NetCmd] Controller Aux1: %f", xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Aux1);
	SJC_DebugOutput_printf("\r\n[NetCmd] Controller Aux2: %f", xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Aux2);

	return SJC_RET_OK;
}


void SJC_GetCommands(void)
{
	Net_ExecuteSystemConfigsUpdateCmd();
	Net_ExecuteDigitalOutputsUpdateCmd();
	Net_ExecuteNetworkConfigsUpdateCmd();
	Net_ExecuteSensorsConfigsUpdateCmd();
	Net_ExecuteActuatorConfigsUpdateCmd();
	Net_ExecuteControllerConfigsUpdateCmd();
	Net_ExecuteControllerParametersUpdateCmd();
}

void SJC_SetDataRegisters(void)
{
	Net_UpdateRuntimeInfoRegisters();
	Net_UpdateNetworkParametersRegisters();
	Net_UpdateSensorsParametersRegisters();
	Net_UpdateActuatorParametersRegisters();
	Net_UpdateControllerParametersRegisters();

//	write_system_uptime_reg(SJC_Timer_GetTimeMs());
//	write_system_voltage_reg(xSjcJoint[SJC_ACTUATOR_JOINT].PSUVoltage.Measurement);
//	write_system_error_flags_reg(0x00);
////
//	write_control_mode_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Mode);
//	write_control_rate_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Rate);
//	write_control_error_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Errors.Error);
//	write_control_reference_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Reference);
//
//	write_control_setup_Kp_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Kp);
//	write_control_setup_Ki_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Ki);
//	write_control_setup_Kd_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Kd);
//	write_control_setup_Aux1_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Aux1);
//	write_control_setup_Aux2_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Aux2);
//
//	write_joint_displacement_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Encoder.Counts);
//	write_joint_output_current_reg(xSjcJoint[SJC_ACTUATOR_JOINT].OutputCurrent.Measurement);
//	write_joint_output_voltage_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Actuator.OutputVoltage);
//	write_joint_actuator_state_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Actuator.Active);

//	write_aux_A_displacement_reg(xSjcJoint[SJC_AUX_JOINT_A].Encoder.Counts);
//	write_aux_B_displacement_reg(xSjcJoint[SJC_AUX_JOINT_B].Encoder.Counts);
//
//	uint8_t states;
//	SJC_ReadDigitalInputs(&states);
//	write_digital_inputs_state_reg(states);
//
//	SJC_ReadDigitalOutputs(&states);
//	write_digital_outputs_state_reg(states);
}

eSjcReturnType_t SJC_InitAdc(uint32_t * buffer, const uint32_t buffer_size)
{
	SJC_AnalogSensors_Configure(buffer, buffer_size);
	MX_TIM4_Init();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_EnableAdc(void)
{
	return SJC_AnalogSensors_StartMeasurements();
}

eSjcReturnType_t SJC_DisableAdc(void)
{
	return SJC_AnalogSensors_StopMeasurements();
}

eSjcReturnType_t SJC_EnableJointActuators(void)
{
	SJC_Actuator_Enable(&xSjcJoint[SJC_ACTUATOR_JOINT].Actuator);
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_DisableJointActuators(void)
{
	SJC_Actuator_Disable(&xSjcJoint[SJC_ACTUATOR_JOINT].Actuator);
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_GetJointActuatorState(const eSjcJoints_t sel, bool * state)
{
	eSjcReturnType_t ret = SJC_RET_OK;

	switch (sel) {
	case SJC_ACTUATOR_JOINT:
		*state = xSjcJoint[SJC_ACTUATOR_JOINT].Actuator.Active;
		break;
	case SJC_AUX_JOINT_A:
		*state = xSjcJoint[SJC_AUX_JOINT_A].Actuator.Active;
		break;
	case SJC_AUX_JOINT_B:
		*state = xSjcJoint[SJC_AUX_JOINT_B].Actuator.Active;
		break;
	default:
		*state = xSjcJoint[SJC_AUX_JOINT_B].Actuator.Active;
		ret = SJC_RET_NOK;
		break;
	}

	return ret;
}

eSjcReturnType_t SJC_SetJointActuatorVoltage(float voltage)
{
	eSjcReturnType_t ret = SJC_Actuator_SetVoltage(&xSjcJoint[SJC_ACTUATOR_JOINT].Actuator, voltage);
	return ret;
}

eSjcReturnType_t SJC_GetJointActuatorFrequency(uint32_t * freq)
{
	*freq = xSjcJoint[SJC_ACTUATOR_JOINT].Actuator.PWM.Frequency;
	return SJC_RET_OK;
}

eSjcReturnType_t SJC_SetJointActuatorFilteredPsuVoltage(const float value)
{
	xSjcJoint[SJC_ACTUATOR_JOINT].PSUVoltage.Measurement = value;
}

eSjcReturnType_t SJC_SetJointActuatorFilteredOutputCurrent(const float value)
{
	xSjcJoint[SJC_ACTUATOR_JOINT].OutputCurrent.Measurement = value;
}

eSjcReturnType_t SJC_ReadDigitalInputs(uint16_t * states)
{
	eSjcReturnType_t ret = SJC_DigitalInterfaces_ReadInputStates(xSjcJoint[SJC_ACTUATOR_JOINT].DigitalSensors);
	*states = 0;
	*states |= xSjcJoint[SJC_ACTUATOR_JOINT].DigitalSensors[SJC_JOINT_SWITCH_A].State << SJC_JOINT_SWITCH_A;
	*states |= xSjcJoint[SJC_ACTUATOR_JOINT].DigitalSensors[SJC_JOINT_SWITCH_B].State << SJC_JOINT_SWITCH_B;
	*states |= xSjcJoint[SJC_ACTUATOR_JOINT].DigitalSensors[SJC_ISOLATED_INPUT_A].State << SJC_ISOLATED_INPUT_A;
	*states |= xSjcJoint[SJC_ACTUATOR_JOINT].DigitalSensors[SJC_ISOLATED_INPUT_B].State << SJC_ISOLATED_INPUT_B;
	return ret;
}

eSjcReturnType_t SJC_ReadDigitalOutputs(uint16_t * states)
{
	eSjcReturnType_t ret = SJC_DigitalInterfaces_ReadOutputStates(xSjcJoint[SJC_ACTUATOR_JOINT].DigitalActuators);
	*states = 0;
	*states |= xSjcJoint[SJC_ACTUATOR_JOINT].DigitalActuators[SJC_DIGITAL_OUTPUT_A].State << SJC_DIGITAL_OUTPUT_A;
	*states |= xSjcJoint[SJC_ACTUATOR_JOINT].DigitalActuators[SJC_DIGITAL_OUTPUT_B].State << SJC_DIGITAL_OUTPUT_B;
	return ret;
}

eSjcReturnType_t SJC_WriteDigitalOutputs(const uint8_t states)
{
	eSjcReturnType_t ret = SJC_DigitalInterfaces_WriteOutputStates(xSjcJoint[SJC_ACTUATOR_JOINT].DigitalActuators, states);
	return ret;
}

eSjcReturnType_t SJC_ReadDigitalInput(const eSjcDigitalInputs_t Input, bool * state)
{
	eSjcReturnType_t ret = SJC_DigitalInterfaces_ReadInputState(&xSjcJoint[SJC_ACTUATOR_JOINT].DigitalSensors[Input]);
	*state = xSjcJoint[SJC_ACTUATOR_JOINT].DigitalSensors[Input].State;

	return ret;
}

eSjcReturnType_t SJC_ReadDigitalOutput(const eSjcDigitalOutputs_t Output, bool *state)
{
	eSjcReturnType_t ret = SJC_DigitalInterfaces_ReadOutputState(&xSjcJoint[SJC_ACTUATOR_JOINT].DigitalActuators[Output]);
	*state = xSjcJoint[SJC_ACTUATOR_JOINT].DigitalActuators[Output].State;

	return ret;
}

eSjcReturnType_t SJC_WriteDigitalOutput(const eSjcDigitalOutputs_t Output, const bool state)
{
	eSjcReturnType_t ret = SJC_DigitalInterfaces_WriteOutputState(&xSjcJoint[SJC_ACTUATOR_JOINT].DigitalActuators[Output], state);
	return ret;
}

eSjcReturnType_t SJC_ToggleDigitalOutput(const eSjcDigitalOutputs_t Output)
{
	eSjcReturnType_t ret = SJC_DigitalInterfaces_ToggleOutputState(&xSjcJoint[SJC_ACTUATOR_JOINT].DigitalActuators[Output]);
	return ret;
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
void InitJointObject(const eSjcJoints_t Joint, const eSjcJointType_t Type, xSjcJoint_t * JointObject)
{
	JointObject->Controller.Mode = (Type == SJC_PASSIVE_JOINT) ? SJC_CONTROL_DISABLED : SJC_OPEN_LOOP_CONTROL;
	JointObject->Controller.Rate = (Type == SJC_PASSIVE_JOINT) ? 0 : SJC_CONTROLLER_MAX_UPDATE_RATE_HZ;
	JointObject->Controller.Reference = 0;
	JointObject->Controller.Errors.Proportional = 0;
	JointObject->Controller.Errors.Integral = 0;
	JointObject->Controller.Errors.Derivative = 0;
	JointObject->Controller.Errors.Error = 0;
	SJC_Encoder_init(&JointObject->Encoder, Joint);
	SJC_Encoder_Enable(&JointObject->Encoder);

	if (Type == SJC_ACTIVE_JOINT)
	{
		SJC_DigitalInterfaces_init(JointObject->DigitalSensors, SJC_DIGITAL_INPUT_PORT);
		SJC_DigitalInterfaces_init(JointObject->DigitalActuators, SJC_DIGITAL_OUTPUT_PORT);
		SJC_AnalogSensors_Init(&JointObject[SJC_ACTUATOR_JOINT].PSUVoltage);
		SJC_AnalogSensors_Init(&JointObject[SJC_ACTUATOR_JOINT].OutputCurrent);

		JointObject[SJC_ACTUATOR_JOINT].Limits.Speed.Max = SJC_ACTUATOR_LIMIT_MAX_SPEED;
		JointObject[SJC_ACTUATOR_JOINT].Limits.Speed.Min = -SJC_ACTUATOR_LIMIT_MAX_SPEED;

		JointObject[SJC_ACTUATOR_JOINT].Limits.Current.Max = SJC_ACTUATOR_LIMIT_MAX_CURRENT;
		JointObject[SJC_ACTUATOR_JOINT].Limits.Current.Min = -SJC_ACTUATOR_LIMIT_MAX_CURRENT;

		JointObject[SJC_ACTUATOR_JOINT].Limits.Voltage.Max = SJC_ACTUATOR_LIMIT_MAX_VOLTAGE;
		JointObject[SJC_ACTUATOR_JOINT].Limits.Voltage.Min = -SJC_ACTUATOR_LIMIT_MAX_VOLTAGE;

		SJC_Actuator_Init(&xSjcJoint[SJC_ACTUATOR_JOINT].Actuator, PWM_FREQ_20kHz, 24.0);
	}
}

void InitJointTimers(void)
{
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM8_Init();
}

// Input registers update process ------------------------------------ //
void Net_UpdateRuntimeInfoRegisters(void)
{
	float fAux;
	uint16_t ui16Aux;
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
	SJC_Encoder_GetLastDisplacementRadians(&xSjcJoint[SJC_ACTUATOR_JOINT].Encoder, &fAux);
	ui16ptr = ((uint16_t*)(&fAux));
	usRegInputBuf[REG_INPUT_ACTUATOR_DISPLACEMENT] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_ACTUATOR_DISPLACEMENT+1] = ui16ptr[0];

	fAux = 0;
	SJC_Encoder_GetLastDisplacementRadians(&xSjcJoint[SJC_AUX_JOINT_A].Encoder, &fAux);
	ui16ptr = ((uint16_t*)(&fAux));
	usRegInputBuf[REG_INPUT_AUX_ENCODER_A_DISPLACEMENT] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_AUX_ENCODER_A_DISPLACEMENT+1] = ui16ptr[0];

	fAux = 0;
	SJC_Encoder_GetLastDisplacementRadians(&xSjcJoint[SJC_AUX_JOINT_B].Encoder, &fAux);
	ui16ptr = ((uint16_t*)(&fAux));
	usRegInputBuf[REG_INPUT_AUX_ENCODER_B_DISPLACEMENT] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_AUX_ENCODER_B_DISPLACEMENT+1] = ui16ptr[0];

	fAux = SJC_Actuator_GetVoltage(&xSjcJoint[SJC_ACTUATOR_JOINT].Actuator);
	usRegInputBuf[REG_INPUT_ACTUATOR_VOLTAGE] = fAux*100.0f;

//	usRegInputBuf[REG_INPUT_ACTUATOR_CURRENT] = fAux*100.0f;

	bool state;
	SJC_GetJointActuatorState(SJC_ACTUATOR_JOINT, &state);
	usRegInputBuf[REG_INPUT_ACTUATOR_STATE] = state;

	ui16Aux = 0;
	SJC_ReadDigitalInputs(&ui16Aux);
	usRegInputBuf[REG_INPUT_DIGITAL_INPUTS] = ui16Aux;

	ui16Aux = 0;
	SJC_ReadDigitalOutputs(&ui16Aux);
	usRegInputBuf[REG_INPUT_DIGITAL_OUTPUTS] = ui16Aux;

	usRegInputBuf[REG_INPUT_SYSTEM_VOLTAGE] = ( (uint16_t) (voltage*100) );
	usRegInputBuf[REG_INPUT_SYSTEM_ERROR_FLAGS] = flags;




	usRegInputBuf[REG_INPUT_DIGITAL_INPUTS] = state;
	usRegInputBuf[REG_INPUT_DIGITAL_OUTPUTS] = state;
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
	usRegInputBuf[REG_INPUT_ACTUATOR_ENCODER_RESOLUTION_PPR] = xSjcJoint[SJC_ACTUATOR_JOINT].Encoder.Resolution_PPR;
	usRegInputBuf[REG_INPUT_AUX_A_ENCODER_RESOLUTION_PPR] = xSjcJoint[SJC_AUX_JOINT_A].Encoder.Resolution_PPR;
	usRegInputBuf[REG_INPUT_AUX_B_ENCODER_RESOLUTION_PPR] = xSjcJoint[SJC_AUX_JOINT_B].Encoder.Resolution_PPR;
}

void Net_UpdateActuatorParametersRegisters(void)
{
	uint16_t * ui16ptr = ((uint16_t*)(&xSjcJoint[SJC_ACTUATOR_JOINT].Limits.Speed));
	usRegInputBuf[REG_INPUT_ACTUATOR_LIMITS_SPEED] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_ACTUATOR_LIMITS_SPEED+1] = ui16ptr[0];
	usRegInputBuf[REG_INPUT_ACTUATOR_BASE_VOLTAGE] = xSjcJoint[SJC_ACTUATOR_JOINT].Actuator.BaseVoltage*100.0f;
	usRegInputBuf[REG_INPUT_ACTUATOR_LIMITS_VOLTAGE] = xSjcJoint[SJC_ACTUATOR_JOINT].Limits.Voltage.Max*100.0f;
	usRegInputBuf[REG_INPUT_ACTUATOR_LIMITS_CURRENT] = xSjcJoint[SJC_ACTUATOR_JOINT].Limits.Current.Max*100.0f;
}

void Net_UpdateControllerParametersRegisters(void)
{
	xControllerParameters_t Parameters;
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
	}
}

void Net_ExecuteActuatorConfigsUpdateCmd(void)
{
	if (usRegHoldingBuf[REG_HOLDING_ACTUATOR_SETUP_UPDATE] & ACTUATOR_CONFIGS_UPDATE_MSK)
	{
		usRegHoldingBuf[REG_HOLDING_ACTUATOR_SETUP_UPDATE] &= (~ACTUATOR_CONFIGS_UPDATE_MSK);

		// Controller reference
		float base_voltage  = (1.0f*usRegHoldingBuf[REG_HOLDING_ACTUATOR_BASE_VOLTAGE]/100.0f);
		float voltage_limit = (1.0f*usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_VOLTAGE]/100.0f);
		float current_limit = (1.0f*usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_CURRENT]/100.0f);
		float speed_limit;

		uint16_t *ptr = (uint16_t*) (&speed_limit);
		ptr[0] = usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_SPEED + 1];
		ptr[1] = usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_SPEED];

		SJC_Actuator_SetBaseVoltage(&xSjcJoint[SJC_ACTUATOR_JOINT].Actuator, base_voltage );

		SJC_Actuator_SetSpeedLimit(&xSjcJoint[SJC_ACTUATOR_JOINT].Limits, speed_limit);
		SJC_Actuator_SetCurrentLimit(&xSjcJoint[SJC_ACTUATOR_JOINT].Limits, current_limit);
		SJC_Actuator_SetVoltageLimit(&xSjcJoint[SJC_ACTUATOR_JOINT].Limits, voltage_limit);

		SJC_DebugOutput_printf("\r\n[NetCmd] Actuator base voltage: %f V", SJC_Actuator_GetBaseVoltage(&xSjcJoint[SJC_ACTUATOR_JOINT].Actuator));
		SJC_DebugOutput_printf("\r\n[NetCmd] Actuator voltage limit: %f V", SJC_Actuator_GetVoltageLimit(&xSjcJoint[SJC_ACTUATOR_JOINT].Limits));
		SJC_DebugOutput_printf("\r\n[NetCmd] Actuator current limit: %f A", SJC_Actuator_GetCurrentLimit(&xSjcJoint[SJC_ACTUATOR_JOINT].Limits));
		SJC_DebugOutput_printf("\r\n[NetCmd] Actuator speed limit: %f rd/s", SJC_Actuator_GetSpeedLimit(&xSjcJoint[SJC_ACTUATOR_JOINT].Limits));
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

		SJC_DebugOutput_printf("\r\n[NetCmd] Network address: %u", config.Address);
		SJC_DebugOutput_printf("\r\n[NetCmd] Network parity: %c", config.Parity);
		SJC_DebugOutput_printf("\r\n[NetCmd] Network baudrate: %lu bps", config.Baudrate);

		// Write to flash
		// Reboot
	}
}

void Net_ExecuteSensorsConfigsUpdateCmd(void) {
	if (usRegHoldingBuf[REG_HOLDING_SENSORS_SETUP_UPDATE] & SENSORS_PARAMETERS_UPDATE_MSK) {

		// Encoders parameters
		usRegHoldingBuf[REG_HOLDING_SENSORS_SETUP_UPDATE] &= (~SENSORS_PARAMETERS_UPDATE_MSK);

		xSjcJoint[SJC_AUX_JOINT_A].Encoder.Resolution_PPR = usRegHoldingBuf[REG_HOLDING_AUX_ENCODER_A_RESOLUTION_PPR];
		xSjcJoint[SJC_AUX_JOINT_B].Encoder.Resolution_PPR = usRegHoldingBuf[REG_HOLDING_AUX_ENCODER_B_RESOLUTION_PPR];
		xSjcJoint[SJC_ACTUATOR_JOINT].Encoder.Resolution_PPR = usRegHoldingBuf[REG_HOLDING_ACTUATOR_ENCODER_RESOLUTION_PPR];

		SJC_DebugOutput_printf("\r\n[NetCmd] AuxA encoder res.: %u ppr", xSjcJoint[SJC_AUX_JOINT_A].Encoder.Resolution_PPR);
		SJC_DebugOutput_printf("\r\n[NetCmd] AuxB encoder res.: %u ppr", xSjcJoint[SJC_AUX_JOINT_B].Encoder.Resolution_PPR);
		SJC_DebugOutput_printf("\r\n[NetCmd] Actuator encoder res.: %u ppr", xSjcJoint[SJC_ACTUATOR_JOINT].Encoder.Resolution_PPR);
	}
}

void Net_ExecuteControllerParametersUpdateCmd(void)
{
	if (usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_UPDATE] & CONTROLLER_PARAMETERS_UPDATE_MSK) {

		// Controller parameters
		usRegHoldingBuf[REG_HOLDING_CONTROLLER_PARAMS_UPDATE] &= (~CONTROLLER_PARAMETERS_UPDATE_MSK);

		xControllerParameters_t params;
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
	}
}


// ------------------------------------------------------------------- //
