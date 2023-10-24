/*
 * motor.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */


// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //

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
void write_system_uptime_reg(const uint32_t uptime);
void write_system_voltage_reg(const float voltage);
void write_system_error_flags_reg(const uint16_t flags);

void write_control_mode_reg(const eSjcControlMode_t mode);
void write_control_rate_reg(const uint16_t rate);
void write_control_error_reg(const float error);
void write_control_reference_reg(const float reference);

void write_control_setup_Kp_reg(const float value);
void write_control_setup_Ki_reg(const float value);
void write_control_setup_Kd_reg(const float value);
void write_control_setup_Aux1_reg(const float value);
void write_control_setup_Aux2_reg(const float value);

void write_joint_displacement_reg(const float displacement);
void write_joint_output_voltage_reg(const float voltage);
void write_joint_output_current_reg(const float current);
void write_joint_actuator_state_reg(const uint16_t state);

void write_joint_encoder_resolution_reg(const uint16_t resolution);
void write_aux_A_encoder_resolution_reg(const uint16_t resolution);
void write_aux_B_encoder_resolution_reg(const uint16_t resolution);

void write_aux_A_displacement_reg(const float displacement);
void write_aux_B_displacement_reg(const float displacement);

void write_digital_inputs_state_reg(const uint16_t state);
void write_digital_outputs_state_reg(const uint16_t state);

void write_actuator_base_voltage_reg(const float voltage);
void write_actuator_speed_limit_reg(const float limit);
void write_actuator_voltage_limit_reg(const float limit);
void write_actuator_current_limit_reg(const float limit);

// Holding registers update process ---------------------------------- //
void read_control_mode_reg(void);
void read_control_reference_reg(void);
void read_digital_outputs_command_reg(void);
void read_actuator_base_voltage_reg(void);
void read_actuator_voltage_limit_reg(void);
void read_actuator_current_limit_reg(void);
void read_actuator_speed_limit_reg(void);
void read_control_setup_Kp_reg(void);
void read_control_setup_Ki_reg(void);
void read_control_setup_Kd_reg(void);
void read_control_setup_Aux1_reg(void);
void read_control_setup_Aux2_reg(void);
void read_joint_encoder_setup_resolution_reg(void);
void read_aux_A_encoder_setup_resolution_reg(void);
void read_aux_B_encoder_setup_resolution_reg(void);
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
}

eSjcReturnType_t SJC_InitOsTasks(void)
{
	/* Init scheduler */
	osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */

	/* Init Data structures */
	SJC_TaskCommon_InitThreads();
	/* Init Tasks */

	/* Start scheduler */
	osKernelStart();

	return SJC_RET_OK;
}

void SJC_PresentSystemInfo(void)
{
	SJC_DebugOutput_PresentSystemInfo();
}

eSjcReturnType_t SJC_Delay(const eSjcDelayType_t type, const uint32_t value)
{
	eSjcReturnType_t ret = SJC_RET_OK;

	if (value == 0)
	{
		return SJC_RET_NOK;
	}

	switch (type) {
	case SJC_TIME_DELAY_MS:
		SJC_Timer_DelayMs(value);
		break;
	case SJC_TIME_DELAY_SEC:
		SJC_Timer_DelaySec(value);
		break;
	default:
		ret = SJC_RET_NOK;
		break;
	}
	return ret;
}

eSjcReturnType_t SJC_GetTime(const eSjcDelayType_t type, uint32_t *value)
{
	eSjcReturnType_t ret = SJC_RET_OK;

		switch (type) {
		case SJC_TIME_DELAY_MS:
			*value = SJC_Timer_GetTimeMs();
			break;
		case SJC_TIME_DELAY_SEC:
			*value = SJC_Timer_GetTimeSec();
			break;
		default:
			ret = SJC_RET_NOK;
			break;
		}
		return ret;
}

eSjcControlMode_t SJC_GetControlMode(void)
{
	return xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Mode;
}

uint32_t SJC_GetControlUpdateRateHz(void)
{
	return (xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Rate);
}

uint32_t SJC_GetControlUpdatePeriodMs(void)
{
	return (1000/xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Rate);
}

void SJC_ReadNetworkCommands(void)
{
	read_control_mode_reg();
	read_control_reference_reg();
	read_digital_outputs_command_reg();
	read_actuator_voltage_limit_reg();
	read_actuator_current_limit_reg();
	read_actuator_speed_limit_reg();
	read_control_setup_Kp_reg();
	read_control_setup_Ki_reg();
	read_control_setup_Kd_reg();
	read_control_setup_Aux1_reg();
	read_control_setup_Aux2_reg();
	read_joint_encoder_setup_resolution_reg();
	read_aux_A_encoder_setup_resolution_reg();
	read_aux_B_encoder_setup_resolution_reg();
}

void SJC_UpdateNetworkRegisters(void)
{
	write_system_uptime_reg(SJC_Timer_GetTimeMs());
	write_system_voltage_reg(xSjcJoint[SJC_ACTUATOR_JOINT].PSUVoltage.Measurement);
	write_system_error_flags_reg(0x00);
//
	write_control_mode_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Mode);
	write_control_rate_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Rate);
	write_control_error_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Errors.Error);
	write_control_reference_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Reference);

	write_control_setup_Kp_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Kp);
	write_control_setup_Ki_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Ki);
	write_control_setup_Kd_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Kd);
	write_control_setup_Aux1_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Aux1);
	write_control_setup_Aux2_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Aux2);

	write_joint_displacement_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Encoder.Counts);
	write_joint_output_current_reg(xSjcJoint[SJC_ACTUATOR_JOINT].OutputCurrent.Measurement);
	write_joint_output_voltage_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Actuator.OutputVoltage);
	write_joint_actuator_state_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Actuator.Active);

	write_joint_encoder_resolution_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Encoder.Resolution_PPR);
	write_aux_A_encoder_resolution_reg(xSjcJoint[SJC_AUX_JOINT_A].Encoder.Resolution_PPR);
	write_aux_B_encoder_resolution_reg(xSjcJoint[SJC_AUX_JOINT_B].Encoder.Resolution_PPR);

	write_aux_A_displacement_reg(xSjcJoint[SJC_AUX_JOINT_A].Encoder.Counts);
	write_aux_B_displacement_reg(xSjcJoint[SJC_AUX_JOINT_B].Encoder.Counts);

	uint8_t states;
	SJC_ReadDigitalInputs(&states);
	write_digital_inputs_state_reg(states);

	SJC_ReadDigitalOutputs(&states);
	write_digital_outputs_state_reg(states);

	write_actuator_speed_limit_reg(xSjcJoint[SJC_ACTUATOR_JOINT].Limits.Speed.Max);
	write_actuator_voltage_limit_reg((100*xSjcJoint[SJC_ACTUATOR_JOINT].Limits.OutputVoltage.Max));
	write_actuator_current_limit_reg((100*xSjcJoint[SJC_ACTUATOR_JOINT].Limits.OutputCurrent.Max));
}

eSjcReturnType_t SJC_InitAdc(void)
{
	MX_ADC1_Init();
	MX_TIM4_Init();

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_EnableAdc(uint32_t * buffer)
{
	return SJC_AnalogSensors_StartMeasurements(buffer);
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

eSjcReturnType_t SJC_GetJointStates(const eSjcJoints_t sel, xSjcJoint_t *Joint)
{
	eSjcReturnType_t ret = SJC_RET_OK;

	switch (sel) {
	case SJC_ACTUATOR_JOINT:
		*Joint = xSjcJoint[SJC_ACTUATOR_JOINT];
		break;
	case SJC_AUX_JOINT_A:
		*Joint = xSjcJoint[SJC_AUX_JOINT_A];
		break;
	case SJC_AUX_JOINT_B:
		*Joint = xSjcJoint[SJC_AUX_JOINT_B];
		break;
	default:
		ret = SJC_RET_NOK;
		break;
	}

	return ret;
}

eSjcReturnType_t SJC_SetJointActuatorVoltage(float voltage)
{
	eSjcReturnType_t ret = SJC_Actuator_SetOutputVoltage(&xSjcJoint[SJC_ACTUATOR_JOINT].Actuator, voltage);
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

eSjcReturnType_t SJC_ReadDigitalInputs(uint8_t * states)
{
	eSjcReturnType_t ret = SJC_DigitalInterfaces_ReadInputStates(xSjcJoint[SJC_ACTUATOR_JOINT].DigitalSensors);
	*states = 0;
	*states |= xSjcJoint[SJC_ACTUATOR_JOINT].DigitalSensors[SJC_JOINT_SWITCH_A].State << SJC_JOINT_SWITCH_A;
	*states |= xSjcJoint[SJC_ACTUATOR_JOINT].DigitalSensors[SJC_JOINT_SWITCH_B].State << SJC_JOINT_SWITCH_B;
	*states |= xSjcJoint[SJC_ACTUATOR_JOINT].DigitalSensors[SJC_ISOLATED_INPUT_A].State << SJC_ISOLATED_INPUT_A;
	*states |= xSjcJoint[SJC_ACTUATOR_JOINT].DigitalSensors[SJC_ISOLATED_INPUT_B].State << SJC_ISOLATED_INPUT_B;
	return ret;
}

eSjcReturnType_t SJC_ReadDigitalOutputs(uint8_t * states)
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
		SJC_DigitalInterfaces_init(JointObject->DigitalSensors, SJC_DIGITAL_INOUT_PORT);
		SJC_DigitalInterfaces_init(JointObject->DigitalActuators, SJC_DIGITAL_OUTPUT_PORT);
		SJC_AnalogSensors_Init(&JointObject[SJC_ACTUATOR_JOINT].PSUVoltage);
		SJC_AnalogSensors_Init(&JointObject[SJC_ACTUATOR_JOINT].OutputCurrent);

		JointObject[SJC_ACTUATOR_JOINT].Limits.Speed.Max = SJC_ACTUATOR_LIMIT_MAX_SPEED;
		JointObject[SJC_ACTUATOR_JOINT].Limits.Speed.Min = -SJC_ACTUATOR_LIMIT_MAX_SPEED;

		JointObject[SJC_ACTUATOR_JOINT].Limits.OutputCurrent.Max = SJC_ACTUATOR_LIMIT_MAX_CURRENT;
		JointObject[SJC_ACTUATOR_JOINT].Limits.OutputCurrent.Min = -SJC_ACTUATOR_LIMIT_MAX_CURRENT;

		JointObject[SJC_ACTUATOR_JOINT].Limits.OutputVoltage.Max = SJC_ACTUATOR_LIMIT_MAX_VOLTAGE;
		JointObject[SJC_ACTUATOR_JOINT].Limits.OutputVoltage.Min = -SJC_ACTUATOR_LIMIT_MAX_VOLTAGE;

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
void write_system_uptime_reg(const uint32_t uptime)
{
	uint16_t * ui16ptr = ((uint16_t*)(&uptime)) ;
	usRegInputBuf[REG_INPUT_SYSTEM_UPTIME] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_SYSTEM_UPTIME+1] = ui16ptr[0];
}

void write_system_voltage_reg(const float voltage)
{
	usRegInputBuf[REG_INPUT_SYSTEM_VOLTAGE] = ( (uint16_t) (voltage*100) );
}

void write_system_error_flags_reg(const uint16_t flags)
{
	usRegInputBuf[REG_INPUT_SYSTEM_ERROR_FLAGS] = flags;
}

void write_control_mode_reg(const eSjcControlMode_t mode)
{
	usRegInputBuf[REG_INPUT_CONTROL_MODE] = mode;
}

void write_control_rate_reg(const uint16_t rate)
{
	usRegInputBuf[REG_INPUT_CONTROL_SETUP_RATE] = rate;
}

void write_control_error_reg(const float error)
{
	uint16_t * ui16ptr = ((uint16_t*)(&error));
	usRegInputBuf[REG_INPUT_CONTROL_ERROR] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROL_ERROR+1] = ui16ptr[0];
}

void write_control_reference_reg(const float reference)
{
	uint16_t * ui16ptr = ((uint16_t*)(&reference));
	usRegInputBuf[REG_INPUT_CONTROL_REFERENCE] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROL_REFERENCE+1] = ui16ptr[0];
}

void write_control_setup_Kp_reg(const float value)
{
	uint16_t * ui16ptr = ((uint16_t*)(&value)) ;
	usRegInputBuf[REG_INPUT_CONTROL_SETUP_KP] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROL_SETUP_KP+1] = ui16ptr[0];
}

void write_control_setup_Ki_reg(const float value)
{
	uint16_t * ui16ptr = ((uint16_t*)(&value)) ;
	usRegInputBuf[REG_INPUT_CONTROL_SETUP_KI] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROL_SETUP_KI+1] = ui16ptr[0];
}

void write_control_setup_Kd_reg(const float value)
{
	uint16_t * ui16ptr = ((uint16_t*)(&value)) ;
	usRegInputBuf[REG_INPUT_CONTROL_SETUP_KD] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROL_SETUP_KD+1] = ui16ptr[0];

}

void write_control_setup_Aux1_reg(const float value)
{
	uint16_t * ui16ptr = ((uint16_t*)(&value)) ;
	usRegInputBuf[REG_INPUT_CONTROL_SETUP_AUX1] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROL_SETUP_AUX1+1] = ui16ptr[0];

}

void write_control_setup_Aux2_reg(const float value)
{
	uint16_t * ui16ptr = ((uint16_t*)(&value)) ;
	usRegInputBuf[REG_INPUT_CONTROL_SETUP_AUX2] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_CONTROL_SETUP_AUX2+1] = ui16ptr[0];

}

void write_joint_displacement_reg(const float displacement)
{
	uint16_t * ui16ptr = ((uint16_t*)(&displacement));
	usRegInputBuf[REG_INPUT_JOINT_DISPLACEMENT] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_JOINT_DISPLACEMENT+1] = ui16ptr[0];
}

void write_joint_output_voltage_reg(const float voltage)
{
	usRegInputBuf[REG_INPUT_JOINT_OUTPUT_VOLTAGE] = ( (uint16_t) (voltage*100) );
}

void write_joint_output_current_reg(const float current)
{
	usRegInputBuf[REG_INPUT_JOINT_OUTPUT_CURRENT] = ( (uint16_t) (current*100) );
}

void write_joint_actuator_state_reg(const uint16_t state)
{
	usRegInputBuf[REG_INPUT_ACTUATOR_STATE] = state;
}

void write_joint_encoder_resolution_reg(const uint16_t resolution)
{
	usRegInputBuf[REG_INPUT_ENCODER_SETUP_JOINT_RESOLUTION_PPR] = resolution;
}

void write_aux_A_encoder_resolution_reg(const uint16_t resolution)
{
	usRegInputBuf[REG_INPUT_ENCODER_SETUP_AUX_A_RESOLUTION_PPR] = resolution;
}

void write_aux_B_encoder_resolution_reg(const uint16_t resolution)
{
	usRegInputBuf[REG_INPUT_ENCODER_SETUP_AUX_B_RESOLUTION_PPR] = resolution;
}

void write_aux_A_displacement_reg(const float displacement)
{
	uint16_t * ui16ptr = ((uint16_t*)(&displacement)) ;
	usRegInputBuf[REG_INPUT_AUX_A_DISPLACEMENT] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_AUX_A_DISPLACEMENT+1] = ui16ptr[0];
}

void write_aux_B_displacement_reg(const float displacement)
{
	uint16_t * ui16ptr = ((uint16_t*)(&displacement)) ;
	usRegInputBuf[REG_INPUT_AUX_B_DISPLACEMENT] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_AUX_B_DISPLACEMENT+1] = ui16ptr[0];
}

void write_digital_inputs_state_reg(const uint16_t state)
{
	usRegInputBuf[REG_INPUT_JOINT_DIGITAL_INPUTS] = state;
}

void write_digital_outputs_state_reg(const uint16_t state)
{
	usRegInputBuf[REG_INPUT_JOINT_DIGITAL_OUTPUTS] = state;
}

void write_actuator_base_voltage_reg(const float voltage)
{
	usRegInputBuf[REG_INPUT_ACTUATOR_BASE_VOLTAGE] = ( (uint16_t) (voltage*100) );
}

void write_actuator_speed_limit_reg(const float limit)
{
	uint16_t * ui16ptr = ((uint16_t*)(&limit));
	usRegInputBuf[REG_INPUT_ACTUATOR_LIMITS_OUTPUT_SPEED] = ui16ptr[1];
	usRegInputBuf[REG_INPUT_ACTUATOR_LIMITS_OUTPUT_SPEED+1] = ui16ptr[0];
}

void write_actuator_voltage_limit_reg(const float limit)
{
	usRegInputBuf[REG_INPUT_ACTUATOR_LIMITS_OUTPUT_VOLTAGE] = ( (uint16_t) (limit*100) );
}

void write_actuator_current_limit_reg(const float limit)
{
	usRegInputBuf[REG_INPUT_ACTUATOR_LIMITS_OUTPUT_CURRENT] = ( (uint16_t) (limit*100) );
}

// Holding registers update process ---------------------------------- //
void read_control_mode_reg(void)
{
	uint16_t Reg = usRegHoldingBuf[REG_HOLDING_CONTROL_MODE];
	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Mode = Reg;
}

void read_control_reference_reg(void)
{
	float value = 0;
	uint16_t * ui16ptr = ((uint16_t*)(&value));
	ui16ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROL_REFERENCE+1];
	ui16ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROL_REFERENCE];
	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Reference = value;
}

void read_control_rate_reg(void)
{
	uint16_t Reg = SJC_Helpers_IntClamp(usRegHoldingBuf[REG_HOLDING_CONTROL_SETUP_RATE], SJC_CONTROLLER_MIN_UPDATE_RATE_HZ, SJC_CONTROLLER_MAX_UPDATE_RATE_HZ);
	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Rate = Reg;
}

void read_digital_outputs_command_reg(void)
{
	uint16_t Reg = (usRegHoldingBuf[REG_HOLDING_JOINT_DIGITAL_OUTPUTS] & 0b11);
	SJC_WriteDigitalOutputs(Reg);
}

void read_actuator_base_voltage_reg(void)
{
	uint16_t Reg = usRegHoldingBuf[REG_HOLDING_ACTUATOR_BASE_VOLTAGE];
	if (Reg != xSjcJoint[SJC_ACTUATOR_JOINT].Actuator.BaseVoltage)
	{
		SJC_Actuator_SetBaseVoltage(&xSjcJoint[SJC_ACTUATOR_JOINT].Actuator, ( (1.0f*Reg)/100.0f ) );
	}
}

void read_actuator_voltage_limit_reg(void)
{
	uint16_t Reg = usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_OUTPUT_VOLTAGE];
	float Aux = (1.0f*Reg)/100.0f;

	xSjcJoint[SJC_ACTUATOR_JOINT].Limits.OutputVoltage.Max = fmin(SJC_ACTUATOR_LIMIT_MAX_VOLTAGE, Aux);
	xSjcJoint[SJC_ACTUATOR_JOINT].Limits.OutputVoltage.Min = -xSjcJoint[SJC_ACTUATOR_JOINT].Limits.OutputVoltage.Max;
}

void read_actuator_current_limit_reg(void)
{
	uint16_t Reg = usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_OUTPUT_CURRENT];
	float Aux = (1.0f*Reg)/100.0f;

	xSjcJoint[SJC_ACTUATOR_JOINT].Limits.OutputVoltage.Max = fmin(SJC_ACTUATOR_LIMIT_MAX_CURRENT, Aux);
	xSjcJoint[SJC_ACTUATOR_JOINT].Limits.OutputVoltage.Min = -xSjcJoint[SJC_ACTUATOR_JOINT].Limits.OutputVoltage.Max;
}

void read_actuator_speed_limit_reg(void)
{
	float value = 0;
	uint16_t * ui16ptr = ((uint16_t*)(&value));
	ui16ptr[0] = usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_OUTPUT_SPEED+1];
	ui16ptr[1] = usRegHoldingBuf[REG_HOLDING_ACTUATOR_LIMITS_OUTPUT_SPEED];
	xSjcJoint[SJC_ACTUATOR_JOINT].Limits.Speed.Max = fmin(SJC_ACTUATOR_LIMIT_MAX_SPEED, value);
	xSjcJoint[SJC_ACTUATOR_JOINT].Limits.Speed.Min = -xSjcJoint[SJC_ACTUATOR_JOINT].Limits.Speed.Max;
}

void read_control_setup_Kp_reg(void)
{
	float value = 0;
	uint16_t * ui16ptr = ((uint16_t*)(&value));
	ui16ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROL_REFERENCE+1];
	ui16ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROL_REFERENCE];
	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Kp = value;
}

void read_control_setup_Ki_reg(void)
{
	float value = 0;
	uint16_t * ui16ptr = ((uint16_t*)(&value));
	ui16ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROL_REFERENCE+1];
	ui16ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROL_REFERENCE];
	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Ki = value;
}

void read_control_setup_Kd_reg(void)
{
	float value = 0;
	uint16_t * ui16ptr = ((uint16_t*)(&value));
	ui16ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROL_REFERENCE+1];
	ui16ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROL_REFERENCE];
	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Kd = value;
}

void read_control_setup_Aux1_reg(void)
{
	float value = 0;
	uint16_t * ui16ptr = ((uint16_t*)(&value));
	ui16ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROL_REFERENCE+1];
	ui16ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROL_REFERENCE];
	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Aux1 = value;
}

void read_control_setup_Aux2_reg(void)
{
	float value = 0;
	uint16_t * ui16ptr = ((uint16_t*)(&value));
	ui16ptr[0] = usRegHoldingBuf[REG_HOLDING_CONTROL_REFERENCE+1];
	ui16ptr[1] = usRegHoldingBuf[REG_HOLDING_CONTROL_REFERENCE];
	xSjcJoint[SJC_ACTUATOR_JOINT].Controller.Parameters.Aux2 = value;
}

void read_joint_encoder_setup_resolution_reg(void)
{
	uint16_t Reg = usRegInputBuf[REG_HOLDING_ENCODER_SETUP_JOINT_RESOLUTION_PPR];
	xSjcJoint[SJC_ACTUATOR_JOINT].Encoder.Resolution_PPR = Reg;
}

void read_aux_A_encoder_setup_resolution_reg(void)
{
	uint16_t Reg = usRegInputBuf[REG_HOLDING_ENCODER_SETUP_AUX_A_RESOLUTION_PPR];
	xSjcJoint[SJC_AUX_JOINT_A].Encoder.Resolution_PPR = Reg;
}

void read_aux_B_encoder_setup_resolution_reg(void)
{
	uint16_t Reg = usRegInputBuf[REG_HOLDING_ENCODER_SETUP_AUX_B_RESOLUTION_PPR];
	xSjcJoint[SJC_AUX_JOINT_B].Encoder.Resolution_PPR = Reg;
}

// ------------------------------------------------------------------- //
