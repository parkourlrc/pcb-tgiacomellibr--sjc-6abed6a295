/*
 * data_structures.h
 *
 *  Created on: Sep 17, 2023
 *      Author: tiago
 */

#ifndef SJC__SYSTEM_TYPES__DATA_STRUCTURES_H_
#define SJC__SYSTEM_TYPES__DATA_STRUCTURES_H_

// System includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "stm32f446xx.h"
// ------------------------------------------------------------------- //

// Definitions and enumerations -------------------------------------- //
// ------------------------------------------------------------------- //
#define ZERO_FLOAT ((float)0.0f)

typedef enum
{
	SJC_RET_NOK = 0x0000,
	SJC_RET_OK = 0x00ff,
	SJC_RET_SIZE
} eSjcReturnType_t;

typedef enum
{
	SJC_DIGITAL_OUTPUT_PORT = 0,
	SJC_DIGITAL_INPUT_PORT = 1,
	SJC_DIGITAL_PORT_TYPE_SIZE
} eSjcDigitalPortType_t;

typedef enum
{
	SJC_TIME_MS = 0,
	SJC_TIME_SEC,
	SJC_TIME_TYPE_SIZE
} eSjcTimeType_t;

typedef enum
{
	SJC_PASSIVE_JOINT = 0x00,
	SJC_ACTIVE_JOINT,
	SJC_JOINT_TYPE_SIZE
} eSjcJointType_t;

typedef enum
{
	SJC_ACTUATOR_JOINT = 0,
	SJC_AUX_JOINT_A,
	SJC_AUX_JOINT_B,
	SJC_JOINTS_SIZE
} eSjcJoints_t;

typedef enum
{
	SJC_CONTROL_DISABLED = 0x0000,
	SJC_OPEN_LOOP_CONTROL,
	SJC_SPEED_CONTROL,
	SJC_TORQUE_CONTROL,
	SJC_PRBS_EXCITATION,
	SJC_TRIANG_EXCITATION,
	SJC_PULSE_FAST_EXCITATION,
	SJC_PULSE_SLOW_EXCITATION
} eSjcControlMode_t;

typedef enum
{
	INPUT_TYPE_NONE = 0,
	INPUT_TYPE_PRBS,
	INPUT_TYPE_TRIANG,
	INPUT_TYPE_PULSE_FAST,
	INPUT_TYPE_PULSE_SLOW,
	INPUT_TYPE_SIZE
} eSjcInputType_t;

typedef enum
{
	INPUT_SIGNAL_STATE_INIT_CONFIG = 0,
	INPUT_SIGNAL_STATE_RUNNING,
	INPUT_SIGNAL_STATE_FINISHED,
	INPUT_SIGNAL_STATE_SIZE
} eSjcInputSignalState_t;

typedef enum
{
	SJC_CURRENT_SENSOR_STD = 0,
	SJC_CURRENT_SENSOR_AUX,
	SJC_VOLTAGE_SENSOR,
	SJC_ANALOG_SENSOR_SIZE
} eSjcAnalogSensorType_t;

typedef enum
{
	SJC_JOINT_SWITCH_A = 0,
	SJC_JOINT_SWITCH_B,
	SJC_ISOLATED_INPUT_A,
	SJC_ISOLATED_INPUT_B,
	SJC_DIGITAL_INPUTS_SIZE
} eSjcDigitalInputs_t;

typedef enum
{
	SJC_DIGITAL_OUTPUT_A = 0,
	SJC_DIGITAL_OUTPUT_B,
	SJC_HEARTBEAT_OUTPUT,
	SJC_DIGITAL_OUTPUTS_SIZE
} eSjcDigitalOutputs_t;

typedef enum
{
	PWM_FREQ_10kHz = 0,
	PWM_FREQ_12kHz,
	PWM_FREQ_14kHz,
	PWM_FREQ_15kHz,
	PWM_FREQ_16kHz,
	PWM_FREQ_18kHz,
	PWM_FREQ_20kHz
} eSjcPwmBaseFreq_t;
// ------------------------------------------------------------------- //

// Data structures --------------------------------------------------- //
// ------------------------------------------------------------------- //
typedef struct
{
	uint32_t Samples;
	uint32_t AccumulatedTimerTicks;
} xSjcMeanSampleTimeMeasurement_t;
static const xSjcMeanSampleTimeMeasurement_t xSjcSampleTimeEmptyObj = {0};

typedef struct
{
	float Min;
	float Max;
} xSjcRange_t;
static const xSjcRange_t xSjcRangeEmptyObj = {0};

typedef struct
{
	float Encoder;
	int32_t Counts;
	TIM_TypeDef *Instance;
	uint16_t Resolution_PPR;
	float radians_per_quadrature_pulse;
} xSjcEncoder_t;

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float Aux1;
	float Aux2;
} xPidControllerParameters_t;
static const xPidControllerParameters_t xControllerParametersEmptyObj = {0};

typedef struct
{
	float Derivative;
	float Integral;
	float ActualError;
	float PrevError;
} xPidControllerErrors_t;
static const xPidControllerErrors_t xControllerErrorsEmptyObj = {0};

typedef struct
{
	float Reference;
	float ControlAction;
	xPidControllerErrors_t Errors;
	xPidControllerParameters_t Parameters;
} xSjcPidController_t;
static const xSjcPidController_t xPidControllerEmptyObj = {0};

typedef struct
{
	float Rate;
	eSjcControlMode_t Mode;
} xSjcControlConfigs_t;
static const xSjcControlConfigs_t xSjcControlConfigsEmptyObj = {0};

typedef struct
{
	TIM_TypeDef *Instance;
	uint32_t CountPeriod;
	uint32_t LeftChannel;
	uint32_t LeftChannelComp;
	uint32_t RightChannel;
	uint32_t RightChannelComp;
} xSjcActuatorPwmTimerConfig_t;

typedef struct
{
	float DutyCycle;
	uint32_t Frequency;
	uint32_t DutyCount;
	xSjcRange_t DutyRange;
	xSjcActuatorPwmTimerConfig_t TimerConfig;
} xSjcActuatorPwm_t;

typedef struct
{
	uint32_t Pin;
	GPIO_TypeDef *Port;
	eSjcDigitalPortType_t Type;
} xSjcDigitalPort_t;

typedef struct
{
	bool State;
	float BaseVoltage;
	float OutputVoltage;
	xSjcRange_t OutputVoltageRange;
	xSjcDigitalPort_t DriveEnablePort;
} xSjcMotorDrive_t;

typedef struct
{
	uint16_t Address;
	uint32_t Baudrate;
	uint8_t Parity;
} xSjcNetworkConfig_t;

// ------------------------------------------------------------------- //

#endif /* SJC__SYSTEM_TYPES__DATA_STRUCTURES_H_ */
