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

// Definitions includes ---------------------------------------------- //
// ------------------------------------------------------------------- //
typedef enum {
	SJC_RET_NOK = 0x0000,
	SJC_RET_OK = 0x00ff,
	SJC_RET_SIZE
} eSjcReturnType_t;

typedef enum {
	SJC_DIGITAL_OUTPUT_PORT	= 0,
	SJC_DIGITAL_INPUT_PORT 	= 1,
	SJC_DIGITAL_PORT_TYPE_SIZE
} eSjcDigitalPortType_t;

typedef enum {
	SJC_TIME_MS = 0,
	SJC_TIME_SEC,
	SJC_TIME_TYPE_SIZE
} eSjcTimeType_t;

typedef enum {
	SJC_PASSIVE_JOINT = 0x00,
	SJC_ACTIVE_JOINT,
	SJC_JOINT_TYPE_SIZE
} eSjcJointType_t;

typedef enum {
	SJC_ACTUATOR_JOINT = 0,
	SJC_AUX_JOINT_A,
	SJC_AUX_JOINT_B,
	SJC_JOINTS_SIZE
} eSjcJoints_t;

typedef enum {
	SJC_CONTROL_DISABLED 	= 0x0000,
	SJC_OPEN_LOOP_CONTROL,
	SJC_SPEED_CONTROL,
	SJC_TORQUE_CONTROL,
	SJC_PRBS_EXCITATION
} eSjcControlMode_t;

typedef enum {
	SJC_CURRENT_SENSOR_STD = 0,
	SJC_CURRENT_SENSOR_AUX,
	SJC_VOLTAGE_SENSOR,
	SJC_ANALOG_SENSOR_SIZE
} eSjcAnalogSensorType_t;

typedef enum {
	SJC_JOINT_SWITCH_A = 0,
	SJC_JOINT_SWITCH_B,
	SJC_ISOLATED_INPUT_A,
	SJC_ISOLATED_INPUT_B,
	SJC_INPUTS_SIZE
} eSjcDigitalInputs_t;

typedef enum {
	SJC_DIGITAL_OUTPUT_A = 0,
	SJC_DIGITAL_OUTPUT_B,
	SJC_HEARTBEAT_OUTPUT,
	SJC_OUTPUTS_SIZE
} eSjcDigitalOutputs_t;

typedef enum {
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
typedef struct {
	uint32_t Timestamp;
	float JointDisplacement;
	float AuxJointDisplacementA;
	float AuxJointDisplacementB;
	float SystemVoltage;
	float MotorCurrent;
	uint8_t DigitalInputs;
	uint8_t IsolatedInputs;
	uint8_t DigitalOutputs;
} xSjcLogData_t;

typedef struct {
	uint32_t AdcConversion;
	uint32_t TimeStamp;
} xSjcAdcConversion_t;

static const xSjcAdcConversion_t xSjcAdcConversionEmptyObj = {0};

typedef struct {
	uint32_t Samples;
	uint32_t AccumulatedTimerTicks;
} xSjcMeanSampleTimeMeasurement_t;

static const xSjcMeanSampleTimeMeasurement_t xSjcSampleTimeEmptyObj = {0};


typedef struct {
	float Min;
	float Max;
} xSjcRange_t;

typedef struct {
	float Measurement;
	float Resolution;
	xSjcRange_t Range;
	float CompFilterRatio;
} xSjcAnalogSensor_t;

typedef struct {
	float Encoder;
	int32_t Counts;
	TIM_TypeDef *Instance;
	uint16_t Resolution_PPR;
} xSjcEncoder_t;

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float Aux1;
	float Aux2;
} xControllerParameters_t;

static const xControllerParameters_t xControllerParametersEmptyObj = {0};

typedef struct {
	float Derivative;
	float Integral;
	float ActualError;
	float PrevError;
} xControllerErrors_t;

static const xControllerErrors_t xControllerErrorsEmptyObj = {0};

typedef struct {
	float Rate;
	float Reference;
	float ControlAction;
	eSjcControlMode_t Mode;
	xControllerErrors_t Errors;
	xControllerParameters_t Parameters;
} xSjcController_t;

typedef struct {
	xSjcRange_t Speed;
	xSjcRange_t Current;
	xSjcRange_t Voltage;
} xSjcActuatorLimits_t;

typedef struct {
	TIM_TypeDef *Instance;
	uint32_t CountPeriod;
	uint32_t LeftChannel;
	uint32_t LeftChannelComp;
	uint32_t RightChannel;
	uint32_t RightChannelComp;
} xSjcActuatorPwmTimerConfig_t;

typedef struct {
	float DutyCycle;
	uint32_t Frequency;
	uint32_t DutyCount;
	xSjcRange_t DutyRange;
	xSjcActuatorPwmTimerConfig_t TimerConfig;
} xSjcActuatorPwm_t;

typedef struct {
	bool State;
	uint32_t Pin;
	GPIO_TypeDef *Port;
	eSjcDigitalPortType_t Type;
} xSjcDigitalPort_t;

typedef struct {
	bool State;
	float BaseVoltage;
	xSjcActuatorPwm_t PWM;
	xSjcRange_t OutputVoltageRange;
	xSjcAnalogSensor_t OutputVoltage;
	xSjcAnalogSensor_t OutputCurrent;
	xSjcDigitalPort_t DriveEnablePort;
	xSjcAnalogSensor_t PowerSupplyVoltage;
} xSjcMotorDrive_t;

typedef struct {
	float Speed;
	xSjcEncoder_t Encoder;
	xSjcAnalogSensor_t Displacement;
	xSjcMotorDrive_t MotorDrive;
	xSjcController_t Controller;
	xSjcActuatorLimits_t Limits;
	xSjcDigitalPort_t DigitalSensors[SJC_INPUTS_SIZE];
	xSjcDigitalPort_t DigitalActuators[SJC_OUTPUTS_SIZE];
} xSjcJoint_t;

typedef struct
{
    uint16_t Address;
    uint32_t Baudrate;
    char Parity;
} xSjcNetworkConfig_t;
// ------------------------------------------------------------------- //



#endif /* SJC__SYSTEM_TYPES__DATA_STRUCTURES_H_ */
