/*
 * motor_drive.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */


// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "main.h"
// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "motor_drive.h"
#include "pwm.h"
#include "configs.h"
#include "analog_interfaces.h"
// ------------------------------------------------------------------- //


// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void SJC_MotorDrive_Init(xSjcMotorDrive_t * MotorDrive, eSjcPwmBaseFreq_t BaseFrequency, float BaseVoltage)
{
	MotorDrive->DriveEnablePort.Pin = SJC_ACTUATOR_DRIVER_ENABLE_PIN;
	MotorDrive->DriveEnablePort.Port = SJC_ACTUATOR_DRIVER_ENABLE_PORT;

	SJC_MotorDrive_Disable(MotorDrive);

	SJC_PWM_Init(&MotorDrive->PWM, BaseFrequency);

	SJC_MotorDrive_SetBaseVoltage(MotorDrive, BaseVoltage);

	SJC_MotorDrive_SetOutputVoltage(MotorDrive, 0.0f);
}

float SJC_MotorDrive_GetBaseVoltage(xSjcMotorDrive_t * MotorDrive)
{
	return MotorDrive->BaseVoltage;
}

eSjcReturnType_t SJC_MotorDrive_SetBaseVoltage(xSjcMotorDrive_t * MotorDrive, float voltage)
{
	MotorDrive->BaseVoltage = voltage;
	MotorDrive->OutputVoltageRange.Max = SJC_PWM_GetMaxDutyCycle(&MotorDrive->PWM)*MotorDrive->BaseVoltage;
	MotorDrive->OutputVoltageRange.Min = -MotorDrive->OutputVoltageRange.Max;
//	MotorDrive->OutputVoltageRange.Min = -(1.0f-SJC_PWM_GetMinDutyCycle(&MotorDrive->PWM))*MotorDrive->BaseVoltage;
	return SJC_RET_OK;
}

void SJC_MotorDrive_Enable(xSjcMotorDrive_t * MotorDrive)
{
	LL_GPIO_ResetOutputPin(MotorDrive->DriveEnablePort.Port, MotorDrive->DriveEnablePort.Pin);

	SJC_PWM_Enable(&MotorDrive->PWM);

	SJC_MotorDrive_SetOutputVoltage(MotorDrive, 0.0f);

	MotorDrive->DriveEnablePort.State = false;

	MotorDrive->State = true;
}

void SJC_MotorDrive_Disable(xSjcMotorDrive_t * MotorDrive)
{
	SJC_MotorDrive_SetOutputVoltage(MotorDrive, 0.0f);

	SJC_PWM_Disable(&MotorDrive->PWM);

	LL_GPIO_SetOutputPin(MotorDrive->DriveEnablePort.Port, MotorDrive->DriveEnablePort.Pin);

	MotorDrive->DriveEnablePort.State = true;

	MotorDrive->State = false;
}

bool SJC_MotorDrive_GetState(xSjcMotorDrive_t * MotorDrive)
{
	return MotorDrive->State;
}

float SJC_MotorDrive_GetPowerSupplyVoltage(xSjcMotorDrive_t * MotorDrive)
{
	float value;
	SJC_AnalogInterfaces_GetAnalogConversion(SJC_VOLTAGE_SENSOR, &value);
	MotorDrive->PowerSupplyVoltage.Measurement = value;

	return MotorDrive->PowerSupplyVoltage.Measurement;
}

float SJC_MotorDrive_GetOutputCurrent(xSjcMotorDrive_t * MotorDrive)
{
	float value;
	SJC_AnalogInterfaces_GetAnalogConversion(SJC_CURRENT_SENSOR_STD, &value);
	MotorDrive->OutputCurrent.Measurement = value;

	return MotorDrive->OutputCurrent.Measurement;
}

float SJC_MotorDrive_GetOutputVoltage(xSjcMotorDrive_t * MotorDrive)
{
	return MotorDrive->OutputVoltage.Measurement;
}

float SJC_MotorDrive_SetOutputVoltage(xSjcMotorDrive_t * MotorDrive, const float voltage)
{
	MotorDrive->OutputVoltage.Measurement = SJC_Helpers_RealClamp(voltage, MotorDrive->OutputVoltageRange.Min, MotorDrive->OutputVoltageRange.Max);

	float xPwmDuty = ( MotorDrive->OutputVoltage.Measurement + MotorDrive->BaseVoltage )/( 2.0 * MotorDrive->BaseVoltage );

	SJC_PWM_SetDutyCycle(&MotorDrive->PWM, xPwmDuty);

	return MotorDrive->OutputVoltage.Measurement;
}

float SJC_MotorDrive_GetVoltageLimit(xSjcActuatorLimits_t * Limits)
{
	return Limits->Voltage.Max;
}

float SJC_MotorDrive_SetVoltageLimit(xSjcActuatorLimits_t * Limits, const float value)
{
	Limits->Voltage.Max = fmin(SJC_ACTUATOR_LIMIT_MAX_VOLTAGE, value);
	Limits->Voltage.Min = -Limits->Voltage.Max;

	return Limits->Voltage.Max;
}

float SJC_MotorDrive_GetCurrentLimit(xSjcActuatorLimits_t * Limits)
{
	return Limits->Current.Max;
}

float SJC_MotorDrive_SetCurrentLimit(xSjcActuatorLimits_t * Limits, const float value)
{
	Limits->Current.Max = fmin(SJC_ACTUATOR_LIMIT_MAX_CURRENT, value);
	Limits->Current.Min = -Limits->Current.Max;

	return Limits->Current.Max;
}

float SJC_MotorDrive_GetSpeedLimit(xSjcActuatorLimits_t * Limits)
{
	return Limits->Speed.Max;
}

float SJC_MotorDrive_SetSpeedLimit(xSjcActuatorLimits_t * Limits, const float value)
{
	Limits->Speed.Max = fmin(SJC_ACTUATOR_LIMIT_MAX_SPEED, value);
	Limits->Speed.Min = -Limits->Speed.Max;


	return Limits->Speed.Max;
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
