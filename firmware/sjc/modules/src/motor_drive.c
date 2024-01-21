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
static xSjcMotorDrive_t MotorDrive;

// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void SJC_MotorDrive_Init(eSjcPwmBaseFreq_t BaseFrequency, float BaseVoltage)
{
	MotorDrive.DriveEnablePort.Pin = SJC_ACTUATOR_DRIVER_ENABLE_PIN;
	MotorDrive.DriveEnablePort.Port = SJC_ACTUATOR_DRIVER_ENABLE_PORT;

	SJC_MotorDrive_Disable();

	SJC_PWM_Init(BaseFrequency);

	SJC_MotorDrive_SetBaseVoltage(BaseVoltage);

	SJC_MotorDrive_SetOutputVoltage(ZERO_FLOAT);
}

eSjcReturnType_t SJC_MotorDrive_GetBaseVoltage(float * voltage)
{
	*voltage = MotorDrive.BaseVoltage;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_MotorDrive_SetBaseVoltage(float voltage)
{
	MotorDrive.BaseVoltage = voltage;
	MotorDrive.OutputVoltageRange.Max = SJC_PWM_GetMaxDutyCycle()*MotorDrive.BaseVoltage;
	MotorDrive.OutputVoltageRange.Min = -MotorDrive.OutputVoltageRange.Max;

	return SJC_RET_OK;
}

void SJC_MotorDrive_Enable(void)
{
	LL_GPIO_ResetOutputPin(MotorDrive.DriveEnablePort.Port, MotorDrive.DriveEnablePort.Pin);

	SJC_PWM_Enable();

	SJC_MotorDrive_SetOutputVoltage(ZERO_FLOAT);

	MotorDrive.State = true;
}

void SJC_MotorDrive_Disable(void)
{
	SJC_MotorDrive_SetOutputVoltage(ZERO_FLOAT);

	SJC_PWM_Disable();

	LL_GPIO_SetOutputPin(MotorDrive.DriveEnablePort.Port, MotorDrive.DriveEnablePort.Pin);

	MotorDrive.State = false;
}

eSjcReturnType_t SJC_MotorDrive_GetState(bool * state)
{
	*state = MotorDrive.State;

	return SJC_RET_OK;
}

eSjcReturnType_t SJC_MotorDrive_GetPowerSupplyVoltage(float * voltage)
{
	return SJC_AnalogInterfaces_GetAnalogConversion(SJC_VOLTAGE_SENSOR, voltage);
}

eSjcReturnType_t SJC_MotorDrive_GetOutputCurrent(float * current)
{
	return SJC_AnalogInterfaces_GetAnalogConversion(SJC_CURRENT_SENSOR_STD, current);
}

eSjcReturnType_t SJC_MotorDrive_GetOutputVoltage(float * voltage)
{
	*voltage = MotorDrive.OutputVoltage;

	return SJC_RET_OK;
}

eSjcReturnType_t  SJC_MotorDrive_SetOutputVoltage(const float voltage)
{
	MotorDrive.OutputVoltage = SJC_Helpers_RealClamp(voltage, MotorDrive.OutputVoltageRange.Min, MotorDrive.OutputVoltageRange.Max);

	float xPwmDuty = ( MotorDrive.OutputVoltage + MotorDrive.BaseVoltage )/( 2.0 * MotorDrive.BaseVoltage );

	SJC_PWM_SetDutyCycle(xPwmDuty);

	return SJC_RET_OK;
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
