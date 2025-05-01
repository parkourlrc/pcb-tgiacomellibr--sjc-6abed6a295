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
#include "base/include/configs.h"
#include "modules/include/analog_interfaces.h"
#include "modules/include/motor_drive.h"
#include "modules/include/pwm_signal.h"
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

static float calculate_duty_from_voltage(const float voltage, const float base_voltage);

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void MotorDrive_init(eSjcPwmBaseFreq_t BaseFrequency, float BaseVoltage)
{
#if SJC_HW_VERSION_MAJOR > 1
	MotorDrive.DriveEnablePort.Pin = SJC_ACTUATOR_DRIVER_ENABLE_PIN;
	MotorDrive.DriveEnablePort.Port = SJC_ACTUATOR_DRIVER_ENABLE_PORT;
#else
	MotorDrive.DriveEnablePort.Pin = SJC_ACTUATOR_POWER_OUTPUT_ENABLE_PIN;
	MotorDrive.DriveEnablePort.Port = SJC_ACTUATOR_POWER_OUTPUT_ENABLE_PORT;
#endif

	MotorDrive_disable();

	PwmSignal_init(BaseFrequency);

	MotorDrive_setBaseVoltage(BaseVoltage);

	MotorDrive_setOutputVoltage(ZERO_FLOAT);
}

eSjcReturnType_t MotorDrive_getBaseVoltage(float *voltage)
{
	if (voltage == NULL)
	{
		return SJC_RET_NOK;
	}

	*voltage = MotorDrive.BaseVoltage;

	return SJC_RET_OK;
}

eSjcReturnType_t MotorDrive_setBaseVoltage(float voltage)
{
	MotorDrive.BaseVoltage = voltage;
	MotorDrive.OutputVoltageRange.Max = PwmSignal_getMaxDutyCycle() * MotorDrive.BaseVoltage;
	MotorDrive.OutputVoltageRange.Min = -MotorDrive.OutputVoltageRange.Max;

	return SJC_RET_OK;
}

void MotorDrive_enable(void)
{
	LL_GPIO_ResetOutputPin(MotorDrive.DriveEnablePort.Port, MotorDrive.DriveEnablePort.Pin);

	PwmSignal_enable();

	MotorDrive_setOutputVoltage(ZERO_FLOAT);

	MotorDrive.State = true;
}

void MotorDrive_disable(void)
{
	MotorDrive_setOutputVoltage(ZERO_FLOAT);

	PwmSignal_disable();

	LL_GPIO_SetOutputPin(MotorDrive.DriveEnablePort.Port, MotorDrive.DriveEnablePort.Pin);

	MotorDrive.State = false;
}

eSjcReturnType_t MotorDrive_getState(bool *state)
{
	if (state == NULL)
	{
		return SJC_RET_NOK;
	}

	*state = MotorDrive.State;

	return SJC_RET_OK;
}

eSjcReturnType_t MotorDrive_getPowerSupplyVoltage(float *voltage)
{
	return AnalogInterfaces_getAnalogConversion(SJC_VOLTAGE_SENSOR, voltage);
}

eSjcReturnType_t MotorDrive_getOutputCurrent(float *current)
{
	if (current == NULL)
	{
		return SJC_RET_NOK;
	}

	return AnalogInterfaces_getAnalogConversion(SJC_CURRENT_SENSOR_AUX, current);
}

eSjcReturnType_t MotorDrive_getOutputVoltage(float *voltage)
{
	if (voltage == NULL)
	{
		return SJC_RET_NOK;
	}

	*voltage = MotorDrive.OutputVoltage;

	return SJC_RET_OK;
}

eSjcReturnType_t MotorDrive_setOutputVoltage(const float voltage)
{
	MotorDrive.OutputVoltage = SJC_Helpers_RealClamp(voltage, MotorDrive.OutputVoltageRange.Min, MotorDrive.OutputVoltageRange.Max);

	float xPwmDuty = calculate_duty_from_voltage(MotorDrive.OutputVoltage, MotorDrive.BaseVoltage);

	PwmSignal_setDutyCycle(xPwmDuty);

	return SJC_RET_OK;
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //
/**
 * @brief Calculates the PWM duty cycle required to produce a given output voltage.
 *
 * This assumes a symmetrical output voltage range centered around zero,
 * and maps it linearly to the PWM duty cycle between 0.0 and 1.0.
 *
 * @param voltage Desired output voltage.
 * @param base_voltage Base voltage (symmetric max magnitude).
 * @return float Duty cycle (0.0 to 1.0).
 */
static float calculate_duty_from_voltage(const float voltage, const float base_voltage)
{
	return (voltage + base_voltage) / (2.0f * base_voltage);
}
// ------------------------------------------------------------------- //
