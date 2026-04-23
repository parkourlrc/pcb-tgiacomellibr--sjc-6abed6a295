/*
 * motor_drive.c
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  H-bridge motor drive with PWM duty cycle and direction control.
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
#include "system_types/include/helpers.h"
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
// Note: xCurrentBias is written once during calibration and read in the control loop.
// On Cortex-M4, aligned float writes (VSTR) are atomic, so no mutex is needed.
static float xCurrentBias = 0.0f;

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
	SJC_RETURN_IF_NULL_LOG_RET(voltage, "MotorDrive_getBaseVoltage: voltage");

	*voltage = MotorDrive.BaseVoltage;

	return SJC_RET_OK;
}

eSjcReturnType_t MotorDrive_setBaseVoltage(float voltage)
{
	if (voltage <= 0.0f)
	{
		return SJC_RET_NOK;
	}

	MotorDrive.BaseVoltage = voltage;
	MotorDrive.OutputVoltageRange.Max = PwmSignal_getMaxDutyCycle() * MotorDrive.BaseVoltage;
	MotorDrive.OutputVoltageRange.Min = -MotorDrive.OutputVoltageRange.Max;

	return SJC_RET_OK;
}

void MotorDrive_enable(void)
{
	MotorDrive_setOutputVoltage(ZERO_FLOAT);
	PwmSignal_enable();
	LL_GPIO_ResetOutputPin(MotorDrive.DriveEnablePort.Port, MotorDrive.DriveEnablePort.Pin);
	MotorDrive.State = true;
}

void MotorDrive_disable(void)
{
	MotorDrive.OutputVoltage = ZERO_FLOAT;
	LL_GPIO_SetOutputPin(MotorDrive.DriveEnablePort.Port, MotorDrive.DriveEnablePort.Pin);
	PwmSignal_disable();
	MotorDrive.State = false;
}

eSjcReturnType_t MotorDrive_getState(bool *state)
{
	SJC_RETURN_IF_NULL_LOG_RET(state, "MotorDrive_getState: state");

	*state = MotorDrive.State;

	return SJC_RET_OK;
}

eSjcReturnType_t MotorDrive_getPowerSupplyVoltage(float *voltage)
{
	SJC_RETURN_IF_NULL_LOG_RET(voltage, "MotorDrive_getPowerSupplyVoltage: voltage");

	return AnalogInterfaces_getAnalogConversion(SJC_VOLTAGE_SENSOR, voltage);
}

eSjcReturnType_t MotorDrive_getOutputCurrent(float *current)
{
	SJC_RETURN_IF_NULL_LOG_RET(current, "MotorDrive_getOutputCurrent: current");

#if SJC_HW_VERSION_MAJOR > 1
	return AnalogInterfaces_getAnalogConversion(SJC_CURRENT_SENSOR_INA240_25A, current);
#else
	return AnalogInterfaces_getAnalogConversion(SJC_CURRENT_SENSOR_AUX, current);
#endif
}

eSjcReturnType_t MotorDrive_getOutputCurrentUnbiased(float *current)
{
	eSjcReturnType_t ret = MotorDrive_getOutputCurrent(current);
	if (ret == SJC_RET_OK)
	{
		*current -= xCurrentBias;
	}
	return ret;
}

void MotorDrive_setCurrentBias(const float bias)
{
	xCurrentBias = bias;
}

eSjcReturnType_t MotorDrive_getOutputVoltage(float *voltage)
{
	SJC_RETURN_IF_NULL_LOG_RET(voltage, "MotorDrive_getOutputVoltage: voltage");

	*voltage = MotorDrive.OutputVoltage;

	return SJC_RET_OK;
}

void MotorDrive_setOutputVoltage(const float voltage)
{
	MotorDrive.OutputVoltage = SJC_Helpers_RealClamp(voltage, MotorDrive.OutputVoltageRange.Min, MotorDrive.OutputVoltageRange.Max);

	float xPwmDuty = calculate_duty_from_voltage(MotorDrive.OutputVoltage, MotorDrive.BaseVoltage);

	PwmSignal_setDutyCycle(xPwmDuty);
}

eSjcReturnType_t MotorDrive_getMeasurements(xMotorDriveMeasurements_t *measurements)
{
	SJC_RETURN_IF_NULL_LOG_RET(measurements, "MotorDrive_getMeasurements: measurements");

	// Read current and supply voltage in a single mutex acquire
	eSjcAnalogSensorType_t sensors[2];
	float values[2];

#if SJC_HW_VERSION_MAJOR > 1
	sensors[0] = SJC_CURRENT_SENSOR_INA240_25A;
#else
	sensors[0] = SJC_CURRENT_SENSOR_AUX;
#endif
	sensors[1] = SJC_VOLTAGE_SENSOR;

	eSjcReturnType_t ret = AnalogInterfaces_getMultipleConversions(sensors, values, 2);
	if (ret != SJC_RET_OK) return ret;

	measurements->current = values[0] - xCurrentBias;
	measurements->supply_voltage = values[1];
	measurements->voltage = MotorDrive.OutputVoltage;
	measurements->base_voltage = MotorDrive.BaseVoltage;
	measurements->state = MotorDrive.State;

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
	if (base_voltage <= 0.0f)
	{
		return 0.5f;
	}
	return (voltage + base_voltage) / (2.0f * base_voltage);
}
// ------------------------------------------------------------------- //
