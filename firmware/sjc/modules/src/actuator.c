/*
 * motor.c
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
#include "actuator.h"

#include "pwm.h"
#include "configs.h"
#include "analog_sensors.h"
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
void SJC_Actuator_Init(xSjcActuator_t * Actuator, eSjcPwmBaseFreq_t BaseFrequency, float BaseVoltage)
{
	Actuator->BaseFrequency = BaseFrequency;

	Actuator->DriverPort.Pin = SJC_ACTUATOR_DRIVER_ENABLE_PIN;
	Actuator->DriverPort.Port = SJC_ACTUATOR_DRIVER_ENABLE_PORT;

	SJC_Actuator_Disable(Actuator);

	SJC_PWM_Init(&Actuator->PWM, BaseFrequency);

	SJC_Actuator_SetBaseVoltage(Actuator, BaseVoltage);

	SJC_Actuator_SetOutputVoltage(Actuator, 0.0f);
}

float SJC_Actuator_GetBaseVoltage(xSjcActuator_t * Actuator)
{
	return Actuator->BaseVoltage;
}

eSjcReturnType_t SJC_Actuator_SetBaseVoltage(xSjcActuator_t * Actuator, float voltage)
{
	Actuator->BaseVoltage = voltage;
	Actuator->OutputVoltageRange.Max = SJC_PWM_GetMaxDutyCycle(&Actuator->PWM)*Actuator->BaseVoltage;
	Actuator->OutputVoltageRange.Min = -Actuator->OutputVoltageRange.Max;
//	Actuator->OutputVoltageRange.Min = -(1.0f-SJC_PWM_GetMinDutyCycle(&Actuator->PWM))*Actuator->BaseVoltage;
	return SJC_RET_OK;
}

void SJC_Actuator_Enable(xSjcActuator_t * Actuator)
{
	LL_GPIO_ResetOutputPin(Actuator->DriverPort.Port, Actuator->DriverPort.Pin);

	SJC_PWM_Enable(&Actuator->PWM);

	SJC_Actuator_SetOutputVoltage(Actuator, 0.0f);

	Actuator->DriverPort.State = false;

	Actuator->Active = true;
}

void SJC_Actuator_Disable(xSjcActuator_t * Actuator)
{
	SJC_Actuator_SetOutputVoltage(Actuator, 0.0f);

	SJC_PWM_Disable(&Actuator->PWM);

	LL_GPIO_SetOutputPin(Actuator->DriverPort.Port, Actuator->DriverPort.Pin);

	Actuator->DriverPort.State = true;

	Actuator->Active = false;
}

float SJC_Actuator_GetPowerSupplyState(void)
{
	return 0.0f;
}

float SJC_Actuator_GetOutputVoltage(xSjcActuator_t * Actuator)
{
	return Actuator->OutputVoltage;
}

float SJC_Actuator_SetOutputVoltage(xSjcActuator_t * Actuator, const float voltage)
{
	Actuator->OutputVoltage = SJC_Helpers_RealClamp(voltage, Actuator->OutputVoltageRange.Min, Actuator->OutputVoltageRange.Max);

	float xPwmDuty = ( Actuator->OutputVoltage + Actuator->BaseVoltage )/( 2.0 * Actuator->BaseVoltage );

	SJC_PWM_SetDutyCycle(&Actuator->PWM, xPwmDuty);

	return Actuator->OutputVoltage;
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
