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
#include "types.h"

#include "motor.h"

#include "pwm.h"
#include "analog_sensors.h"
// ------------------------------------------------------------------- //


// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //
#define AD8418_GAIN									( (float) 20.0f		)
#define AD8418_VREF									( (float) 2.5f		)
#define AD8418_SHUNT_RESISTANCE						( (float) 0.008f	)
#define AD8418_ANALOG_FRONTED_GAIN					( (float) 0.79f		)
#define SJC_SIGN_FCN(X)								( (int)  (( (X*(-1)) < 0 ) ? 1 : -1) )

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

typedef struct {
	bool Active;
	float BaseVoltage;
	float OutputVoltage;
	sjc_range_t OutputVoltageRange;
} sjc_motor_config_t;
// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static sjc_motor_config_t xMotorConfig;

// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void SJC_Motor_Init(const float BaseVoltage, const eSjcPwmFreq_t BaseFrequency)
{
	xMotorConfig.Active = false;
	xMotorConfig.BaseVoltage = BaseVoltage;

	SJC_PWM_Init(BaseFrequency);

	xMotorConfig.OutputVoltageRange.Max = SJC_PWM_GetMaxDutyCycle()*xMotorConfig.BaseVoltage;
	xMotorConfig.OutputVoltageRange.Min = (1.0f-SJC_PWM_GetMinDutyCycle())*xMotorConfig.BaseVoltage*(-1.0f);

	SJC_Motor_SetOutputVoltage(0);
}

void SJC_Motor_Enable(void)
{
	LL_GPIO_ResetOutputPin(V_MOTOR_nEN_GPIO_Port, V_MOTOR_nEN_Pin);

	SJC_PWM_Enable();

	SJC_Motor_SetOutputVoltage(0.0f);

	xMotorConfig.Active = true;
}

void SJC_Motor_Disable(void)
{
	SJC_Motor_SetOutputVoltage(0.0f);

	SJC_PWM_Disable();

	LL_GPIO_SetOutputPin(V_MOTOR_nEN_GPIO_Port, V_MOTOR_nEN_Pin);

	xMotorConfig.Active = false;
}

float SJC_Motor_GetPowerSupplyState(void)
{
	return xMotorConfig.Active;
}

float SJC_Motor_GetOutputVoltage(void)
{
	return xMotorConfig.OutputVoltage;
}

float SJC_Motor_SetOutputVoltage(const float voltage)
{
	if (voltage < xMotorConfig.OutputVoltageRange.Min)
	{
		xMotorConfig.OutputVoltage = xMotorConfig.OutputVoltageRange.Min;
	}
	else if (voltage > xMotorConfig.OutputVoltageRange.Max)
	{
		xMotorConfig.OutputVoltage = xMotorConfig.OutputVoltageRange.Max;
	}
	else
	{
		xMotorConfig.OutputVoltage = voltage;
	}

	float xPwmDuty = ( voltage + xMotorConfig.BaseVoltage )/( 2.0 * xMotorConfig.BaseVoltage );

	SJC_PWM_SetDutyCycle(xPwmDuty);

	return xMotorConfig.OutputVoltage;
}

float SJC_Motor_GetElectricCurrent(void)
{
	//	I_adc_meas = ((V_adc_avg./Av_input_3v3)-Vref)./(Rshunt_std*Av_AD8418)

	float current = 0;
	current = (SJC_AnalogSensors_GetConversion(SJC_CURRENT_SENSOR_1)/AD8418_ANALOG_FRONTED_GAIN)-AD8418_VREF;
	current /= (AD8418_SHUNT_RESISTANCE*AD8418_GAIN);

	return (current);
}

float SJC_Motor_GetPowerSupplyVoltage(void)
{
	return (SJC_AnalogSensors_GetConversion(SJC_VOLTAGE_SENSOR)*11);
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
