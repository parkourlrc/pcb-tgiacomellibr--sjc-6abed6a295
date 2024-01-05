/*
 * motor.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */


// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "pwm.h"

#include "configs.h"
#include "helpers.h"
// ------------------------------------------------------------------- //


// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //


//extern TIM_HandleTypeDef hSJC_PWM_INSTANCE;
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void SJC_PWM_Init(xSjcActuatorPwm_t * Pwm, const eSjcPwmBaseFreq_t BaseFrequency)
{
	SJC_PWM_SetFrequency(Pwm, BaseFrequency);

	Pwm->DutyRange.Max = SJC_PWM_MAX_DUTY_CYCLE;
	Pwm->DutyRange.Min = SJC_PWM_MIN_DUTY_CYCLE;

	Pwm->TimerConfig.Instance = SJC_PWM_INSTANCE;

	Pwm->TimerConfig.LeftChannel = SJC_PWM_LEFT_CHANNEL;
	Pwm->TimerConfig.RightChannel = SJC_PWM_RIGHT_CHANNEL;
	Pwm->TimerConfig.LeftChannelComp = SJC_PWM_LEFT_COMPLEMENTARY_CHANNEL;
	Pwm->TimerConfig.RightChannelComp = SJC_PWM_RIGHT_COMPLEMENTARY_CHANNEL;
}

void SJC_PWM_Enable(xSjcActuatorPwm_t * Pwm)
{
	LL_TIM_CC_EnableChannel(Pwm->TimerConfig.Instance, Pwm->TimerConfig.LeftChannel);
	LL_TIM_CC_EnableChannel(Pwm->TimerConfig.Instance, Pwm->TimerConfig.RightChannel);

	LL_TIM_CC_EnableChannel(Pwm->TimerConfig.Instance, Pwm->TimerConfig.LeftChannelComp);
	LL_TIM_CC_EnableChannel(Pwm->TimerConfig.Instance, Pwm->TimerConfig.RightChannelComp);

	LL_TIM_EnableCounter(Pwm->TimerConfig.Instance);
	LL_TIM_EnableAllOutputs(Pwm->TimerConfig.Instance);
}

void SJC_PWM_Disable(xSjcActuatorPwm_t * Pwm)
{
	LL_TIM_DisableAllOutputs(SJC_PWM_INSTANCE);
	LL_TIM_DisableCounter(SJC_PWM_INSTANCE);

	LL_TIM_CC_DisableChannel(Pwm->TimerConfig.Instance, Pwm->TimerConfig.LeftChannel);
	LL_TIM_CC_DisableChannel(Pwm->TimerConfig.Instance, Pwm->TimerConfig.RightChannel);

	LL_TIM_CC_DisableChannel(Pwm->TimerConfig.Instance, Pwm->TimerConfig.LeftChannelComp);
	LL_TIM_CC_DisableChannel(Pwm->TimerConfig.Instance, Pwm->TimerConfig.RightChannelComp);
}

float SJC_PWM_GetMinDutyCycle(xSjcActuatorPwm_t * Pwm)
{
	return Pwm->DutyRange.Min;
}

float SJC_PWM_GetMaxDutyCycle(xSjcActuatorPwm_t * Pwm)
{
	return Pwm->DutyRange.Max;
}

float SJC_PWM_GetDutyCycle(xSjcActuatorPwm_t * Pwm)
{
	return Pwm->DutyCycle;
}

eSjcReturnType_t SJC_PWM_SetDutyCycle(xSjcActuatorPwm_t * Pwm, const float duty)
{
	Pwm->DutyCycle = SJC_Helpers_RealClamp(duty, Pwm->DutyRange.Min, Pwm->DutyRange.Max);
	Pwm->DutyCount = ( (uint32_t) (Pwm->DutyCycle*Pwm->TimerConfig.CountPeriod) );

	LL_TIM_OC_SetCompareCH1(Pwm->TimerConfig.Instance, Pwm->DutyCount);
	LL_TIM_OC_SetCompareCH3(Pwm->TimerConfig.Instance, (Pwm->TimerConfig.CountPeriod - Pwm->DutyCount));

	return SJC_RET_OK;
}

uint32_t SJC_PWM_GetFrequency(xSjcActuatorPwm_t * Pwm)
{
	return Pwm->Frequency;
}

uint32_t SJC_PWM_SetFrequency(xSjcActuatorPwm_t * Pwm, const eSjcPwmBaseFreq_t BaseFrequency)
{
	switch (BaseFrequency) {
	case PWM_FREQ_10kHz:
		Pwm->Frequency = 10000;
		Pwm->TimerConfig.CountPeriod = ( (uint32_t)(SJC_PWM_COUNT_PERIOD*2.0f) );
		break;
	case PWM_FREQ_12kHz:
		Pwm->Frequency = 12000;
		Pwm->TimerConfig.CountPeriod = ( (uint32_t)(SJC_PWM_COUNT_PERIOD*1.665f) );
		break;
	case PWM_FREQ_14kHz:
		Pwm->Frequency = 14000;
		Pwm->TimerConfig.CountPeriod = ( (uint32_t)(SJC_PWM_COUNT_PERIOD*1.428f) );
		break;
	case PWM_FREQ_15kHz:
		Pwm->Frequency = 15000;
		Pwm->TimerConfig.CountPeriod = ( (uint32_t)(SJC_PWM_COUNT_PERIOD*1.332f) );
		break;
	case PWM_FREQ_16kHz:
		Pwm->Frequency = 16000;
		Pwm->TimerConfig.CountPeriod = ( (uint32_t)(SJC_PWM_COUNT_PERIOD*1.249f) );
		break;
	case PWM_FREQ_18kHz:
		Pwm->Frequency = 18000;
		Pwm->TimerConfig.CountPeriod = ( (uint32_t)(SJC_PWM_COUNT_PERIOD*1.110f) );
		break;
	default: // PWM_FREQ_20kHz:
		Pwm->Frequency = 20000;
		Pwm->TimerConfig.CountPeriod = ( (uint32_t)(SJC_PWM_COUNT_PERIOD*1.0f) );
		break;
	}

	LL_TIM_SetAutoReload(Pwm->TimerConfig.Instance, Pwm->TimerConfig.CountPeriod -1);

	return Pwm->Frequency;
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
