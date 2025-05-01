/*
 * pwm.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "modules/include/pwm_signal.h"
#include "base/include/configs.h"
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
static xSjcActuatorPwm_t Pwm;

// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void PwmSignal_init(const eSjcPwmBaseFreq_t BaseFrequency)
{
	PwmSignal_setFrequency(BaseFrequency);

	Pwm.DutyRange.Max = SJC_PWM_MAX_DUTY_CYCLE;
	Pwm.DutyRange.Min = SJC_PWM_MIN_DUTY_CYCLE;

	Pwm.TimerConfig.Instance = SJC_PWM_INSTANCE;

	Pwm.TimerConfig.LeftChannel = SJC_PWM_LEFT_CHANNEL;
	Pwm.TimerConfig.RightChannel = SJC_PWM_RIGHT_CHANNEL;
	Pwm.TimerConfig.LeftChannelComp = SJC_PWM_LEFT_COMPLEMENTARY_CHANNEL;
	Pwm.TimerConfig.RightChannelComp = SJC_PWM_RIGHT_COMPLEMENTARY_CHANNEL;
}

void PwmSignal_enable(void)
{
	LL_TIM_CC_EnableChannel(Pwm.TimerConfig.Instance, Pwm.TimerConfig.LeftChannel);
	LL_TIM_CC_EnableChannel(Pwm.TimerConfig.Instance, Pwm.TimerConfig.RightChannel);

	LL_TIM_CC_EnableChannel(Pwm.TimerConfig.Instance, Pwm.TimerConfig.LeftChannelComp);
	LL_TIM_CC_EnableChannel(Pwm.TimerConfig.Instance, Pwm.TimerConfig.RightChannelComp);

	LL_TIM_EnableCounter(Pwm.TimerConfig.Instance);
	LL_TIM_EnableAllOutputs(Pwm.TimerConfig.Instance);
}

void PwmSignal_disable(void)
{
	LL_TIM_DisableAllOutputs(SJC_PWM_INSTANCE);
	LL_TIM_DisableCounter(SJC_PWM_INSTANCE);

	LL_TIM_CC_DisableChannel(Pwm.TimerConfig.Instance, Pwm.TimerConfig.LeftChannel);
	LL_TIM_CC_DisableChannel(Pwm.TimerConfig.Instance, Pwm.TimerConfig.RightChannel);

	LL_TIM_CC_DisableChannel(Pwm.TimerConfig.Instance, Pwm.TimerConfig.LeftChannelComp);
	LL_TIM_CC_DisableChannel(Pwm.TimerConfig.Instance, Pwm.TimerConfig.RightChannelComp);
}

float PwmSignal_getDutyCycle(void)
{
	return Pwm.DutyCycle;
}

float PwmSignal_getMinDutyCycle(void)
{
	return Pwm.DutyRange.Min;
}

float PwmSignal_getMaxDutyCycle(void)
{
	return Pwm.DutyRange.Max;
}

eSjcReturnType_t PwmSignal_setDutyCycle(const float duty)
{
	Pwm.DutyCycle = SJC_Helpers_RealClamp(duty, Pwm.DutyRange.Min, Pwm.DutyRange.Max);
	Pwm.DutyCount = ((uint32_t)(Pwm.DutyCycle * Pwm.TimerConfig.CountPeriod));

	LL_TIM_OC_SetCompareCH1(Pwm.TimerConfig.Instance, Pwm.DutyCount);
	LL_TIM_OC_SetCompareCH3(Pwm.TimerConfig.Instance, Pwm.DutyCount);
	//	LL_TIM_OC_SetCompareCH3(Pwm.TimerConfig.Instance, (Pwm.TimerConfig.CountPeriod - Pwm.DutyCount));

	return SJC_RET_OK;
}

uint32_t PwmSignal_getFrequency(void)
{
	return Pwm.Frequency;
}

uint32_t PwmSignal_setFrequency(const eSjcPwmBaseFreq_t BaseFrequency)
{
	switch (BaseFrequency)
	{
	case PWM_FREQ_10kHz:
		Pwm.Frequency = 10000;
		Pwm.TimerConfig.CountPeriod = ((uint32_t)(SJC_PWM_COUNT_PERIOD * 2.0f));
		break;
	case PWM_FREQ_12kHz:
		Pwm.Frequency = 12000;
		Pwm.TimerConfig.CountPeriod = ((uint32_t)(SJC_PWM_COUNT_PERIOD * 1.665f));
		break;
	case PWM_FREQ_14kHz:
		Pwm.Frequency = 14000;
		Pwm.TimerConfig.CountPeriod = ((uint32_t)(SJC_PWM_COUNT_PERIOD * 1.428f));
		break;
	case PWM_FREQ_15kHz:
		Pwm.Frequency = 15000;
		Pwm.TimerConfig.CountPeriod = ((uint32_t)(SJC_PWM_COUNT_PERIOD * 1.332f));
		break;
	case PWM_FREQ_16kHz:
		Pwm.Frequency = 16000;
		Pwm.TimerConfig.CountPeriod = ((uint32_t)(SJC_PWM_COUNT_PERIOD * 1.249f));
		break;
	case PWM_FREQ_18kHz:
		Pwm.Frequency = 18000;
		Pwm.TimerConfig.CountPeriod = ((uint32_t)(SJC_PWM_COUNT_PERIOD * 1.110f));
		break;
	default: // PWM_FREQ_20kHz:
		Pwm.Frequency = 20000;
		Pwm.TimerConfig.CountPeriod = ((uint32_t)(SJC_PWM_COUNT_PERIOD * 1.0f));
		break;
	}

	LL_TIM_SetAutoReload(Pwm.TimerConfig.Instance, Pwm.TimerConfig.CountPeriod - 1);

	return Pwm.Frequency;
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
