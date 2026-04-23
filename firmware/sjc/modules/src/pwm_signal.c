/*
 * pwm_signal.c
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  PWM timer configuration, frequency selection, and duty cycle control.
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "modules/include/pwm_signal.h"
#include "base/include/configs.h"
#include "modules/include/debug_output.h"
#include "system_types/include/helpers.h"

// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
// Lookup table for PWM frequency configuration (frequency value and period multiplier)
typedef struct
{
	eSjcPwmBaseFreq_t freq_enum;
	uint32_t freq_value;
	float multiplier;
} PwmFreqConfig_t;

static const PwmFreqConfig_t pwm_freq_table[] = {
	{PWM_FREQ_10kHz, 10000, 2.0f},
	{PWM_FREQ_12kHz, 12000, 1.665f},
	{PWM_FREQ_14kHz, 14000, 1.428f},
	{PWM_FREQ_15kHz, 15000, 1.332f},
	{PWM_FREQ_16kHz, 16000, 1.249f},
	{PWM_FREQ_18kHz, 18000, 1.110f},
	{PWM_FREQ_20kHz, 20000, 1.0f} // default
};
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
	LL_TIM_DisableAllOutputs(Pwm.TimerConfig.Instance);
	LL_TIM_DisableCounter(Pwm.TimerConfig.Instance);

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
	if (!isfinite(duty))
	{
		return SJC_RET_NOK;
	}

	Pwm.DutyCycle = SJC_Helpers_RealClamp(duty, Pwm.DutyRange.Min, Pwm.DutyRange.Max);
	Pwm.DutyCount = ((uint32_t)(Pwm.DutyCycle * Pwm.TimerConfig.CountPeriod));

	// Using fixed CH1 and CH3 channels for complementary modulation
	LL_TIM_OC_SetCompareCH1(Pwm.TimerConfig.Instance, Pwm.DutyCount);
	LL_TIM_OC_SetCompareCH3(Pwm.TimerConfig.Instance, Pwm.DutyCount);

	return SJC_RET_OK;
}

uint32_t PwmSignal_getFrequency(void)
{
	return Pwm.Frequency;
}

uint32_t PwmSignal_setFrequency(const eSjcPwmBaseFreq_t BaseFrequency)
{
	const size_t table_size = sizeof(pwm_freq_table) / sizeof(pwm_freq_table[0]);

	for (size_t i = 0; i < table_size; ++i)
	{
		if (pwm_freq_table[i].freq_enum == BaseFrequency)
		{
			Pwm.Frequency = pwm_freq_table[i].freq_value;
			Pwm.TimerConfig.CountPeriod = (uint32_t)(SJC_PWM_COUNT_PERIOD * pwm_freq_table[i].multiplier);
			LL_TIM_SetAutoReload(Pwm.TimerConfig.Instance, Pwm.TimerConfig.CountPeriod - 1);

			DebugOutput_systemPrintf("\r\n[PWM] Base frequency set to %lu Hz", Pwm.Frequency);
			return Pwm.Frequency;
		}
	}

	// Fallback to default: 20kHz
	Pwm.Frequency = 20000;
	Pwm.TimerConfig.CountPeriod = (uint32_t)(SJC_PWM_COUNT_PERIOD * 1.0f);
	LL_TIM_SetAutoReload(Pwm.TimerConfig.Instance, Pwm.TimerConfig.CountPeriod - 1);

	DebugOutput_systemPrintf("\r\n[PWM] Invalid frequency enum, defaulting to %lu Hz", Pwm.Frequency);
	return Pwm.Frequency;
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
