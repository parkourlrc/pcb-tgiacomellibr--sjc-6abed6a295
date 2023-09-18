/*
 * motor.c
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */


// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //
#include "tim.h"
// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "types.h"

#include "pwm.h"
// ------------------------------------------------------------------- //


// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //
#define SJC_PWM_COUNT_PERIOD 						( (uint32_t) 1000 	)
#define SJC_PWM_MIN_DUTY_CYCLE 						( (float) 	 0.02	)
#define SJC_PWM_MAX_DUTY_CYCLE 						( (float) 	 0.98	)
#define SJC_PWM_INSTANCE										 TIM1
// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //
typedef struct {
	float DutyCycle;
	uint32_t Frequency;
	uint32_t DutyCount;
	uint32_t CountPeriod;
	sjc_range_t DutyRange;
} sjc_pwm_config_t;
// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //
static sjc_pwm_config_t xPwmConfig;

//extern TIM_HandleTypeDef hSJC_PWM_INSTANCE;
// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void SJC_PWM_Init(const eSjcPwmFreq_t BaseFrequency)
{
	SJC_PWM_SetFrequency(BaseFrequency);

	xPwmConfig.DutyRange.Max = SJC_PWM_MAX_DUTY_CYCLE;
	xPwmConfig.DutyRange.Min = SJC_PWM_MIN_DUTY_CYCLE;
}

void SJC_PWM_Enable(void)
{
	LL_TIM_CC_EnableChannel(SJC_PWM_INSTANCE, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(SJC_PWM_INSTANCE, LL_TIM_CHANNEL_CH3);

	LL_TIM_CC_EnableChannel(SJC_PWM_INSTANCE, LL_TIM_CHANNEL_CH1N);
	LL_TIM_CC_EnableChannel(SJC_PWM_INSTANCE, LL_TIM_CHANNEL_CH3N);
	LL_TIM_EnableCounter(SJC_PWM_INSTANCE);
	LL_TIM_EnableAllOutputs(SJC_PWM_INSTANCE);
}

void SJC_PWM_Disable(void)
{
	LL_TIM_DisableAllOutputs(SJC_PWM_INSTANCE);
	LL_TIM_DisableCounter(SJC_PWM_INSTANCE);
	LL_TIM_CC_DisableChannel(SJC_PWM_INSTANCE, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_DisableChannel(SJC_PWM_INSTANCE, LL_TIM_CHANNEL_CH1N);
	LL_TIM_CC_DisableChannel(SJC_PWM_INSTANCE, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_DisableChannel(SJC_PWM_INSTANCE, LL_TIM_CHANNEL_CH3N);
}

float SJC_PWM_GetMinDutyCycle(void)
{
	return xPwmConfig.DutyRange.Min;
}

float SJC_PWM_GetMaxDutyCycle(void)
{
	return xPwmConfig.DutyRange.Max;
}

float SJC_PWM_GetDutyCycle(const float duty)
{
	return xPwmConfig.DutyCycle;
}

float SJC_PWM_SetDutyCycle(const float duty)
{
	if (duty > xPwmConfig.DutyRange.Max)
	{
		xPwmConfig.DutyCycle = xPwmConfig.DutyRange.Max;
	}
	else if (duty < xPwmConfig.DutyRange.Min)
	{
		xPwmConfig.DutyCycle = xPwmConfig.DutyRange.Min;
	}
	else
	{
		xPwmConfig.DutyCycle = duty;
	}

	xPwmConfig.DutyCount = ( (uint32_t) (xPwmConfig.DutyCycle*xPwmConfig.CountPeriod) );

	LL_TIM_OC_SetCompareCH1(SJC_PWM_INSTANCE, xPwmConfig.DutyCount);
	LL_TIM_OC_SetCompareCH3(SJC_PWM_INSTANCE, (xPwmConfig.CountPeriod - xPwmConfig.DutyCount));

	return xPwmConfig.DutyCycle;
}

uint32_t SJC_PWM_GetFrequency(const eSjcPwmFreq_t BaseFrequency)
{
	return xPwmConfig.Frequency;
}

uint32_t SJC_PWM_SetFrequency(const eSjcPwmFreq_t BaseFrequency)
{
	switch (BaseFrequency) {
	case PWM_FREQ_10kHz:
		xPwmConfig.Frequency = 10000;
		xPwmConfig.CountPeriod = ( (uint32_t)(SJC_PWM_COUNT_PERIOD*2.0f) );
		break;
	case PWM_FREQ_12kHz:
		xPwmConfig.Frequency = 10000;
		xPwmConfig.CountPeriod = ( (uint32_t)(SJC_PWM_COUNT_PERIOD*1.665f) );
		break;
	case PWM_FREQ_14kHz:
		xPwmConfig.Frequency = 10000;
		xPwmConfig.CountPeriod = ( (uint32_t)(SJC_PWM_COUNT_PERIOD*1.428f) );
		break;
	case PWM_FREQ_15kHz:
		xPwmConfig.Frequency = 15000;
		xPwmConfig.CountPeriod = ( (uint32_t)(SJC_PWM_COUNT_PERIOD*1.332f) );
		break;
	case PWM_FREQ_16kHz:
		xPwmConfig.Frequency = 15000;
		xPwmConfig.CountPeriod = ( (uint32_t)(SJC_PWM_COUNT_PERIOD*1.249f) );
		break;
	case PWM_FREQ_18kHz:
		xPwmConfig.Frequency = 15000;
		xPwmConfig.CountPeriod = ( (uint32_t)(SJC_PWM_COUNT_PERIOD*1.110f) );
		break;
	default: // PWM_FREQ_20kHz:
		xPwmConfig.Frequency = 20000;
		xPwmConfig.CountPeriod = ( (uint32_t)(SJC_PWM_COUNT_PERIOD*1.0f) );
		break;
	}

	LL_TIM_SetAutoReload(SJC_PWM_INSTANCE, xPwmConfig.CountPeriod -1);

	return xPwmConfig.Frequency;
}
// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
