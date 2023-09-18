/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__PWM_H_
#define SJC__PWM_H_

typedef enum {
	PWM_FREQ_10kHz = 0,
	PWM_FREQ_12kHz,
	PWM_FREQ_14kHz,
	PWM_FREQ_15kHz,
	PWM_FREQ_16kHz,
	PWM_FREQ_18kHz,
	PWM_FREQ_20kHz
} eSjcPwmFreq_t;

void SJC_PWM_Init(const eSjcPwmFreq_t BaseFrequency);
void SJC_PWM_Enable(void);
void SJC_PWM_Disable(void);

float SJC_PWM_GetMinDutyCycle(void);
float SJC_PWM_GetMaxDutyCycle(void);
float SJC_PWM_GetDutyCycle(const float duty);
float SJC_PWM_SetDutyCycle(const float duty);

uint32_t SJC_PWM_GetFrequency(const eSjcPwmFreq_t BaseFrequency);
uint32_t SJC_PWM_SetFrequency(const eSjcPwmFreq_t BaseFrequency);


#endif /* MOTOR_H_ */
