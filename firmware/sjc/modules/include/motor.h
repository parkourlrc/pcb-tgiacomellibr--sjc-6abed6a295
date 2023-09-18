/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MOTOR_H_
#define SJC__MOTOR_H_

#include "pwm.h"

void SJC_Motor_Init(const float BaseVoltage, const eSjcPwmFreq_t BaseFrequency);
void SJC_Motor_Enable(void);
void SJC_Motor_Disable(void);
float SJC_Motor_GetPowerSupplyState(void);

float SJC_Motor_GetOutputVoltage(void);
float SJC_Motor_SetOutputVoltage(const float voltage);

float SJC_Motor_GetElectricCurrent(void);
float SJC_Motor_GetPowerSupplyVoltage(void);

#endif /* MOTOR_H_ */
