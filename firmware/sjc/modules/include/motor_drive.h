/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__DC_MOTOR_DRIVE_H_
#define SJC__MODULES__DC_MOTOR_DRIVE_H_

#include "system_types/include/types.h"

void SJC_MotorDrive_Init(eSjcPwmBaseFreq_t BaseFrequency, float BaseVoltage);
void SJC_MotorDrive_Enable(void);
void SJC_MotorDrive_Disable(void);

eSjcReturnType_t SJC_MotorDrive_GetState(bool *state);
eSjcReturnType_t SJC_MotorDrive_GetPowerSupplyVoltage(float *voltage);

eSjcReturnType_t SJC_MotorDrive_GetBaseVoltage(float *voltage);
eSjcReturnType_t SJC_MotorDrive_SetBaseVoltage(float voltage);

eSjcReturnType_t SJC_MotorDrive_GetOutputCurrent(float *current);
eSjcReturnType_t SJC_MotorDrive_GetOutputVoltage(float *voltage);
eSjcReturnType_t SJC_MotorDrive_SetOutputVoltage(const float voltage);

#endif /* SJC__MODULES__DC_MOTOR_DRIVE_H_ */
