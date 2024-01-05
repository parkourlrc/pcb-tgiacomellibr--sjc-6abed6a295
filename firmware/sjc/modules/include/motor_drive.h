/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__DC_MOTOR_DRIVE_H_
#define SJC__MODULES__DC_MOTOR_DRIVE_H_

#include "types.h"

void SJC_MotorDrive_Init(xSjcMotorDrive_t * MotorDrive, eSjcPwmBaseFreq_t BaseFrequency, float BaseVoltage);
void SJC_MotorDrive_Enable(xSjcMotorDrive_t * MotorDrive);
void SJC_MotorDrive_Disable(xSjcMotorDrive_t * MotorDrive);

bool SJC_MotorDrive_GetState(xSjcMotorDrive_t * MotorDrive);
float SJC_MotorDrive_GetPowerSupplyVoltage(xSjcMotorDrive_t * MotorDrive);

float SJC_MotorDrive_GetBaseVoltage(xSjcMotorDrive_t * MotorDrive);
eSjcReturnType_t SJC_MotorDrive_SetBaseVoltage(xSjcMotorDrive_t * MotorDrive, float voltage);

float SJC_MotorDrive_GetOutputCurrent(xSjcMotorDrive_t * MotorDrive);
float SJC_MotorDrive_GetOutputVoltage(xSjcMotorDrive_t * MotorDrive);
float SJC_MotorDrive_SetOutputVoltage(xSjcMotorDrive_t * MotorDrive, const float voltage);


float SJC_MotorDrive_GetSpeedLimit(xSjcActuatorLimits_t * Limits);
float SJC_MotorDrive_SetSpeedLimit(xSjcActuatorLimits_t * Limits, const float value);
float SJC_MotorDrive_GetVoltageLimit(xSjcActuatorLimits_t * Limits);
float SJC_MotorDrive_SetVoltageLimit(xSjcActuatorLimits_t * Limits, const float value);
float SJC_MotorDrive_GetCurrentLimit(xSjcActuatorLimits_t * Limits);
float SJC_MotorDrive_SetCurrentLimit(xSjcActuatorLimits_t * Limits, const float value);


#endif /* SJC__MODULES__DC_MOTOR_DRIVE_H_ */
