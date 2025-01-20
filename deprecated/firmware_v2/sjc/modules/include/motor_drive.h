/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__DC_MOTOR_DRIVE_H_
#define SJC__MODULES__DC_MOTOR_DRIVE_H_

#include "types.h"

void MotorDrive_init(eSjcPwmBaseFreq_t BaseFrequency, float BaseVoltage);
void MotorDrive_enable(void);
void MotorDrive_disable(void);

eSjcReturnType_t MotorDrive_getState(bool *state);
eSjcReturnType_t MotorDrive_getPowerSupplyVoltage(float *voltage);

eSjcReturnType_t MotorDrive_getBaseVoltage(float *voltage);
eSjcReturnType_t MotorDrive_setBaseVoltage(float voltage);

eSjcReturnType_t MotorDrive_getOutputCurrent(float *current);
eSjcReturnType_t MotorDrive_getOutputVoltage(float *voltage);
eSjcReturnType_t MotorDrive_setOutputVoltage(const float voltage);

#endif /* SJC__MODULES__DC_MOTOR_DRIVE_H_ */
