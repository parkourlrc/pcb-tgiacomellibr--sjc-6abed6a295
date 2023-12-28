/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__ACTUATOR_H_
#define SJC__MODULES__ACTUATOR_H_

#include "types.h"

void SJC_Actuator_Init(xSjcActuator_t * Actuator, eSjcPwmBaseFreq_t BaseFrequency, float BaseVoltage);
void SJC_Actuator_Enable(xSjcActuator_t * Actuator);
void SJC_Actuator_Disable(xSjcActuator_t * Actuator);

float SJC_Actuator_GetPowerSupplyState(void);

float SJC_Actuator_GetVoltage(xSjcActuator_t * Actuator);
float SJC_Actuator_SetVoltage(xSjcActuator_t * Actuator, const float voltage);

float SJC_Actuator_GetSpeedLimit(xSjcActuatorLimits_t * Limits);
float SJC_Actuator_SetSpeedLimit(xSjcActuatorLimits_t * Limits, const float value);
float SJC_Actuator_GetVoltageLimit(xSjcActuatorLimits_t * Limits);
float SJC_Actuator_SetVoltageLimit(xSjcActuatorLimits_t * Limits, const float value);
float SJC_Actuator_GetCurrentLimit(xSjcActuatorLimits_t * Limits);
float SJC_Actuator_SetCurrentLimit(xSjcActuatorLimits_t * Limits, const float value);

float SJC_Actuator_GetBaseVoltage(xSjcActuator_t * Actuator);
eSjcReturnType_t SJC_Actuator_SetBaseVoltage(xSjcActuator_t * Actuator, float voltage);

#endif /* MOTOR_H_ */
