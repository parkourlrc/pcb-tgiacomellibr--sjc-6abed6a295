/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__SYSTEM_PARAMETERS_H_
#define SJC__MODULES__SYSTEM_PARAMETERS_H_

#include "types.h"

void SJC_Init(void);
eSjcReturnType_t SJC_InitOsTasks(void);
void SJC_PresentSystemInfo(void);

eSjcReturnType_t SJC_Delay(const eSjcDelayType_t type, const uint32_t value);
eSjcReturnType_t SJC_GetTime(const eSjcDelayType_t type, uint32_t *value);

eSjcControlMode_t SJC_GetControlMode(void);
uint32_t SJC_GetControlUpdateRateHz(void);
uint32_t SJC_GetControlUpdatePeriodMs(void);

void SJC_ReadNetworkCommands(void);
void SJC_UpdateNetworkRegisters(void);

eSjcReturnType_t SJC_InitAdc(void);
eSjcReturnType_t SJC_EnableAdc(uint32_t * buffer);
eSjcReturnType_t SJC_DisableAdc(void);

eSjcReturnType_t SJC_EnableJointActuators(void);
eSjcReturnType_t SJC_DisableJointActuators(void);
eSjcReturnType_t SJC_GetJointStates(const eSjcJoints_t sel, xSjcJoint_t * Joint);
eSjcReturnType_t SJC_SetJointActuatorVoltage(float voltage);
eSjcReturnType_t SJC_GetJointActuatorFrequency(uint32_t * freq);
eSjcReturnType_t SJC_SetJointActuatorFilteredPsuVoltage(const float value);
eSjcReturnType_t SJC_SetJointActuatorFilteredOutputCurrent(const float value);

eSjcReturnType_t SJC_ReadDigitalInputs(uint8_t * states);
eSjcReturnType_t SJC_ReadDigitalOutputs(uint8_t * states);
eSjcReturnType_t SJC_WriteDigitalOutputs(const uint8_t states);
eSjcReturnType_t SJC_ReadDigitalInput(const eSjcDigitalInputs_t Input, bool * state);
eSjcReturnType_t SJC_ReadDigitalOutput(const eSjcDigitalOutputs_t Output, bool * state);
eSjcReturnType_t SJC_WriteDigitalOutput(const eSjcDigitalOutputs_t Output, const bool state);

#endif /* SJC__MODULES__SYSTEM_PARAMETERS_H_ */
