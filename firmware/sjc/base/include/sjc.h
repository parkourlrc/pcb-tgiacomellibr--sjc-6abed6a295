/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__BASE__SJC_H_
#define SJC__BASE__SJC_H_

#include "types.h"

void SJC_Init(void);
void SJC_InitOsKernel(void);
void SJC_PresentSystemInfo(void);
eSjcReturnType_t SJC_SetVersions(void);

eSjcReturnType_t SJC_Delay(const eSjcTimeType_t type, const uint32_t value);
eSjcReturnType_t SJC_GetTime(const eSjcTimeType_t type, uint32_t *value);
eSjcReturnType_t SJC_GetElapsedTime(const eSjcTimeType_t type, const uint32_t timepoint, uint32_t *value);

void SJC_ExecuteNetworkCommands(void);
void SJC_UpdateNetworkDataRegisters(void);

eSjcReturnType_t SJC_EnableAdc(void);
eSjcReturnType_t SJC_DisableAdc(void);

eSjcReturnType_t SJC_EnableJointActuators(void);
eSjcReturnType_t SJC_DisableJointActuators(void);
eSjcReturnType_t SJC_GetJointActuatorState(bool * state);

eSjcReturnType_t SJC_GetJointActuatorPwmFrequency(uint32_t *freq);
eSjcReturnType_t SJC_GetJointActuatorPowerSupplyVoltage(float * value);

eSjcReturnType_t SJC_GetJointActuatorCurrent(float * value);
eSjcReturnType_t SJC_GetJointActuatorVoltage(float * voltage);
eSjcReturnType_t SJC_SetJointActuatorVoltage(const float voltage);

eSjcReturnType_t SJC_ResetJointAngularDisplacement(void);
eSjcReturnType_t SJC_GetJointRelativeAndAngularDisplacement(const eSjcJoints_t joint, float * disp);
eSjcReturnType_t SJC_GetJointAngularDisplacement(const eSjcJoints_t joint, float * disp);
eSjcReturnType_t SJC_GetJointRelativeAngularDisplacement(const eSjcJoints_t joint, float * RelativeDisp, const float PrevDisp);

eSjcReturnType_t SJC_ReadDigitalInputs(uint32_t *states);
eSjcReturnType_t SJC_ReadDigitalOutputs(uint32_t *states);
eSjcReturnType_t SJC_WriteDigitalOutputs(const uint8_t states);
eSjcReturnType_t SJC_ToggleDigitalOutput(const eSjcDigitalOutputs_t Output);
eSjcReturnType_t SJC_ReadDigitalInput(const eSjcDigitalInputs_t Input, bool *state);
eSjcReturnType_t SJC_ReadDigitalOutput(const eSjcDigitalOutputs_t Output, bool *state);
eSjcReturnType_t SJC_WriteDigitalOutput(const eSjcDigitalOutputs_t Output, const bool state);

eSjcReturnType_t SJC_GetControlMode(eSjcControlMode_t * config);
eSjcReturnType_t SJC_SetControlMode(const eSjcControlMode_t config);
eSjcReturnType_t SJC_GetControlUpdateRateHz(float * config);
eSjcReturnType_t SJC_SetControlUpdateRateHz(const float config);
eSjcReturnType_t SJC_GetControlUpdatePeriodMs(uint32_t * config);

eSjcReturnType_t SJC_GetControlError(float * controller_error);
eSjcReturnType_t SJC_GetControlReference(float * reference);
eSjcReturnType_t SJC_SetControlReference(const float reference);
eSjcReturnType_t SJC_GetControlParameters(xPidControllerParameters_t * Parameters);
eSjcReturnType_t SJC_SetControlParameters(const xPidControllerParameters_t Parameters);
eSjcReturnType_t SJC_ComputeControlAction(const float ControlError, const float dt, float * ControlAction);

#endif /* SJC__MODULES__SYSTEM_PARAMETERS_H_ */
