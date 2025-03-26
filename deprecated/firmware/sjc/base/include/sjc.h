/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__BASE__SJC_H_
#define SJC__BASE__SJC_H_

#include "system_types/include/types.h"

void SJC_Init(void);
void SJC_InitOsKernel(void);
void SJC_PresentSystemInfo(void);
eSjcReturnType_t SJC_SetVersions(void);

eSjcReturnType_t SJC_Delay(const eSjcTimeType_t type, const uint32_t value);
eSjcReturnType_t SJC_GetTime(const eSjcTimeType_t type, uint32_t *value);
eSjcReturnType_t SJC_GetElapsedTime(const eSjcTimeType_t type, const uint32_t timepoint, uint32_t *value);

void SJC_ExecuteNetworkDaemon(void);
void SJC_ProcessNetworkData(void);
void SJC_UpdateHoldingRegisters(void);
void SJC_UpdateInputRegisters(void);

eSjcReturnType_t SJC_enableAdc(void);
eSjcReturnType_t SJC_disableAdc(void);

eSjcReturnType_t SJC_enableJointActuatorDriver(void);
eSjcReturnType_t SJC_disableJointActuatorDriver(void);
eSjcReturnType_t SJC_getJointActuatorDriverState(bool *state);

eSjcReturnType_t SJC_enableJointActuatorOutput(void);
eSjcReturnType_t SJC_disableJointActuatorOutput(void);
eSjcReturnType_t SJC_getJointActuatorOutputState(bool *state);

eSjcReturnType_t SJC_getJointActuatorPwmFrequency(uint32_t *freq);
eSjcReturnType_t SJC_getJointActuatorPowerSupplyVoltage(float *value);

eSjcReturnType_t SJC_getJointActuatorCurrent(float *value);
eSjcReturnType_t SJC_getJointActuatorCurrentUnbiased(float *value);
eSjcReturnType_t SJC_setJointActuatorCurrentBias(const float bias);
eSjcReturnType_t SJC_getJointActuatorVoltage(float *voltage);
eSjcReturnType_t SJC_setJointActuatorVoltage(const float voltage);

eSjcReturnType_t SJC_resetJointAngularDisplacement(void);
eSjcReturnType_t SJC_getJointRelativeAndAngularDisplacement(const eSjcJoints_t joint, float *disp);
eSjcReturnType_t SJC_getJointAngularDisplacement(const eSjcJoints_t joint, float *disp);
eSjcReturnType_t SJC_getJointRelativeAngularDisplacement(const eSjcJoints_t joint, float *RelativeDisp, const float PrevDisp);

eSjcReturnType_t SJC_readDigitalInputs(uint32_t *states);
eSjcReturnType_t SJC_readDigitalOutputs(uint32_t *states);
eSjcReturnType_t SJC_writeDigitalOutputs(const uint8_t states);
eSjcReturnType_t SJC_toggleDigitalOutput(const eSjcDigitalOutputs_t Output);
eSjcReturnType_t SJC_readDigitalInput(const eSjcDigitalInputs_t Input, bool *state);
eSjcReturnType_t SJC_readDigitalOutput(const eSjcDigitalOutputs_t Output, bool *state);
eSjcReturnType_t SJC_writeDigitalOutput(const eSjcDigitalOutputs_t Output, const bool state);

eSjcReturnType_t SJC_getControlMode(eSjcControlMode_t *config);
eSjcReturnType_t SJC_setControlMode(const eSjcControlMode_t config);
eSjcReturnType_t SJC_getControlUpdateRateHz(float *config);
eSjcReturnType_t SJC_setControlUpdateRateHz(const float config);
eSjcReturnType_t SJC_getControlUpdatePeriodMs(uint32_t *config);
eSjcReturnType_t SJC_setControlUpdatePeriodMs(const uint32_t config);

eSjcReturnType_t SJC_getControlError(float *controller_error);
eSjcReturnType_t SJC_getControlReference(float *reference);
eSjcReturnType_t SJC_setControlReference(const float reference);
eSjcReturnType_t SJC_getControlParameters(xSjcServoPidParameters_t *Parameters);
eSjcReturnType_t SJC_setControlParameters(const xSjcServoPidParameters_t Parameters);
eSjcReturnType_t SJC_computeControlAction(const float ControlError, const float dt, float *ControlAction);

#endif /* SJC__MODULES__SYSTEM_PARAMETERS_H_ */
