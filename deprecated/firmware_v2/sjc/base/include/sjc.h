/*
 * sjc.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 *
 *  Description:
 *    Public API for the SJC base module (STM32F446RE).
 *    Handles initialization, OS startup, network daemon,
 *    Modbus register updates, I/O accessors, and control-loop configuration.
 */

#ifndef SJC__BASE__SJC_H_
#define SJC__BASE__SJC_H_

// Project includes -------------------------------------------------- //
#include "modules/include/analog_interfaces.h" /* eSjcAdcFilterMode_t */
#include "system_types/include/types.h"
// ------------------------------------------------------------------- //

#ifdef __cplusplus
extern "C"
{
#endif
    // ------------------------------------------------------------------- //

    /* ================================================================== */
    /* System / OS lifecycle                                              */
    /* ================================================================== */

    /** Initialize board peripherals, drivers, modules, and RTOS. */
    void SJC_init(void);

    /** Initialize the RTOS kernel (scheduler only). */
    void SJC_initOsKernel(void);

    /** Create RTOS synchronization objects. */
    void SJC_initOsObjects(void);

    /** Create RTOS threads and start the scheduler (does not return). */
    void SJC_initOsThreads(void);

    /** Print system information (HW/FW, build, features). */
    void SJC_presentSystemInfo(void);

    /** Write HW/FW version fields into input registers. */
    eSjcReturnType_t SJC_setVersions(void);

    /* ================================================================== */
    /* Time utilities                                                     */
    /* ================================================================== */

    /** Busy-wait delay. */
    eSjcReturnType_t SJC_delay(const eSjcTimeType_t type, const uint32_t value);

    /** Get current time. */
    eSjcReturnType_t SJC_getTime(const eSjcTimeType_t type, uint32_t *value);

    /** Compute elapsed time since a given timepoint. */
    eSjcReturnType_t SJC_getElapsedTime(const eSjcTimeType_t type, const uint32_t timepoint, uint32_t *value);

    /* ================================================================== */
    /* Network daemon & register map                                      */
    /* ================================================================== */

    /** Run Modbus network service (polling/daemon). */
    void SJC_executeNetworkDaemon(void);

    /** Process holding-register commands and apply effects. */
    void SJC_updateHoldingRegisters(void);

    /** Refresh input-register telemetry. */
    void SJC_updateInputRegisters(void);

    /* ================================================================== */
    /* Analog front-end (ADC)                                             */
    /* ================================================================== */

    /** Enable analog acquisition. */
    eSjcReturnType_t SJC_enableAdc(void);

    /** Disable analog acquisition. */
    eSjcReturnType_t SJC_disableAdc(void);

    /* ================================================================== */
    /* Actuator driver/output & power                                     */
    /* ================================================================== */

    /** Enable actuator power driver. */
    eSjcReturnType_t SJC_enableJointActuatorDriver(void);

    /** Disable actuator power driver. */
    eSjcReturnType_t SJC_disableJointActuatorDriver(void);

    /** Get actuator driver state. */
    eSjcReturnType_t SJC_getJointActuatorDriverState(bool *state);

    /** Enable actuator output stage. */
    eSjcReturnType_t SJC_enableJointActuatorOutput(void);

    /** Disable actuator output stage. */
    eSjcReturnType_t SJC_disableJointActuatorOutput(void);

    /** Get actuator output stage state. */
    eSjcReturnType_t SJC_getJointActuatorOutputState(bool *state);

    /** Get actuator PWM carrier frequency [Hz]. */
    eSjcReturnType_t SJC_getJointActuatorPwmFrequency(uint32_t *freq);

    /** Get DC bus voltage [V]. */
    eSjcReturnType_t SJC_getJointActuatorPowerSupplyVoltage(float *value);

    /** Get actuator current [A]. */
    eSjcReturnType_t SJC_getJointActuatorCurrent(float *value);

    /** Get actuator current without bias [A]. */
    eSjcReturnType_t SJC_getJointActuatorCurrentUnbiased(float *value);

    /** Set actuator current bias [A]. */
    eSjcReturnType_t SJC_setJointActuatorCurrentBias(const float bias);

    /** Get commanded actuator voltage [V]. */
    eSjcReturnType_t SJC_getJointActuatorVoltage(float *voltage);

    /** Set commanded actuator voltage [V]. */
    eSjcReturnType_t SJC_setJointActuatorVoltage(const float voltage);

    /* ================================================================== */
    /* Joint sensing (positions & velocities)                             */
    /* ================================================================== */

    /** Reset primary joint angular position. */
    eSjcReturnType_t SJC_resetJointAngularPosition(void);

    /** Reset auxiliary joints angular positions. */
    eSjcReturnType_t SJC_resetAuxJointsAngularPosition(void);

    /** Get absolute angular position [rad]. */
    eSjcReturnType_t SJC_getJointAngularPosition(const eSjcJoints_t joint, float *disp);

    /** Get relative angular displacement since previous sample [rad]. */
    eSjcReturnType_t SJC_getJointAngularDisplacement(const eSjcJoints_t joint, float *disp, const float prev_position);

    /** Reset velocity estimate for a joint. */
    eSjcReturnType_t SJC_resetJointVelocity(const eSjcJoints_t joint);

    /** Get angular velocity [rad/s]. */
    eSjcReturnType_t SJC_getJointAngularVelocity(const eSjcJoints_t joint, float *vel);

    /** Update angular velocity measurement [rad/s]. */
    eSjcReturnType_t SJC_updateJointVelocity(const eSjcJoints_t joint, const float vel);

    /* ================================================================== */
    /* Digital I/O                                                        */
    /* ================================================================== */

    /** Read all digital inputs as bitfield. */
    eSjcReturnType_t SJC_readDigitalInputs(uint32_t *states);

    /** Read all digital outputs as bitfield. */
    eSjcReturnType_t SJC_readDigitalOutputs(uint32_t *states);

    /** Write all digital outputs (8-bit mask). */
    eSjcReturnType_t SJC_writeDigitalOutputs(const uint8_t states);

    /** Toggle a single digital output. */
    eSjcReturnType_t SJC_toggleDigitalOutput(const eSjcDigitalOutputs_t Output);

    /** Read a single digital input. */
    eSjcReturnType_t SJC_readDigitalInput(const eSjcDigitalInputs_t Input, bool *state);

    /** Read a single digital output. */
    eSjcReturnType_t SJC_readDigitalOutput(const eSjcDigitalOutputs_t Output, bool *state);

    /** Write a single digital output. */
    eSjcReturnType_t SJC_writeDigitalOutput(const eSjcDigitalOutputs_t Output, const bool state);

    /* ================================================================== */
    /* Control configuration & telemetry                                  */
    /* ================================================================== */

    /** Get control mode. */
    eSjcReturnType_t SJC_getControlMode(eSjcControlMode_t *config);

    /** Set control mode. */
    eSjcReturnType_t SJC_setControlMode(const eSjcControlMode_t config);

    /** Get control update rate [Hz]. */
    eSjcReturnType_t SJC_getControlUpdateRateHz(float *config);

    /** Set control update rate [Hz]. */
    eSjcReturnType_t SJC_setControlUpdateRateHz(const float config);

    /** Get control update period [ms]. */
    eSjcReturnType_t SJC_getControlUpdatePeriodMs(uint32_t *period_ms);

    /** Set control update period [ms]. */
    eSjcReturnType_t SJC_setControlUpdatePeriodMs(const uint32_t period_ms);

    /** Get instantaneous control error. */
    eSjcReturnType_t SJC_getControlError(float *controller_error);

    /** Get controller reference setpoint. */
    eSjcReturnType_t SJC_getControlReference(float *reference);

    /** Set controller reference setpoint. */
    eSjcReturnType_t SJC_setControlReference(const float reference);

    /** Reset active controller (PID, LQR, or discrete). */
    eSjcReturnType_t SJC_resetController(void);

// ------------------------------------------------------------------- //
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SJC__BASE__SJC_H_ */
