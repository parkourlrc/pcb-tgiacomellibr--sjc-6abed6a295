/*
 * sjc.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 *
 *  Description:
 *    Public API for the SJC base module (STM32F446RE).
 *    This module orchestrates initialization, OS startup, network daemon
 *    service, register-map updates (Modbus), I/O accessors, and basic
 *    control-loop configuration/query helpers.
 */

#ifndef SJC__BASE__SJC_H_
#define SJC__BASE__SJC_H_

// Project includes -------------------------------------------------- //
#include "modules/include/analog_interfaces.h" /* eSjcAdcFilterMode_t */
#include "system_types/include/types.h"
// ------------------------------------------------------------------- //

// C++ compatibility ------------------------------------------------- //
#ifdef __cplusplus
extern "C"
{
#endif
    // ------------------------------------------------------------------- //

    /* ================================================================== */
    /* System / OS lifecycle                                              */
    /* ================================================================== */

    /**
     * @brief Initialize board peripherals, drivers, controllers and RTOS.
     *
     * - Initializes GPIO, DMA, timers, ADC, UART/USB, encoders, PWM, etc.
     * - Initializes modules (logger, debug, analog/digital IFs, motor drive/output, PID).
     * - Sets default control mode/rate and spawns RTOS threads.
     * - Blocks inside the scheduler after startup (never returns).
     *
     * @note Must be called once at boot from the main context.
     */
    void SJC_init(void);

    /**
     * @brief Initialize the RTOS kernel (scheduler only).
     * @note Called by SJC_init(). Exposed for unit/integration tests.
     */
    void SJC_initOsKernel(void);

    /**
     * @brief Create RTOS synchronization objects (queues, semaphores, timers).
     * @note Called by SJC_init(). Safe to call once.
     */
    void SJC_initOsObjects(void);

    /**
     * @brief Create RTOS threads/tasks required by the system.
     * @note Starts the scheduler and does not return (idle loop fallback).
     */
    void SJC_initOsThreads(void);

    /**
     * @brief Print system information (HW/FW, build, features) through debug output.
     */
    void SJC_presentSystemInfo(void);

    /**
     * @brief Write HW/FW version fields into input registers.
     * @return SJC_RET_OK on success.
     *
     * @details
     *   - HW version: major/minor packed into a single 16-bit register.
     *   - FW version: major/minor in first 16-bit register; patch in the next.
     */
    eSjcReturnType_t SJC_setVersions(void);

    /* ================================================================== */
    /* Time utilities                                                     */
    /* ================================================================== */

    /**
     * @brief Busy-wait delay using the timeout timer.
     * @param type  Time unit selector (ms or s).
     * @param value Amount of time (must be > 0).
     * @return SJC_RET_OK if delayed, SJC_RET_NOK on invalid args.
     */
    eSjcReturnType_t SJC_delay(const eSjcTimeType_t type, const uint32_t value);

    /**
     * @brief Get current time from the timeout timer.
     * @param type  Time unit selector (ms or s).
     * @param value [out] Current time value.
     * @return SJC_RET_OK on success; SJC_RET_NOK on invalid args.
     */
    eSjcReturnType_t SJC_getTime(const eSjcTimeType_t type, uint32_t *value);

    /**
     * @brief Compute elapsed time handling 32-bit wrap-around.
     * @param type       Time unit selector (ms or s).
     * @param timepoint  Reference time (previously sampled).
     * @param value      [out] Elapsed time since @p timepoint.
     * @return SJC_RET_OK on success; SJC_RET_NOK on invalid args.
     */
    eSjcReturnType_t SJC_getElapsedTime(const eSjcTimeType_t type, const uint32_t timepoint, uint32_t *value);

    /* ================================================================== */
    /* Network daemon & register map                                      */
    /* ================================================================== */

    /**
     * @brief Execute the Modbus network service (polling/daemon).
     * @details Typically called from a periodic RTOS task.
     */
    void SJC_executeNetworkDaemon(void);

    /**
     * @brief Process holding-register commands (writes) and apply effects.
     * @details Decodes and executes updates for system/network/sensors/actuator,
     *          controller mode/reference, controller parameters, ADC filter config.
     */
    void SJC_updateHoldingRegisters(void);

    /**
     * @brief Refresh input-register telemetry/readbacks.
     * @details Publishes runtime info, network parameters, sensors, actuator, and
     *          controller parameters into the input-register table.
     */
    void SJC_updateInputRegisters(void);

    /* ================================================================== */
    /* Analog front-end (ADC)                                             */
    /* ================================================================== */

    /**
     * @brief Enable analog front-end acquisition.
     */
    eSjcReturnType_t SJC_enableAdc(void);

    /**
     * @brief Disable analog front-end acquisition.
     */
    eSjcReturnType_t SJC_disableAdc(void);

    /**
     * @brief Configure ADC front-end filtering mode.
     * @param mode Filter mode selector.
     * @param fc   Cutoff frequency in Hz (used by applicable modes).
     */
    eSjcReturnType_t SJC_setAdcFilterMode(const eSjcAdcFilterMode_t mode, const float fc);

    /* ================================================================== */
    /* Actuator driver/output & power                                     */
    /* ================================================================== */

    /**
     * @brief Enable actuator power driver (gate/enable line).
     */
    eSjcReturnType_t SJC_enableJointActuatorDriver(void);

    /**
     * @brief Disable actuator power driver (gate/enable line).
     */
    eSjcReturnType_t SJC_disableJointActuatorDriver(void);

    /**
     * @brief Query actuator driver enable state.
     * @param state [out] true if enabled.
     */
    eSjcReturnType_t SJC_getJointActuatorDriverState(bool *state);

    /**
     * @brief Enable actuator output stage (PWM/bridge output).
     */
    eSjcReturnType_t SJC_enableJointActuatorOutput(void);

    /**
     * @brief Disable actuator output stage (PWM/bridge output).
     */
    eSjcReturnType_t SJC_disableJointActuatorOutput(void);

    /**
     * @brief Query actuator output stage state.
     * @param state [out] true if enabled.
     */
    eSjcReturnType_t SJC_getJointActuatorOutputState(bool *state);

    /**
     * @brief Read the PWM carrier frequency for the actuator.
     * @param freq [out] Frequency in Hz.
     */
    eSjcReturnType_t SJC_getJointActuatorPwmFrequency(uint32_t *freq);

    /**
     * @brief Read DC bus (power supply) voltage.
     * @param value [out] Voltage in volts [V].
     */
    eSjcReturnType_t SJC_getJointActuatorPowerSupplyVoltage(float *value);

    /**
     * @brief Read actuator output current.
     * @param value [out] Current in amperes [A].
     */
    eSjcReturnType_t SJC_getJointActuatorCurrent(float *value);

    /**
     * @brief Read actuator output current with bias removed.
     * @param value [out] Current in amperes [A], bias-compensated.
     */
    eSjcReturnType_t SJC_getJointActuatorCurrentUnbiased(float *value);

    /**
     * @brief Set current bias to be subtracted in the unbiased reading.
     * @param bias Bias value [A].
     */
    eSjcReturnType_t SJC_setJointActuatorCurrentBias(const float bias);

    /**
     * @brief Read commanded actuator output voltage.
     * @param voltage [out] Voltage in volts [V].
     */
    eSjcReturnType_t SJC_getJointActuatorVoltage(float *voltage);

    /**
     * @brief Set commanded actuator output voltage.
     * @param voltage Voltage in volts [V].
     */
    eSjcReturnType_t SJC_setJointActuatorVoltage(const float voltage);

    /* ================================================================== */
    /* Joint sensing (positions & velocities)                             */
    /* ================================================================== */

    /**
     * @brief Reset the primary (actuator) joint angular position to zero.
     */
    eSjcReturnType_t SJC_resetJointAngularPosition(void);

    /**
     * @brief Reset auxiliary joints angular position to zero (THX, THY).
     */
    eSjcReturnType_t SJC_resetAuxJointsAngularPosition(void);

    /**
     * @brief Read absolute angular position for a given joint.
     * @param joint Joint selector.
     * @param disp  [out] Angular displacement in radians [rad].
     */
    eSjcReturnType_t SJC_getJointAngularPosition(const eSjcJoints_t joint, float *disp);

    /**
     * @brief Read relative angular displacement since a previous absolute reading.
     * @param joint          Joint selector.
     * @param disp           [out] Relative displacement in radians [rad].
     * @param prev_position  Previous absolute position [rad] to subtract.
     */
    eSjcReturnType_t SJC_getJointAngularDisplacement(const eSjcJoints_t joint, float *disp, const float prev_position);

    /**
     * @brief Reset the velocity estimate for a given joint (filter reset).
     */
    eSjcReturnType_t SJC_resetJointVelocity(const eSjcJoints_t joint);

    /**
     * @brief Read filtered angular velocity for a given joint.
     * @param joint Joint selector.
     * @param vel   [out] Angular velocity in radians per second [rad/s].
     */
    eSjcReturnType_t SJC_getJointAngularVelocity(const eSjcJoints_t joint, float *vel);

    /**
     * @brief Update the joint velocity measurement (feeds the internal filter).
     * @param joint Joint selector.
     * @param vel   Raw/angular velocity sample [rad/s].
     */
    eSjcReturnType_t SJC_updateJointVelocity(const eSjcJoints_t joint, const float vel);

    /* ================================================================== */
    /* Digital I/O                                                        */
    /* ================================================================== */

    /**
     * @brief Read all digital inputs as a bitfield.
     * @param states [out] Bitfield of inputs (implementation-defined mapping).
     */
    eSjcReturnType_t SJC_readDigitalInputs(uint32_t *states);

    /**
     * @brief Read all digital outputs as a bitfield.
     * @param states [out] Bitfield of outputs (implementation-defined mapping).
     */
    eSjcReturnType_t SJC_readDigitalOutputs(uint32_t *states);

    /**
     * @brief Write all digital outputs at once from an 8-bit mask.
     * @param states Bitfield (LSB-first) with desired output states.
     */
    eSjcReturnType_t SJC_writeDigitalOutputs(const uint8_t states);

    /**
     * @brief Toggle a single digital output.
     * @param Output Output selector.
     */
    eSjcReturnType_t SJC_toggleDigitalOutput(const eSjcDigitalOutputs_t Output);

    /**
     * @brief Read a single digital input line.
     * @param Input Input selector.
     * @param state [out] true if asserted.
     */
    eSjcReturnType_t SJC_readDigitalInput(const eSjcDigitalInputs_t Input, bool *state);

    /**
     * @brief Read a single digital output line.
     * @param Output Output selector.
     * @param state  [out] true if asserted.
     */
    eSjcReturnType_t SJC_readDigitalOutput(const eSjcDigitalOutputs_t Output, bool *state);

    /**
     * @brief Write a single digital output line.
     * @param Output Output selector.
     * @param state  Desired state (true=asserted).
     */
    eSjcReturnType_t SJC_writeDigitalOutput(const eSjcDigitalOutputs_t Output, const bool state);

    /* ================================================================== */
    /* Control configuration & telemetry                                  */
    /* ================================================================== */

    /**
     * @brief Get current control mode.
     * @param config [out] Mode enum.
     */
    eSjcReturnType_t SJC_getControlMode(eSjcControlMode_t *config);

    /**
     * @brief Set control mode.
     * @param config Mode enum. If SJC_ANALOG_CALIBRATION, output is forced to 0 V.
     */
    eSjcReturnType_t SJC_setControlMode(const eSjcControlMode_t config);

    /**
     * @brief Get controller update rate.
     * @param config [out] Rate in Hertz [Hz].
     */
    eSjcReturnType_t SJC_getControlUpdateRateHz(float *config);

    /**
     * @brief Set controller update rate.
     * @param config Rate in Hertz [Hz]. Clamped to allowed range.
     */
    eSjcReturnType_t SJC_setControlUpdateRateHz(const float config);

    /**
     * @brief Get controller update period.
     * @param period_ms [out] Period in milliseconds [ms].
     * @note Uses simple truncation (no rounding).
     */
    eSjcReturnType_t SJC_getControlUpdatePeriodMs(uint32_t *period_ms);

    /**
     * @brief Set controller update period.
     * @param period_ms Period in milliseconds [ms]. Internally converted to Hz.
     */
    eSjcReturnType_t SJC_setControlUpdatePeriodMs(const uint32_t period_ms);

    /**
     * @brief Get instantaneous control error.
     * @param controller_error [out] Error value in controller units (e.g., rad or rad/s).
     */
    eSjcReturnType_t SJC_getControlError(float *controller_error);

    /**
     * @brief Get controller reference setpoint.
     * @param reference [out] Setpoint in controller units.
     */
    eSjcReturnType_t SJC_getControlReference(float *reference);

    /**
     * @brief Set controller reference setpoint.
     * @param reference Setpoint in controller units.
     */
    eSjcReturnType_t SJC_setControlReference(const float reference);

    /**
     * @brief Get PID-like controller parameters (Kp, Ki, Kd, KT, KA).
     * @param Parameters [out] Structure with current parameters.
     */
    eSjcReturnType_t SJC_getControlParameters(xSjcServoPidParameters_t *Parameters);

    /**
     * @brief Set PID-like controller parameters (Kp, Ki, Kd, KT, KA).
     * @param Parameters Structure with parameters to apply.
     */
    eSjcReturnType_t SJC_setControlParameters(const xSjcServoPidParameters_t Parameters);

    /**
     * @brief Compute control action for a given error and dt.
     * @param ControlError Control error in controller units.
     * @param dt           Time step in seconds [s].
     * @param ControlAction [out] Commanded effort (e.g., volts).
     */
    eSjcReturnType_t SJC_computeControlAction(const float ControlError, const float dt, float *ControlAction);

// C++ compatibility ------------------------------------------------- //
#ifdef __cplusplus
} /* extern "C" */
#endif
// ------------------------------------------------------------------- //

#endif /* SJC__BASE__SJC_H_ */
