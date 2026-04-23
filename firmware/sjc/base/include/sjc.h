/*
 * sjc.h
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  Public API for the SJC base module (STM32F446RE).
 *  Handles initialization, OS startup, network daemon,
 *  Modbus register updates, I/O accessors, and control-loop configuration.
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
    /* Control configuration & telemetry                                  */
    /* ================================================================== */

    /** Get control mode. */
    eSjcReturnType_t SJC_getControlMode(eSjcControlMode_t *config);

    /** Set control mode. */
    eSjcReturnType_t SJC_setControlMode(const eSjcControlMode_t config);

    /** Set control mode and reset the active controller atomically. */
    eSjcReturnType_t SJC_setControlModeAndReset(const eSjcControlMode_t config);

    /** Get control update rate [Hz]. */
    eSjcReturnType_t SJC_getControlUpdateRateHz(float *config);

    /** Set control update rate [Hz]. */
    eSjcReturnType_t SJC_setControlUpdateRateHz(const float config);

    /** Get all control configs in a single mutex-protected call. */
    eSjcReturnType_t SJC_getControlConfigs(xSjcControlConfigs_t *configs);

    /** Get instantaneous control error. */
    eSjcReturnType_t SJC_getControlError(float *controller_error);

    /** Get controller reference setpoint. */
    eSjcReturnType_t SJC_getControlReference(float *reference);

    /** Get all control configs and error in a single mutex-protected call. */
    eSjcReturnType_t SJC_getControlStatus(xSjcControlConfigs_t *configs, float *error);

    /** Set controller reference setpoint. */
    eSjcReturnType_t SJC_setControlReference(const float reference);

    /** Reset active controller (PID, LQR, or discrete). */
    eSjcReturnType_t SJC_resetController(void);

// ------------------------------------------------------------------- //
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SJC__BASE__SJC_H_ */
