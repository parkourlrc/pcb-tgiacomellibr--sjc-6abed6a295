/*
 * test_signals.h
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  This module provides utilities to generate predefined input signals
 *  (PRBS, triangular, fast pulse, slow pulse) for actuator and control
 *  testing. The time scaling is based directly on the controller
 *  sampling period (Ts), provided in seconds, ensuring that all signals
 *  preserve their real-time duration proportions across a wide range
 *  (from 1 kHz to 10 Hz control loop frequencies).
 */

#ifndef SJC__MODULES__TEST_SIGNALS_H_
#define SJC__MODULES__TEST_SIGNALS_H_

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the test signal generator.
 *
 * Sets the internal state to INIT_CONFIG and clears counters.
 */
void TestSignals_init(void);

/**
 * @brief Reset the test signal generator.
 *
 * Resets the internal state machine and counters, allowing the
 * signal generator to be reused.
 */
void TestSignals_reset(void);

/**
 * @brief Configure and start a test signal.
 *
 * @param type Signal type to generate (see eSjcInputType_t).
 * @param Ts   Controller sampling period in **seconds**.
 *             - Example: Ts = 0.001f → 1 kHz loop
 *             - Example: Ts = 0.010f → 100 Hz loop
 *             - Example: Ts = 0.100f → 10 Hz loop
 *
 * The signal generator internally computes the required number
 * of samples for each transition based on Ts, ensuring that the
 * real-time duration of each signal remains constant regardless
 * of the control loop frequency.
 *
 * @return SJC_RET_OK if configuration was successful.
 */
eSjcReturnType_t TestSignals_setConfig(const eSjcInputType_t type,
                                       const float Ts);

/**
 * @brief Get the current state of the test signal generator.
 *
 * @param[out] state Current state:
 *                   - INIT_CONFIG
 *                   - RUNNING
 *                   - FINISHED
 *
 * @return SJC_RET_OK on success.
 */
eSjcReturnType_t TestSignals_getState(eSjcInputSignalState_t *state);

/**
 * @brief Retrieve the current value of the active test signal.
 *
 * This function advances the signal generator by one sample.
 * The output depends on the configured type and Ts. Once the
 * sequence ends, the state changes to FINISHED and the output
 * value becomes 0.0f.
 *
 * @param[out] value Pointer to store the current signal value.
 *
 * @return SJC_RET_OK on success.
 */
eSjcReturnType_t TestSignals_getValue(float *value);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__TEST_SIGNALS_H_ */
