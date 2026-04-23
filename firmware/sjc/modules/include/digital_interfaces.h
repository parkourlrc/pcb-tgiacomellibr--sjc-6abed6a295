/*
 * digital_interfaces.h
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  Digital I/O abstraction layer for GPIO access.
 *  GPIO register accesses on STM32 are inherently atomic (single bus-width
 *  aligned reads/writes to GPIOx->IDR / ODR / BSRR), so no RTOS
 *  synchronization primitive is required.
 */

#ifndef SJC__MODULES__DIGITAL_INTERFACES_H_
#define SJC__MODULES__DIGITAL_INTERFACES_H_

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initializes all digital interfaces (input and output mapping).
     *
     * Must be called once during system initialization.
     */
    void DigitalInterfaces_init(void);

    /**
     * @brief Reads the state of a digital input pin.
     *
     * @param input  Digital input identifier
     * @param state  Pointer to store the result (true = HIGH, false = LOW)
     * @return SJC_RET_OK on success, SJC_RET_NOK on error
     */
    eSjcReturnType_t DigitalInterfaces_readInputState(const eSjcDigitalInputs_t input, bool *state);

    /**
     * @brief Reads the state of a digital output pin.
     *
     * @param output Digital output identifier
     * @param state  Pointer to store the result (true = HIGH, false = LOW)
     * @return SJC_RET_OK on success, SJC_RET_NOK on error
     */
    eSjcReturnType_t DigitalInterfaces_readOutputState(const eSjcDigitalOutputs_t output, bool *state);

    /**
     * @brief Toggles the state of a digital output pin.
     *
     * @param output Digital output identifier
     * @return SJC_RET_OK on success, SJC_RET_NOK on error
     */
    eSjcReturnType_t DigitalInterfaces_toggleOutputState(const eSjcDigitalOutputs_t output);

    /**
     * @brief Writes the state of a digital output pin.
     *
     * @param output Digital output identifier
     * @param state  Desired state (true = set, false = reset)
     * @return SJC_RET_OK on success, SJC_RET_NOK on error
     */
    eSjcReturnType_t DigitalInterfaces_writeOutputState(const eSjcDigitalOutputs_t output, const bool state);

    /**
     * @brief Reads the state of all digital input pins simultaneously.
     *
     * Each bit in `states` corresponds to a digital input.
     *
     * @param states Pointer to store bitfield of all input states
     * @return SJC_RET_OK on success, SJC_RET_NOK on error
     */
    eSjcReturnType_t DigitalInterfaces_readInputStates(uint32_t *states);

    /**
     * @brief Reads the state of all digital output pins simultaneously.
     *
     * Each bit in `states` corresponds to a digital output.
     *
     * @param states Pointer to store bitfield of all output states
     * @return SJC_RET_OK on success, SJC_RET_NOK on error
     */
    eSjcReturnType_t DigitalInterfaces_readOutputStates(uint32_t *states);

    /**
     * @brief Writes the state of all digital output pins simultaneously.
     *
     * Each bit in `states` corresponds to a digital output.
     *
     * @param states Bitfield with the desired output states
     * @return SJC_RET_OK on success, SJC_RET_NOK on error
     */
    eSjcReturnType_t DigitalInterfaces_writeOutputStates(const uint32_t states);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__DIGITAL_INTERFACES_H_ */
