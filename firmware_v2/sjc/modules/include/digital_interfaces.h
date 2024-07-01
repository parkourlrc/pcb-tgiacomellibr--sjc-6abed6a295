/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__DIGITAL_INTERFACES_H_
#define SJC__MODULES__DIGITAL_INTERFACES_H_

#include "types.h"

void DigitalInterfaces_init(void);

eSjcReturnType_t DigitalInterfaces_readInputState(const eSjcDigitalInputs_t input, bool *state);
eSjcReturnType_t DigitalInterfaces_readOutputState(const eSjcDigitalOutputs_t output, bool *state);
eSjcReturnType_t DigitalInterfaces_toggleOutputState(const eSjcDigitalOutputs_t output);
eSjcReturnType_t DigitalInterfaces_writeOutputState(const eSjcDigitalOutputs_t output, const bool state);

eSjcReturnType_t DigitalInterfaces_readInputStates(uint32_t *states);
eSjcReturnType_t DigitalInterfaces_readOutputStates(uint32_t *states);
eSjcReturnType_t DigitalInterfaces_writeOutputStates(const uint32_t states);

#endif /* SJC__MODULES__DIGITAL_INTERFACES_H_ */
