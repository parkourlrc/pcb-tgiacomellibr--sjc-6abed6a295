/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__DIGITAL_INTERFACES_H_
#define SJC__MODULES__DIGITAL_INTERFACES_H_

#include "system_types/include/types.h"

void SJC_DigitalInterfaces_init(void);

eSjcReturnType_t SJC_DigitalInterfaces_ReadInputState(const eSjcDigitalInputs_t input, bool *state);
eSjcReturnType_t SJC_DigitalInterfaces_ReadOutputState(const eSjcDigitalOutputs_t output, bool *state);
eSjcReturnType_t SJC_DigitalInterfaces_ToggleOutputState(const eSjcDigitalOutputs_t output);
eSjcReturnType_t SJC_DigitalInterfaces_WriteOutputState(const eSjcDigitalOutputs_t output, const bool state);

eSjcReturnType_t SJC_DigitalInterfaces_ReadInputStates(uint32_t *states);
eSjcReturnType_t SJC_DigitalInterfaces_ReadOutputStates(uint32_t *states);
eSjcReturnType_t SJC_DigitalInterfaces_WriteOutputStates(const uint32_t states);

#endif /* SJC__MODULES__DIGITAL_INTERFACES_H_ */
