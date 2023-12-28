/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__DIGITAL_INTERFACES_H_
#define SJC__MODULES__DIGITAL_INTERFACES_H_

#include "types.h"

eSjcReturnType_t SJC_DigitalInterfaces_init(xSjcDigitalPort_t *Interfaces, const eSjcDigitalPortType_t type);

eSjcReturnType_t SJC_DigitalInterfaces_ReadInputState(xSjcDigitalPort_t *Interface);
eSjcReturnType_t SJC_DigitalInterfaces_ReadOutputState(xSjcDigitalPort_t *Interface);
eSjcReturnType_t SJC_DigitalInterfaces_ToggleOutputState(xSjcDigitalPort_t *Interface);
eSjcReturnType_t SJC_DigitalInterfaces_WriteOutputState(xSjcDigitalPort_t *Interface, const bool state);

eSjcReturnType_t SJC_DigitalInterfaces_ReadInputStates(xSjcDigitalPort_t *Interfaces, uint16_t * states);
eSjcReturnType_t SJC_DigitalInterfaces_ReadOutputStates(xSjcDigitalPort_t *Interfaces);
eSjcReturnType_t SJC_DigitalInterfaces_WriteOutputStates(xSjcDigitalPort_t *Interface, const uint8_t states);

#endif /* SJC__MODULES__DIGITAL_INTERFACES_H_ */
