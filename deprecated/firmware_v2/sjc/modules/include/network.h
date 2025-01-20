/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__NETWORK_H_
#define SJC__MODULES__NETWORK_H_

#include "types.h"

eSjcReturnType_t Network_init(const uint32_t xSlaveAddress);
eSjcReturnType_t Network_getAddress(uint32_t *xAddress);
eSjcReturnType_t Network_getBaudRate(uint32_t *xBaudRate);
eSjcReturnType_t Network_getParameters(uint16_t *xSlaveAddress, uint32_t *xBaudRate, char *xParity);
void Network_update(void);
void Network_processReceivedData(void);

#endif /* MOTOR_H_ */
