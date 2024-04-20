/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__NETWORK_H_
#define SJC__MODULES__NETWORK_H_

#include "types.h"

eSjcReturnType_t SJC_Network_Init(const uint32_t xSlaveAddress);
eSjcReturnType_t SJC_Network_GetBaudRate(uint32_t *xBaudRate);
eSjcReturnType_t SJC_Network_GetParameters(uint16_t *xSlaveAddress, uint32_t *xBaudRate, char *xParity);
void SJC_Network_Update(void);
void SJC_Network_ProcessReceivedData(void);

#endif /* MOTOR_H_ */
