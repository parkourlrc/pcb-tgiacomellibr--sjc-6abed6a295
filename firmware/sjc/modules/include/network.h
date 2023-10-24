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
void SJC_Network_Update(void);

#endif /* MOTOR_H_ */
