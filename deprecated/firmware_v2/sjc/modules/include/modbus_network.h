/*
 * modbus_network.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__MODBUS_NETWORK_H
#define SJC__MODULES__MODBUS_NETWORK_H

#include "system_types/include/types.h"

eSjcReturnType_t ModbusNetwork_init(const uint32_t xSlaveAddress);
eSjcReturnType_t ModbusNetwork_getAddress(uint32_t *xAddress);
eSjcReturnType_t ModbusNetwork_getBaudRate(uint32_t *xBaudRate);
eSjcReturnType_t ModbusNetwork_getParameters(uint16_t *xSlaveAddress, uint32_t *xBaudRate, char *xParity);
void ModbusNetwork_update(void);

#endif /* MOTOR_H_ */
