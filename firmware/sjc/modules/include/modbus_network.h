/**
 * @file modbus_network.h
 * @brief Modbus RTU network configuration and access API.
 *
 * Provides initialization and access methods for Modbus slave configuration,
 * address, and communication parameters.
 *
 * @author Tiago
 * @date Sep 9, 2023
 */

#ifndef SJC__MODULES__MODBUS_NETWORK_H
#define SJC__MODULES__MODBUS_NETWORK_H

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initializes the Modbus network with a specified slave address.
     *
     * @param xSlaveAddress Modbus slave address to be used.
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise.
     */
    eSjcReturnType_t ModbusNetwork_init(const uint32_t xSlaveAddress);

    /**
     * @brief Retrieves the current network address based on hardware configuration.
     *
     * @param xAddress Pointer to store the retrieved network address.
     * @return SJC_RET_OK if successful.
     */
    eSjcReturnType_t ModbusNetwork_getAddress(uint32_t *xAddress);

    /**
     * @brief Determines the current baud rate from the UART peripheral.
     *
     * @param xBaudRate Pointer to store the detected baud rate.
     * @return SJC_RET_OK if successful.
     */
    eSjcReturnType_t ModbusNetwork_getBaudRate(uint32_t *xBaudRate);

    /**
     * @brief Retrieves the current network parameters (address, baudrate, parity).
     *
     * @param xSlaveAddress Pointer to store slave address.
     * @param xBaudRate Pointer to store baud rate.
     * @param xParity Pointer to store parity (e.g., 'N', 'E', 'O').
     * @return SJC_RET_OK if successful.
     */
    eSjcReturnType_t ModbusNetwork_getParameters(uint16_t *xSlaveAddress, uint32_t *xBaudRate, char *xParity);

    /**
     * @brief Performs a Modbus poll cycle (should be called cyclically).
     */
    void ModbusNetwork_update(void);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__MODBUS_NETWORK_H */
