#include "sjc_interface.hpp"

#ifdef __cplusplus
extern "C"
{
#include "errno.h"
#include "unistd.h"
#include "base/include/network_configs.h"
}
#endif

#include <iostream>
#include <endian.h>

SjcInterface::SjcInterface(const std::string port, const int address, const int baudrate)
{
    this->address_ = address;
    this->port_ = port;
    this->baudrate_ = baudrate;
}

SjcInterface::~SjcInterface(void)
{
    this->release();
}

eSjcReturnType_t SjcInterface::connect(void)
{
    if (SJC_INITIATED == this->init())
    {
        if (modbus_connect(ctx) == -1)
        {
            fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
            this->release();
            return SJC_CONNECTION_RELEASED;
        }
        else
        {
            modbus_set_slave(ctx, this->address_);
            return SJC_CONNECTED;
        }
    }
    else
    {
        return SJC_DISCONNECTED;
    }
}

eSjcReturnType_t SjcInterface::disconnect(void)
{
    modbus_close(ctx);
    return SJC_DISCONNECTED;
}

eSjcReturnType_t SjcInterface::getDeviceInfo(void)
{
    const int nb = 1 - REG_INPUT_HW_VERSION + REG_INPUT_FW_VERSION;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(REG_INPUT_START + REG_INPUT_HW_VERSION, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else 
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::getRuntimeInfo(void)
{
    const int nb = 1 - REG_INPUT_SYSTEM_UPTIME + REG_INPUT_SYSTEM_ERROR_FLAGS;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(REG_INPUT_START + REG_INPUT_SYSTEM_UPTIME, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else 
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::getNetworkParameters(void)
{
    const int nb = 1 - REG_INPUT_NETWORK_ADDRESS + REG_INPUT_NETWORK_PARITY;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(REG_INPUT_START + REG_INPUT_NETWORK_ADDRESS, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else 
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::setNetworkParameters(void)
{
    return eSjcReturnType_t();
}

eSjcReturnType_t SjcInterface::getActuatorParameters(void)
{
    const int nb = 1 - REG_INPUT_ACTUATOR_STATE + REG_INPUT_ACTUATOR_LIMITS_OUTPUT_SPEED;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(REG_INPUT_START + REG_INPUT_ACTUATOR_STATE, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else 
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::setActuatorParameters(void)
{
    return eSjcReturnType_t();
}

eSjcReturnType_t SjcInterface::getControllerParameters(void)
{
    const int nb = 1 - REG_INPUT_CONTROLLER_PARAMS_KP + REG_INPUT_CONTROLLER_PARAMS_RATE;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(REG_INPUT_START + REG_INPUT_CONTROLLER_PARAMS_KP, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else 
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::setControllerParameters(void)
{
    return eSjcReturnType_t();
}

eSjcReturnType_t SjcInterface::getSensorsParameters(void)
{
    const int nb = 1 - REG_INPUT_ACTUATOR_ENCODER_RESOLUTION_PPR + REG_INPUT_AUX_B_ENCODER_RESOLUTION_PPR;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(REG_INPUT_START + REG_INPUT_ACTUATOR_ENCODER_RESOLUTION_PPR, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else 
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::setSensorsParameters(void)
{
    return eSjcReturnType_t();
}

eSjcReturnType_t SjcInterface::setDigitalOutputsStates(void)
{
    return eSjcReturnType_t();
}

eSjcReturnType_t SjcInterface::init(void)
{
    ctx = modbus_new_rtu(this->port_.c_str(), baudrate_, 'N', 8, 1);

    if (ctx == NULL)
    {
        fprintf(stderr, "Unable to create the libmodbus context\n");
        this->release();
        return SJC_CONNECTION_RELEASED;
    }
    else
    {
        printf("\r\nDevice Initiated!");
        return SJC_INITIATED;
    }
}

eSjcReturnType_t SjcInterface::release(void)
{
    // modbus_free(ctx);
    return SJC_CONNECTION_RELEASED;
}

eSjcReturnType_t SjcInterface::getInputRegisters(const int initial_reg, const int num_of_registers, uint16_t *reg_table)
{
    int rc;
            
    try
    {
        rc = modbus_read_input_registers(ctx, initial_reg, num_of_registers, reg_table);
    }
    catch (const std::exception &e)
    {
        modbus_flush(ctx);
        std::cerr << e.what() << '\n';
    }

    if (rc == -1)
    {
        fprintf(stderr, "Input register reading failed: %s\n", modbus_strerror(errno));

        modbus_flush(ctx);

        return SJC_REGISTER_READING_FAIL;
    }
    else
    {
        return SJC_REGISTER_READING_PASS;
    }
}

eSjcReturnType_t SjcInterface::getHoldingRegisters(const int initial_reg, const int num_of_registers, uint16_t *reg_table)
{
    int rc;
            
    try
    {
        rc = modbus_read_registers(ctx, initial_reg, num_of_registers, reg_table);
    }
    catch (const std::exception &e)
    {
        modbus_flush(ctx);
        std::cerr << e.what() << '\n';
    }

    if (rc == -1)
    {
        fprintf(stderr, "Holding register reading failed: %s\n", modbus_strerror(errno));

        modbus_flush(ctx);

        return SJC_REGISTER_READING_FAIL;
    }
    else
    {
        return SJC_REGISTER_READING_PASS;
    }
}

eSjcReturnType_t SjcInterface::setHoldingRegisters(const int initial_reg, const int num_of_registers, uint16_t *reg_table)
{
    int rc;
            
    try
    {
        rc = modbus_write_registers(ctx, initial_reg, num_of_registers, reg_table);
    }
    catch (const std::exception &e)
    {
        modbus_flush(ctx);
        std::cerr << e.what() << '\n';
    }

    if (rc == -1)
    {
        fprintf(stderr, "Holding register writing failed: %s\n", modbus_strerror(errno));

        modbus_flush(ctx);

        return SJC_REGISTER_WRITING_FAIL;
    }
    else
    {
        return SJC_REGISTER_WRITING_PASS;
    }
}