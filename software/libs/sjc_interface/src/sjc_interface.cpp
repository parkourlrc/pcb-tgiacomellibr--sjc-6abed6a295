#include "sjc_interface.hpp"

#ifdef __cplusplus
extern "C"
{
#include "errno.h"
#include "unistd.h"
#include "network_configs.h"
#include "helpers.h"
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

eSjcReturnType_t SjcInterface::getDeviceInfo(xSjcDeviceInfo_t * info)
{
    const int nb = 3;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(REG_INPUT_START + REG_INPUT_HW_VERSION, nb, tab_reg))
    {
        info->firmware_ver.major = tab_reg[REG_INPUT_FW_VERSION - DEVICE_INFO_OFFSET] >> 8;                
        info->firmware_ver.minor = tab_reg[REG_INPUT_FW_VERSION - DEVICE_INFO_OFFSET+1] & 0x00ff;                
        info->firmware_ver.patch = tab_reg[REG_INPUT_FW_VERSION+1 - DEVICE_INFO_OFFSET];
        
        info->hardware_ver.major = tab_reg[REG_INPUT_HW_VERSION - DEVICE_INFO_OFFSET] >> 8;                
        info->hardware_ver.minor = tab_reg[REG_INPUT_HW_VERSION - DEVICE_INFO_OFFSET] & 0x00ff;                

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::getRuntimeInfo(xSjcRuntimeInfo_t * info)
{
    const int nb = 19;
    uint16_t tab_reg[nb];    

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(REG_INPUT_START + REG_INPUT_SYSTEM_UPTIME, nb, tab_reg))
    {
        compose_ui32_from_byte_array(&tab_reg[REG_INPUT_SYSTEM_UPTIME - DEVICE_RUNTIME_OFFSET], &info->uptime);

        info->controller_mode = tab_reg[REG_INPUT_CONTROL_MODE - DEVICE_RUNTIME_OFFSET];
        compose_float_from_byte_array(&tab_reg[REG_INPUT_CONTROL_ERROR - DEVICE_RUNTIME_OFFSET], &info->controller_error);
        compose_float_from_byte_array(&tab_reg[REG_INPUT_CONTROL_MODE - DEVICE_RUNTIME_OFFSET], &info->controller_ref);

        compose_float_from_byte_array(&tab_reg[REG_INPUT_ACTUATOR_DISPLACEMENT - DEVICE_RUNTIME_OFFSET], &info->actuator_displacement);
        compose_float_from_byte_array(&tab_reg[REG_INPUT_AUX_ENCODER_A_DISPLACEMENT - DEVICE_RUNTIME_OFFSET], &info->aux_A_displacement);
        compose_float_from_byte_array(&tab_reg[REG_INPUT_AUX_ENCODER_B_DISPLACEMENT - DEVICE_RUNTIME_OFFSET], &info->aux_B_displacement);

        info->actuator_displacement = tab_reg[REG_INPUT_ACTUATOR_OUTPUT_VOLTAGE - DEVICE_RUNTIME_OFFSET];
        info->aux_A_displacement = tab_reg[REG_INPUT_ACTUATOR_CURRENT - DEVICE_RUNTIME_OFFSET];

        info->digital_inputs = tab_reg[REG_INPUT_DIGITAL_INPUTS - DEVICE_RUNTIME_OFFSET];
        info->digital_outputs = tab_reg[REG_INPUT_DIGITAL_OUTPUTS - DEVICE_RUNTIME_OFFSET];

        info->system_voltage = tab_reg[REG_INPUT_SYSTEM_VOLTAGE - DEVICE_RUNTIME_OFFSET];
        
        info->error_flags = tab_reg[REG_INPUT_SYSTEM_ERROR_FLAGS - DEVICE_RUNTIME_OFFSET];        

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::getNetworkParameters(xSjcNetworkInfo_t * info)
{
    const int nb = 4;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(REG_INPUT_START + REG_INPUT_NETWORK_ADDRESS, nb, tab_reg))
    {
        info->address = tab_reg[REG_INPUT_NETWORK_ADDRESS - NETWORK_INFO_OFFSET];
        info->parity = tab_reg[REG_INPUT_NETWORK_PARITY - NETWORK_INFO_OFFSET];
        // compose_ui32_from_byte_array(&tab_reg[REG_INPUT_NETWORK_BAUD_RATE - NETWORK_INFO_OFFSET], &info->baudrate);        
        info->baudrate = tab_reg[REG_INPUT_NETWORK_BAUD_RATE - NETWORK_INFO_OFFSET] << 16 | tab_reg[REG_INPUT_NETWORK_BAUD_RATE - NETWORK_INFO_OFFSET + 1];
        
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