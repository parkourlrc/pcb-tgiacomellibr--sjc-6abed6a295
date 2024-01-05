#include "sjc_interface.hpp"

#ifdef __cplusplus
extern "C"
{
#include "errno.h"
#include "unistd.h"
#include "helpers.h"
#include "data_masks.h"
#include "network_configs.h"
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

eSjcReturnType_t SjcInterface::getDeviceInfo(xSjcDeviceInfo_t *config)
{
    const int nb = 3;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(REG_INPUT_HW_VERSION, nb, tab_reg))
    {
        config->firmware_ver.major = tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_FW_VERSION, DEVICE_INFO_OFFSET)] >> 8;
        config->firmware_ver.minor = tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_FW_VERSION, DEVICE_INFO_OFFSET)] & 0x00ff;
        config->firmware_ver.patch = tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_FW_VERSION + 1, DEVICE_INFO_OFFSET)];

        config->hardware_ver.major = tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_HW_VERSION, DEVICE_INFO_OFFSET)] >> 8;
        config->hardware_ver.minor = tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_HW_VERSION, DEVICE_INFO_OFFSET)] & 0x00ff;

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::getRuntimeInfo(xSjcRuntimeInfo_t *config)
{
    const int nb = 21;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(REG_INPUT_SYSTEM_UPTIME, nb, tab_reg))
    {
        compose_ui32_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_SYSTEM_UPTIME, RUNTIME_INFO_OFFSET)], &config->uptime);
        
        config->controller_mode = tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_CONTROL_MODE, RUNTIME_INFO_OFFSET)];
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_CONTROL_ERROR, RUNTIME_INFO_OFFSET)], &config->controller_error);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_CONTROL_REFERENCE, RUNTIME_INFO_OFFSET)], &config->controller_ref);

        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_SYSTEM_VOLTAGE, RUNTIME_INFO_OFFSET)], &config->system_voltage);        
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_ACTUATOR_CURRENT, RUNTIME_INFO_OFFSET)], &config->actuator_current);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_ACTUATOR_VOLTAGE, RUNTIME_INFO_OFFSET)], &config->actuator_voltage);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_ACTUATOR_DISPLACEMENT, RUNTIME_INFO_OFFSET)], &config->actuator_displacement);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_AUX_ENCODER_A_DISPLACEMENT, RUNTIME_INFO_OFFSET)], &config->aux_A_displacement);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_AUX_ENCODER_B_DISPLACEMENT, RUNTIME_INFO_OFFSET)], &config->aux_B_displacement);

        uint16_t digital_inputs = tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_DIGITAL_INPUTS, RUNTIME_INFO_OFFSET)];
        uint16_t digital_outputs = tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_DIGITAL_OUTPUTS, RUNTIME_INFO_OFFSET)];
        config->inputs.switchA = digital_inputs & (1 << switchA_msk);
        config->inputs.switchB = digital_inputs & (1 << switchB_msk);
        config->inputs.iso_inputA = digital_inputs & (1 << iso_inputA_msk);
        config->inputs.iso_inputB = digital_inputs & (1 << iso_inputB_msk);
        config->outputs.outputA = digital_outputs & (1 << outputA_msk);
        config->outputs.outputB = digital_outputs & (1 << outputB_msk);

        config->error_flags = tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_SYSTEM_ERROR_FLAGS, RUNTIME_INFO_OFFSET)];

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::setControllerConfig(xSjcControllerConfig_t config)
{
    const int nb = 3;
    uint16_t tab_reg[nb];
    
    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_CONTROLLER_MODE_UPDATE, CONTROLLER_UPDATE_OFFSET)] = config.controller_mode_update | CONTROLLER_UPDATE_MSK;    
    compose_byte_array_from_float(config.controller_ref, &tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_CONTROLLER_REFERENCE, CONTROLLER_UPDATE_OFFSET)]);

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(REG_HOLDING_CONTROLLER_REFERENCE, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::getNetworkParameters(xSjcNetworkConfig_t *config)
{
    const int nb = 4;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(REG_INPUT_NETWORK_ADDRESS, nb, tab_reg))
    {
        config->address = tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_NETWORK_ADDRESS, NETWORK_INFO_OFFSET)];
        config->parity = tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_NETWORK_PARITY, NETWORK_INFO_OFFSET)];
        config->baudrate = tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_NETWORK_BAUD_RATE, NETWORK_INFO_OFFSET)] << 16 | tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_NETWORK_BAUD_RATE + 1, NETWORK_INFO_OFFSET)];

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::setNetworkParameters(const xSjcNetworkConfig_t config)
{
    const int nb = 5;
    uint16_t tab_reg[nb];

    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_NETWORK_CONFIG_ADDRESS, NETWORK_PARAMETERS_UPDATE_OFFSET)] = config.address;
    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_NETWORK_CONFIG_PARITY, NETWORK_PARAMETERS_UPDATE_OFFSET)] = config.parity;
    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_NETWORK_CONFIG_BAUD_RATE, NETWORK_PARAMETERS_UPDATE_OFFSET)] = config.baudrate >> 16;
    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_NETWORK_CONFIG_BAUD_RATE + 1, NETWORK_PARAMETERS_UPDATE_OFFSET)] = config.baudrate & 0x00ff;
    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_NETWORK_CONFIG_UPDATE, NETWORK_PARAMETERS_UPDATE_OFFSET)] = NETWORK_PARAMETERS_UPDATE_MSK;

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(REG_HOLDING_NETWORK_CONFIG_ADDRESS, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::getActuatorParameters(xSjcActuatorConfig_t *config)
{
    const int nb = 6;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(REG_INPUT_ACTUATOR_STATE, nb, tab_reg))
    {
        config->state = tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_ACTUATOR_STATE, ACTUATOR_INFO_OFFSET)];
        config->base_voltage = (tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_ACTUATOR_BASE_VOLTAGE, ACTUATOR_INFO_OFFSET)] / 100);
        config->current_limit = (tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_ACTUATOR_LIMITS_CURRENT, ACTUATOR_INFO_OFFSET)] / 100);
        config->voltage_limit = (tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_ACTUATOR_LIMITS_VOLTAGE, ACTUATOR_INFO_OFFSET)] / 100);

        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_ACTUATOR_LIMITS_SPEED, ACTUATOR_INFO_OFFSET)], &config->speed_limit);

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::setActuatorParameters(const xSjcActuatorConfig_t config)
{
    const int nb = 7;
    uint16_t tab_reg[nb];

    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_ACTUATOR_SETUP_UPDATE, ACTUATOR_CONFIGS_UPDATE_OFFSET)] = ACTUATOR_CONFIGS_UPDATE_MSK;
    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_ACTUATOR_BASE_VOLTAGE, ACTUATOR_CONFIGS_UPDATE_OFFSET)] = (config.base_voltage * 100);
    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_ACTUATOR_LIMITS_CURRENT, ACTUATOR_CONFIGS_UPDATE_OFFSET)] = (config.current_limit * 100);
    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_ACTUATOR_LIMITS_VOLTAGE, ACTUATOR_CONFIGS_UPDATE_OFFSET)] = (config.voltage_limit * 100);
    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_ACTUATOR_RESERVED, ACTUATOR_CONFIGS_UPDATE_OFFSET)] = 1UL;
    compose_byte_array_from_float(config.speed_limit, &tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_ACTUATOR_LIMITS_SPEED, ACTUATOR_CONFIGS_UPDATE_OFFSET)]);

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(REG_HOLDING_ACTUATOR_BASE_VOLTAGE, nb, tab_reg))
    {
        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::getControllerParameters(xSjcControllerParameters_t *config)
{
    const int nb = 12;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(REG_INPUT_CONTROLLER_PARAMS_KP, nb, tab_reg))
    {
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_CONTROLLER_PARAMS_KP, CONTROLLER_INFO_OFFSET)], &config->kp_parameter);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_CONTROLLER_PARAMS_KI, CONTROLLER_INFO_OFFSET)], &config->ki_parameter);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_CONTROLLER_PARAMS_KD, CONTROLLER_INFO_OFFSET)], &config->kd_parameter);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_CONTROLLER_PARAMS_AUX1, CONTROLLER_INFO_OFFSET)], &config->aux1_parameter);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_CONTROLLER_PARAMS_AUX2, CONTROLLER_INFO_OFFSET)], &config->aux2_parameter);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_CONTROLLER_PARAMS_RATE, CONTROLLER_INFO_OFFSET)], &config->rate_parameter);

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::setControllerParameters(const xSjcControllerParameters_t config)
{
    const int nb = 13;
    uint16_t tab_reg[nb];

    compose_byte_array_from_float(config.kp_parameter, &tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_CONTROLLER_PARAMS_KP, CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_byte_array_from_float(config.ki_parameter, &tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_CONTROLLER_PARAMS_KI, CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_byte_array_from_float(config.kd_parameter, &tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_CONTROLLER_PARAMS_KD, CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_byte_array_from_float(config.aux1_parameter, &tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_CONTROLLER_PARAMS_AUX1, CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_byte_array_from_float(config.aux2_parameter, &tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_CONTROLLER_PARAMS_AUX2, CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_byte_array_from_float(config.rate_parameter, &tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_CONTROLLER_PARAMS_RATE, CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_CONTROLLER_PARAMS_UPDATE, CONTROLLER_PARAMETERS_UPDATE_OFFSET)] = CONTROLLER_PARAMETERS_UPDATE_MSK;

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(REG_HOLDING_CONTROLLER_PARAMS_KP, nb, tab_reg))
    {
        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::getSensorsParameters(xSjcSensorsConfig_t *config)
{
    const int nb = 3;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(REG_INPUT_ACTUATOR_ENCODER_RESOLUTION_PPR, nb, tab_reg))
    {
        config->auxA_sensor_res_ppr = tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_AUX_A_ENCODER_RESOLUTION_PPR, SENSORS_INFO_OFFSET)];
        config->auxB_sensor_res_ppr = tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_AUX_B_ENCODER_RESOLUTION_PPR, SENSORS_INFO_OFFSET)];
        config->actuator_sensor_res_ppr = tab_reg[GET_REG_TABLE_INDEX(REG_INPUT_ACTUATOR_ENCODER_RESOLUTION_PPR, SENSORS_INFO_OFFSET)];

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::setSensorsParameters(const xSjcSensorsConfig_t config)
{
    const int nb = 4;
    uint16_t tab_reg[nb];

    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_SENSORS_SETUP_UPDATE, SENSORS_PARAMETERS_UPDATE_OFFSET)] = SENSORS_PARAMETERS_UPDATE_MSK;
    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_AUX_ENCODER_A_RESOLUTION_PPR, SENSORS_PARAMETERS_UPDATE_OFFSET)] = config.auxA_sensor_res_ppr;
    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_AUX_ENCODER_B_RESOLUTION_PPR, SENSORS_PARAMETERS_UPDATE_OFFSET)] = config.auxB_sensor_res_ppr;
    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_ACTUATOR_ENCODER_RESOLUTION_PPR, SENSORS_PARAMETERS_UPDATE_OFFSET)] = config.actuator_sensor_res_ppr;

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(REG_HOLDING_ACTUATOR_ENCODER_RESOLUTION_PPR, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::setDigitalOutputsStates(const xSjcOutputsStates_t config)
{
    const int nb = 1;
    uint16_t tab_reg[nb];

    uint16_t temp = DIGITAL_OUTPUTS_UPDATE_MSK;
    temp |= (config.outputA << outputA_msk);
    temp |= (config.outputB << outputB_msk);
    tab_reg[GET_REG_TABLE_INDEX(REG_HOLDING_DIGITAL_OUTPUTS_STATE_UPDATE, DIGITAL_OUTPUTS_OFFSET)] = SENSORS_PARAMETERS_UPDATE_MSK;

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(REG_HOLDING_DIGITAL_OUTPUTS_STATE_UPDATE, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
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
        // printf("\r\nDevice Initiated!");
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
        rc = modbus_read_input_registers(ctx, GET_INPUT_REG_ADDRESS(initial_reg), num_of_registers, reg_table);
    }
    catch (const std::exception &e)
    {
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

eSjcReturnType_t SjcInterface::setHoldingRegisters(const int initial_reg, const int num_of_registers, uint16_t *reg_table)
{
    int rc;

    try
    {
        rc = modbus_write_registers(ctx, GET_HOLDING_REG_ADDRESS(initial_reg), num_of_registers, reg_table);
    }
    catch (const std::exception &e)
    {
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