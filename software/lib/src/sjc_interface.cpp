#include "sjc_interface.hpp"

#ifdef __cplusplus
extern "C"
{
    #include "errno.h"
    #include "data_masks.hpp"
    #include <endian.h>
    #include "unistd.h"
}
#endif

#include <iostream>

#include "helpers.hpp"
#include "network_configs.hpp"

#define STD_WAIT_TIME_US 5UL

SjcInterface::SjcInterface(const std::string port, const int baudrate)
{
    this->address_ = 247;
    this->port_ = port;
    this->baudrate_ = baudrate;

    std::cout << "[SJC] Network config: " << std::endl;
    std::cout << "[SJC] - port: " << port_.c_str() << std::endl;
    std::cout << "[SJC] - baudrate: " << baudrate_ << std::endl;
    std::cout << "[SJC] - address: " << address_ << std::endl;
}

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
            modbus_rtu_set_serial_mode(ctx, MODBUS_RTU_RS485);
            modbus_set_response_timeout(ctx, 0, 20000);
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

eSjcReturnType_t SjcInterface::getSlaveAddress(int *addr)
{
    int ret = modbus_get_slave(ctx);

    if (ret == -1)
    {
        return SJC_SLAVE_ADDRESS_NOT_ASSIGNED;
    }
    else
    {
        *addr = ret;
    }

    return SJC_RET_OK;
}

eSjcReturnType_t SjcInterface::setSlaveAddress(const int address)
{
    int ret = modbus_set_slave(ctx, address);

    if (ret == -1)
    {
        return SJC_SLAVE_ADDRESS_NOT_ASSIGNED;
    }
    else
    {
        this->address_ = address;
    }

    return SJC_RET_OK;
}

eSjcReturnType_t SjcInterface::reboot(void)
{
    const int nb = 1;
    uint16_t tab_reg[nb];

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_SYSTEM_COMMANDS, MAP_HOLDING_REG_SYSTEM_COMMANDS_OFFSET)] = SYS_CMD_REBOOT;

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(HOLDING_REG_SYSTEM_COMMANDS, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::resetAuxEncoders(void)
{
    const int nb = 1;
    uint16_t tab_reg[nb];

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_SYSTEM_COMMANDS, MAP_HOLDING_REG_SYSTEM_COMMANDS_OFFSET)] = SYS_CMD_RESET_AUX_ENCODER_COUNTER;

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(HOLDING_REG_SYSTEM_COMMANDS, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::resetJointEncoder(void)
{
    const int nb = 1;
    uint16_t tab_reg[nb];

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_SYSTEM_COMMANDS, MAP_HOLDING_REG_SYSTEM_COMMANDS_OFFSET)] = SYS_CMD_RESET_PRIMARY_ENCODER_COUNTER;

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(HOLDING_REG_SYSTEM_COMMANDS, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::resetAuxVelocity(void)
{
    const int nb = 1;
    uint16_t tab_reg[nb];

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_SYSTEM_COMMANDS, MAP_HOLDING_REG_SYSTEM_COMMANDS_OFFSET)] = SYS_CMD_RESET_AUX_JOINT_VELOCITY;

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(HOLDING_REG_SYSTEM_COMMANDS, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::resetJointVelocity(void)
{
    const int nb = 1;
    uint16_t tab_reg[nb];

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_SYSTEM_COMMANDS, MAP_HOLDING_REG_SYSTEM_COMMANDS_OFFSET)] = SYS_CMD_RESET_PRIMARY_JOINT_VELOCITY;

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(HOLDING_REG_SYSTEM_COMMANDS, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::getBoardInfo(xSjcDeviceInfo_t *config)
{
    const int nb = 9;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(INPUT_REG_HW_VERSION, nb, tab_reg))
    {        
        config->hardware_ver.major = tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_HW_VERSION, MAP_INPUT_REG_BOARD_INFO_OFFSET)] >> 8;
        config->hardware_ver.minor = tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_HW_VERSION, MAP_INPUT_REG_BOARD_INFO_OFFSET)] & 0x00ff;
        
        config->firmware_ver.major = tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_FW_VERSION, MAP_INPUT_REG_BOARD_INFO_OFFSET)] >> 8;
        config->firmware_ver.minor = tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_FW_VERSION, MAP_INPUT_REG_BOARD_INFO_OFFSET)] & 0x00ff;
        config->firmware_ver.patch = tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_FW_VERSION + 1, MAP_INPUT_REG_BOARD_INFO_OFFSET)];

        config->net_address = tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_NETWORK_ADDRESS, MAP_INPUT_REG_BOARD_INFO_OFFSET)];
        config->net_parity = tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_NETWORK_PARITY, MAP_INPUT_REG_BOARD_INFO_OFFSET)];
        config->net_baudrate = tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_NETWORK_BAUD_RATE, MAP_INPUT_REG_BOARD_INFO_OFFSET)] << 16;
        config->net_baudrate |= tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_NETWORK_BAUD_RATE + 1, MAP_INPUT_REG_BOARD_INFO_OFFSET)];
        
        config->controller_mode = tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_CONTROL_MODE, MAP_INPUT_REG_BOARD_INFO_OFFSET)];
        config->error_flags = tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_SYSTEM_ERROR_FLAGS, MAP_INPUT_REG_BOARD_INFO_OFFSET)];

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::getControlMeasInfo(xSjcControlMeasInfo_t *config)
{
    const int nb = 22;
    uint16_t tab_reg[nb];         

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(INPUT_REG_SYSTEM_UPTIME, nb, tab_reg))
    {
        compose_ui32_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_SYSTEM_UPTIME, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)], &config->uptime);

        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_CONTROL_ERROR, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)], &config->controller_error);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_CONTROL_REFERENCE, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)], &config->controller_ref);
        
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_CURRENT, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)], &config->actuator_current);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_VOLTAGE, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)], &config->actuator_voltage);

        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_POSITION, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)], &config->actuator_angular_position);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_AUX_SWING_POSITION_THX, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)], &config->swing_angular_position_thx);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_AUX_SWING_POSITION_THY, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)], &config->swing_angular_position_thy);

        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_VELOCITY, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)], &config->actuator_angular_velocity);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_SWING_VELOCITY_THX, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)], &config->swing_angular_velocity_thx);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_SWING_VELOCITY_THY, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)], &config->swing_angular_velocity_thy);      

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::setControllerReference(const float reference)
{
    const int nb = 3;
    uint16_t tab_reg[nb];

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_CONTROLLER_REFERENCE_UPDATE, MAP_HOLDING_REG_CONTROLLER_REFERENCE_UPDATE_OFFSET)] = HOLDING_REG_APPLY_MSK;
    compose_byte_array_from_float(reference, &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_CONTROLLER_REFERENCE_VALUE, MAP_HOLDING_REG_CONTROLLER_REFERENCE_UPDATE_OFFSET)]);

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(HOLDING_REG_CONTROLLER_REFERENCE_VALUE, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::setControllerOperatingMode(const eSjcControlMode_t mode)
{
    const int nb = 1;
    uint16_t tab_reg[nb];

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_CONTROLLER_MODE_UPDATE, MAP_HOLDING_REG_CONTROLLER_MODE_UPDATE_OFFSET)] = mode | HOLDING_REG_APPLY_MSK;

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(HOLDING_REG_CONTROLLER_MODE_UPDATE, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::getDigitalIOInfo(xSjcDigitalIOInfo_t *config)    
{
    const int nb = 2;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(INPUT_REG_NETWORK_ADDRESS, nb, tab_reg))
    {
        uint16_t digital_inputs = tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_DIGITAL_INPUTS, MAP_INPUT_REG_DIGITAL_IO_INFO_OFFSET)];
        uint16_t digital_outputs = tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_DIGITAL_OUTPUTS, MAP_INPUT_REG_DIGITAL_IO_INFO_OFFSET)];        

        config->inputs.switch_a = digital_inputs & (1 << SWITCH_A_MASK);
        config->inputs.switch_b = digital_inputs & (1 << SWITCH_B_MASK);
        config->inputs.iso_input_a = digital_inputs & (1 << ISO_INPUT_A_MASK);
        config->inputs.iso_input_b = digital_inputs & (1 << ISO_INPUT_B_MASK);
        config->outputs.output_a = digital_outputs & (1 << OUTPUT_A_MASK);
        config->outputs.output_b = digital_outputs & (1 << OUTPUT_B_MASK);

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::setNetworkParameters(const uint16_t address, const char parity, const uint32_t baudrate)
{
    const int nb = 5;
    uint16_t tab_reg[nb];

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_NETWORK_CONFIG_ADDRESS, MAP_HOLDING_REG_NETWORK_PARAMETERS_UPDATE_OFFSET)] = address;
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_NETWORK_CONFIG_PARITY, MAP_HOLDING_REG_NETWORK_PARAMETERS_UPDATE_OFFSET)] = parity;
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_NETWORK_CONFIG_BAUD_RATE, MAP_HOLDING_REG_NETWORK_PARAMETERS_UPDATE_OFFSET)] = baudrate >> 16;
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_NETWORK_CONFIG_BAUD_RATE + 1, MAP_HOLDING_REG_NETWORK_PARAMETERS_UPDATE_OFFSET)] = baudrate & 0x00ff;
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_NETWORK_CONFIG_UPDATE, MAP_HOLDING_REG_NETWORK_PARAMETERS_UPDATE_OFFSET)] = HOLDING_REG_APPLY_MSK;

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(HOLDING_REG_NETWORK_CONFIG_ADDRESS, nb, tab_reg))
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
    const int nb = 11;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(INPUT_REG_ACTUATOR_STATE, nb, tab_reg))
    {
        config->state = tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_STATE, MAP_INPUT_REG_ACTUATOR_INFO_OFFSET)];
        
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_SYSTEM_VOLTAGE, MAP_INPUT_REG_ACTUATOR_INFO_OFFSET)], &config->system_voltage);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_BASE_VOLTAGE, MAP_INPUT_REG_ACTUATOR_INFO_OFFSET)], &config->base_voltage);        

        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_LIMITS_CURRENT, MAP_INPUT_REG_ACTUATOR_INFO_OFFSET)], &config->current_limit);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_LIMITS_VOLTAGE, MAP_INPUT_REG_ACTUATOR_INFO_OFFSET)], &config->voltage_limit);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_LIMITS_SPEED, MAP_INPUT_REG_ACTUATOR_INFO_OFFSET)], &config->speed_limit);

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

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_ACTUATOR_SETUP_UPDATE, MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET)] = HOLDING_REG_APPLY_MSK;
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_ACTUATOR_BASE_VOLTAGE, MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET)] = (config.base_voltage * 100);
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_ACTUATOR_LIMITS_CURRENT, MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET)] = (config.current_limit * 100);
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_ACTUATOR_LIMITS_VOLTAGE, MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET)] = (config.voltage_limit * 100);
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_ACTUATOR_RESERVED, MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET)] = 1UL;
    compose_byte_array_from_float(config.speed_limit, &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_ACTUATOR_LIMITS_SPEED, MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET)]);

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(HOLDING_REG_ACTUATOR_BASE_VOLTAGE, nb, tab_reg))
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

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(INPUT_REG_CONTROLLER_PARAMS_KP, nb, tab_reg))
    {
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_CONTROLLER_PARAMS_KP, MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET)], &config->Kp);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_CONTROLLER_PARAMS_KI, MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET)], &config->Ki);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_CONTROLLER_PARAMS_KD, MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET)], &config->Kd);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_CONTROLLER_PARAMS_KT, MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET)], &config->Kt);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_CONTROLLER_PARAMS_KA, MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET)], &config->Ka);
        compose_float_from_byte_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_CONTROLLER_PARAMS_RATE, MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET)], &config->update_rate);

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

    compose_byte_array_from_float(config.Kp, &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_CONTROLLER_PARAMS_KP, MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_byte_array_from_float(config.Ki, &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_CONTROLLER_PARAMS_KI, MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_byte_array_from_float(config.Kd, &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_CONTROLLER_PARAMS_KD, MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_byte_array_from_float(config.Kt, &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_CONTROLLER_PARAMS_KT, MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_byte_array_from_float(config.Ka, &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_CONTROLLER_PARAMS_KA, MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_byte_array_from_float(config.update_rate, &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_CONTROLLER_PARAMS_RATE, MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_CONTROLLER_PARAMS_UPDATE, MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET)] = HOLDING_REG_APPLY_MSK;

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(HOLDING_REG_CONTROLLER_PARAMS_KP, nb, tab_reg))
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

    if (SJC_REGISTER_READING_PASS == this->getInputRegisters(INPUT_REG_ACTUATOR_ENCODER_RESOLUTION_PPR, nb, tab_reg))
    {
        config->aux_sensor_resolution_a = tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_SWING_THX_ENCODER_RESOLUTION_PPR, MAP_INPUT_REG_SENSORS_INFO_OFFSET)];
        config->aux_sensor_resolution_b = tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_SWING_THY_ENCODER_RESOLUTION_PPR, MAP_INPUT_REG_SENSORS_INFO_OFFSET)];
        config->actuator_sensor_resolution = tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_ENCODER_RESOLUTION_PPR, MAP_INPUT_REG_SENSORS_INFO_OFFSET)];

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

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_SENSORS_SETUP_UPDATE, MAP_HOLDING_REG_SENSORS_PARAMETERS_UPDATE_OFFSET)] = HOLDING_REG_APPLY_MSK;
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_SWING_THX_ENCODER_RESOLUTION_PPR, MAP_HOLDING_REG_SENSORS_PARAMETERS_UPDATE_OFFSET)] = config.aux_sensor_resolution_a;
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_SWING_THY_ENCODER_RESOLUTION_PPR, MAP_HOLDING_REG_SENSORS_PARAMETERS_UPDATE_OFFSET)] = config.aux_sensor_resolution_b;
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_ACTUATOR_ENCODER_RESOLUTION_PPR, MAP_HOLDING_REG_SENSORS_PARAMETERS_UPDATE_OFFSET)] = config.actuator_sensor_resolution;

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(HOLDING_REG_ACTUATOR_ENCODER_RESOLUTION_PPR, nb, tab_reg))
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

    uint16_t temp = HOLDING_REG_APPLY_MSK;
    temp |= (config.output_a << OUTPUT_A_MASK);
    temp |= (config.output_b << OUTPUT_B_MASK);
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_DIGITAL_OUTPUTS_STATE_UPDATE, MAP_HOLDING_REG_DIGITAL_OUTPUTS_OFFSET)] = temp |= HOLDING_REG_APPLY_MSK;

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(HOLDING_REG_DIGITAL_OUTPUTS_STATE_UPDATE, nb, tab_reg))
    {

        return SJC_RET_OK;
    }
    else
    {
        return SJC_RET_NOK;
    }
}

eSjcReturnType_t SjcInterface::setAdcFilterMode(const eSjcAdcFilterMode_t mode, const float cutoff_frequency)
{
    const int nb = 4;
    uint16_t tab_reg[nb];

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_ADC_FILTER_CONFIG_MODE, MAP_HOLDING_REG_ADC_FILTER_CONFIG_UPDATE_OFFSET)] = mode;
    compose_byte_array_from_float(cutoff_frequency, &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_ADC_FILTER_CONFIG_CUTOFF_FREQUENCY, MAP_HOLDING_REG_ADC_FILTER_CONFIG_UPDATE_OFFSET)]);
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_ADC_FILTER_CONFIG_UPDATE, MAP_HOLDING_REG_ADC_FILTER_CONFIG_UPDATE_OFFSET)] = HOLDING_REG_APPLY_MSK;

    if (SJC_REGISTER_WRITING_PASS == this->setHoldingRegisters(HOLDING_REG_ADC_FILTER_CONFIG_MODE, nb, tab_reg))
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
    modbus_free(ctx);
    ctx = nullptr;
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
    waitTime_us(STD_WAIT_TIME_US);
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

    waitTime_us(STD_WAIT_TIME_US);
}

eSjcReturnType_t SjcInterface::waitTime_us(const unsigned int wait_time_us)
{
    usleep(wait_time_us);

    return SJC_RET_OK;
}
