#include "sjc_interface.hpp"

#ifdef __cplusplus
extern "C"
{
#include <endian.h>
#include <errno.h>
#include <unistd.h>
}
#endif

#include <chrono>
#include <iostream>
#include <thread>

#define STD_WAIT_TIME_US 5UL

/* ============================================================
 *  Constructors / Destructor
 * ============================================================ */

SjcInterface::SjcInterface(const std::string &port, const int baudrate)
    : address_(247), baudrate_(baudrate), port_(port), ctx(nullptr)

{
    std::cout << "[SJC] Network config:\n"
              << "  - port: " << port_ << "\n"
              << "  - baudrate: " << baudrate_ << "\n"
              << "  - address: " << address_ << std::endl;
}

SjcInterface::SjcInterface(const std::string &port, const int address, const int baudrate)
    : address_(address), baudrate_(baudrate), port_(port), ctx(nullptr)
{
    std::cout << "[SJC] Network config:\n"
              << "  - port: " << port_ << "\n"
              << "  - baudrate: " << baudrate_ << "\n"
              << "  - address: " << address_ << std::endl;
}

SjcInterface::~SjcInterface()
{
    this->release();
}

/* ============================================================
 *  Connection management
 * ============================================================ */

eSjcReturnType_t SjcInterface::connect()
{
    if (SJC_INITIATED == this->init())
    {
        if (modbus_connect(ctx) == -1)
        {
            std::cerr << "[SJC] Connection failed: " << modbus_strerror(errno) << std::endl;
            this->release();
            return SJC_CONNECTION_RELEASED;
        }

        modbus_set_slave(ctx, this->address_);
        modbus_rtu_set_serial_mode(ctx, MODBUS_RTU_RS485);
        modbus_set_response_timeout(ctx, 0, 10000);

        return SJC_CONNECTED;
    }
    return SJC_DISCONNECTED;
}

eSjcReturnType_t SjcInterface::disconnect()
{
    modbus_close(ctx);
    return SJC_DISCONNECTED;
}

eSjcReturnType_t SjcInterface::getSlaveAddress(int *addr)
{
    int ret = modbus_get_slave(ctx);
    if (ret == -1)
        return SJC_SLAVE_ADDRESS_NOT_ASSIGNED;

    *addr = ret;
    return SJC_RET_OK;
}

eSjcReturnType_t SjcInterface::setSlaveAddress(const int address)
{
    int ret = modbus_set_slave(ctx, address);
    if (ret == -1)
        return SJC_SLAVE_ADDRESS_NOT_ASSIGNED;

    this->address_ = address;
    return SJC_RET_OK;
}

/* ============================================================
 *  System commands
 * ============================================================ */

eSjcReturnType_t SjcInterface::reboot()
{
    const int nb = 1;
    uint16_t tab_reg[nb];
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_SYSTEM_COMMANDS, MAP_HOLDING_REG_SYSTEM_COMMANDS_OFFSET)] =
        SYS_CMD_REBOOT;
    return (SJC_REGISTER_WRITING_PASS ==
            this->setHoldingRegisters(HOLDING_REG_SYSTEM_COMMANDS, nb, tab_reg))
               ? SJC_RET_OK
               : SJC_RET_NOK;
}

eSjcReturnType_t SjcInterface::resetAuxEncoders()
{
    const int nb = 1;
    uint16_t tab_reg[nb];
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_SYSTEM_COMMANDS, MAP_HOLDING_REG_SYSTEM_COMMANDS_OFFSET)] =
        SYS_CMD_RESET_AUX_ENCODER_COUNTER;
    return (SJC_REGISTER_WRITING_PASS ==
            this->setHoldingRegisters(HOLDING_REG_SYSTEM_COMMANDS, nb, tab_reg))
               ? SJC_RET_OK
               : SJC_RET_NOK;
}

eSjcReturnType_t SjcInterface::resetJointEncoder()
{
    const int nb = 1;
    uint16_t tab_reg[nb];
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_SYSTEM_COMMANDS, MAP_HOLDING_REG_SYSTEM_COMMANDS_OFFSET)] =
        SYS_CMD_RESET_PRIMARY_ENCODER_COUNTER;
    return (SJC_REGISTER_WRITING_PASS ==
            this->setHoldingRegisters(HOLDING_REG_SYSTEM_COMMANDS, nb, tab_reg))
               ? SJC_RET_OK
               : SJC_RET_NOK;
}

eSjcReturnType_t SjcInterface::resetAuxVelocity()
{
    const int nb = 1;
    uint16_t tab_reg[nb];
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_SYSTEM_COMMANDS, MAP_HOLDING_REG_SYSTEM_COMMANDS_OFFSET)] =
        SYS_CMD_RESET_AUX_JOINT_VELOCITY;
    return (SJC_REGISTER_WRITING_PASS ==
            this->setHoldingRegisters(HOLDING_REG_SYSTEM_COMMANDS, nb, tab_reg))
               ? SJC_RET_OK
               : SJC_RET_NOK;
}

eSjcReturnType_t SjcInterface::resetJointVelocity()
{
    const int nb = 1;
    uint16_t tab_reg[nb];
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_SYSTEM_COMMANDS, MAP_HOLDING_REG_SYSTEM_COMMANDS_OFFSET)] =
        SYS_CMD_RESET_PRIMARY_JOINT_VELOCITY;
    return (SJC_REGISTER_WRITING_PASS ==
            this->setHoldingRegisters(HOLDING_REG_SYSTEM_COMMANDS, nb, tab_reg))
               ? SJC_RET_OK
               : SJC_RET_NOK;
}

/* ============================================================
 *  Get methods (input registers)
 * ============================================================ */

eSjcReturnType_t SjcInterface::getBoardInfo(xSjcBoardInfo_t *config)
{
    const int nb = 12;
    uint16_t tab_reg[nb];
    if (SJC_REGISTER_READING_PASS ==
        this->getInputRegisters(INPUT_REG_HW_VERSION, nb, tab_reg))
    {
        config->hardware_ver.major =
            tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_HW_VERSION, MAP_INPUT_REG_BOARD_INFO_OFFSET)] >> 8;
        config->hardware_ver.minor =
            tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_HW_VERSION, MAP_INPUT_REG_BOARD_INFO_OFFSET)] & 0xFF;

        config->firmware_ver.major =
            tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_FW_VERSION_MAJOR_MINOR, MAP_INPUT_REG_BOARD_INFO_OFFSET)] >> 8;
        config->firmware_ver.minor =
            tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_FW_VERSION_MAJOR_MINOR, MAP_INPUT_REG_BOARD_INFO_OFFSET)] & 0xFF;
        config->firmware_ver.patch =
            tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_FW_VERSION_PATCH, MAP_INPUT_REG_BOARD_INFO_OFFSET)];

        config->net_address =
            tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_NETWORK_ADDRESS, MAP_INPUT_REG_BOARD_INFO_OFFSET)];
        config->net_parity =
            tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_NETWORK_PARITY, MAP_INPUT_REG_BOARD_INFO_OFFSET)];
        compose_ui32_from_ui16_array(&tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_NETWORK_BAUD_RATE, MAP_INPUT_REG_BOARD_INFO_OFFSET)], &config->net_baudrate);

        config->control_mode =
            tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_CONTROL_MODE, MAP_INPUT_REG_BOARD_INFO_OFFSET)];
        config->error_flags =
            tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_SYSTEM_ERROR_FLAGS, MAP_INPUT_REG_BOARD_INFO_OFFSET)];

        return SJC_RET_OK;
    }
    return SJC_RET_NOK;
}

eSjcReturnType_t SjcInterface::getControlMeasInfo(xSjcControlMeasInfo_t *config)
{
    const int nb = 22;
    uint16_t tab_reg[nb];
    if (SJC_REGISTER_READING_PASS ==
        this->getInputRegisters(INPUT_REG_SYSTEM_UPTIME, nb, tab_reg))
    {
        compose_ui32_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_SYSTEM_UPTIME, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)],
            &config->uptime);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_CONTROL_ERROR, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)],
            &config->controller_error);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_CONTROL_REFERENCE, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)],
            &config->controller_ref);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_CURRENT, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)],
            &config->actuator_current);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_VOLTAGE, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)],
            &config->actuator_voltage);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_POSITION, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)],
            &config->actuator_angular_position);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_AUX_SWING_POSITION_THX, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)],
            &config->swing_angular_position_thx);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_AUX_SWING_POSITION_THY, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)],
            &config->swing_angular_position_thy);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_VELOCITY, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)],
            &config->actuator_angular_velocity);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_SWING_VELOCITY_THX, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)],
            &config->swing_angular_velocity_thx);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_SWING_VELOCITY_THY, MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET)],
            &config->swing_angular_velocity_thy);

        return SJC_RET_OK;
    }
    return SJC_RET_NOK;
}

/* ============================================================
 *  Digital IO
 * ============================================================ */

eSjcReturnType_t SjcInterface::getDigitalIOInfo(xSjcDigitalIOInfo_t *config)
{
    const int nb = 2;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS ==
        this->getInputRegisters(INPUT_REG_DIGITAL_INPUTS, nb, tab_reg))
    {
        uint16_t digital_inputs =
            tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_DIGITAL_INPUTS, MAP_INPUT_REG_DIGITAL_IO_INFO_OFFSET)];
        uint16_t digital_outputs =
            tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_DIGITAL_OUTPUTS, MAP_INPUT_REG_DIGITAL_IO_INFO_OFFSET)];

        config->inputs.switch_a = (digital_inputs >> SWITCH_A_MASK) & 0x1;
        config->inputs.switch_b = (digital_inputs >> SWITCH_B_MASK) & 0x1;
        config->inputs.iso_input_a = (digital_inputs >> ISO_INPUT_A_MASK) & 0x1;
        config->inputs.iso_input_b = (digital_inputs >> ISO_INPUT_B_MASK) & 0x1;

        config->outputs.output_a = (digital_outputs >> OUTPUT_A_MASK) & 0x1;
        config->outputs.output_b = (digital_outputs >> OUTPUT_B_MASK) & 0x1;

        return SJC_RET_OK;
    }
    return SJC_RET_NOK;
}

eSjcReturnType_t SjcInterface::setDigitalOutputsStates(const xSjcOutputsStates_t &config)
{
    const int nb = 1;
    uint16_t tab_reg[nb];

    uint16_t value = 0;
    value |= (config.output_a << OUTPUT_A_MASK);
    value |= (config.output_b << OUTPUT_B_MASK);
    value |= HOLDING_REG_APPLY_MSK;

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_DIGITAL_OUTPUTS_STATE_UPDATE,
                                MAP_HOLDING_REG_DIGITAL_OUTPUTS_OFFSET)] = value;

    return (SJC_REGISTER_WRITING_PASS ==
            this->setHoldingRegisters(HOLDING_REG_DIGITAL_OUTPUTS_STATE_UPDATE, nb, tab_reg))
               ? SJC_RET_OK
               : SJC_RET_NOK;
}

/* ============================================================
 *  Actuator
 * ============================================================ */

eSjcReturnType_t SjcInterface::getActuatorParameters(xSjcActuatorInfo_t *config)
{
    const int nb = 12;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS ==
        this->getInputRegisters(INPUT_REG_ACTUATOR_STATE, nb, tab_reg))
    {
        config->state = (tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_STATE, MAP_INPUT_REG_ACTUATOR_INFO_OFFSET)] & 0x01U) != 0U;

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_SYSTEM_VOLTAGE, MAP_INPUT_REG_ACTUATOR_INFO_OFFSET)],
            &config->system_voltage);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_BASE_VOLTAGE, MAP_INPUT_REG_ACTUATOR_INFO_OFFSET)],
            &config->base_voltage);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_LIMITS_CURRENT, MAP_INPUT_REG_ACTUATOR_INFO_OFFSET)],
            &config->current_limit);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_LIMITS_VOLTAGE, MAP_INPUT_REG_ACTUATOR_INFO_OFFSET)],
            &config->voltage_limit);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_LIMITS_SPEED, MAP_INPUT_REG_ACTUATOR_INFO_OFFSET)],
            &config->speed_limit);

        return SJC_RET_OK;
    }
    return SJC_RET_NOK;
}

eSjcReturnType_t SjcInterface::setActuatorParameters(const xSjcActuatorInfo_t &config)
{
    const int nb = 9;
    uint16_t tab_reg[nb];

    compose_ui16_array_from_float(config.base_voltage,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_ACTUATOR_BASE_VOLTAGE, MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.current_limit,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_ACTUATOR_LIMITS_CURRENT, MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.voltage_limit,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_ACTUATOR_LIMITS_VOLTAGE, MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.speed_limit,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_ACTUATOR_LIMITS_SPEED, MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET)]);

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_ACTUATOR_SETUP_UPDATE,
                                MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET)] = HOLDING_REG_APPLY_MSK;

    return (SJC_REGISTER_WRITING_PASS ==
            this->setHoldingRegisters(HOLDING_REG_ACTUATOR_BASE_VOLTAGE, nb, tab_reg))
               ? SJC_RET_OK
               : SJC_RET_NOK;
}

/* ============================================================
 *  PID Controller
 * ============================================================ */

eSjcReturnType_t SjcInterface::getPIDControllerParameters(xSjcPIDControllerParameters_t *config)
{
    const int nb = 12;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS ==
        this->getInputRegisters(INPUT_REG_PID_CONTROLLER_PARAMS_KP, nb, tab_reg))
    {
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_PID_CONTROLLER_PARAMS_KP, MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET)], &config->Kp);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_PID_CONTROLLER_PARAMS_KI, MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET)], &config->Ki);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_PID_CONTROLLER_PARAMS_KD, MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET)], &config->Kd);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_PID_CONTROLLER_PARAMS_K_AW, MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET)], &config->K_aw);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_PID_CONTROLLER_PARAMS_U_MIN, MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET)], &config->u_min);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_PID_CONTROLLER_PARAMS_U_MAX, MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET)], &config->u_max);

        return SJC_RET_OK;
    }
    return SJC_RET_NOK;
}

eSjcReturnType_t SjcInterface::setPIDControllerParameters(const xSjcPIDControllerParameters_t &config)
{
    const int nb = 13;
    uint16_t tab_reg[nb];

    compose_ui16_array_from_float(config.Kp,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_PID_CONTROLLER_PARAMS_KP, MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Ki,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_PID_CONTROLLER_PARAMS_KI, MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Kd,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_PID_CONTROLLER_PARAMS_KD, MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.K_aw,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_PID_CONTROLLER_PARAMS_K_AW, MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.u_min,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_PID_CONTROLLER_PARAMS_U_MIN, MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.u_max,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_PID_CONTROLLER_PARAMS_U_MAX, MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_PID_CONTROLLER_UPDATE,
                                MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET)] = HOLDING_REG_APPLY_MSK;

    return (SJC_REGISTER_WRITING_PASS ==
            this->setHoldingRegisters(HOLDING_REG_PID_CONTROLLER_PARAMS_KP, nb, tab_reg))
               ? SJC_RET_OK
               : SJC_RET_NOK;
}

/* ============================================================
 *  LQR Controller
 * ============================================================ */

eSjcReturnType_t SjcInterface::getLQRControllerParameters(xSjcLQRControllerParameters_t *config)
{
    const int nb = 54;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS ==
        this->getInputRegisters(INPUT_REG_LQR_CONTROLLER_PARAMS_ORDER, nb, tab_reg))
    {
        config->order =
            tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_ORDER, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)];
        config->use_observer =
            tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_USE_OBS, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)];

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_KX0, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Kx0);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_KX1, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Kx1);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_KI, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Ki);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_KW, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Kw);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_CUTOFF_FREQ, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->cutoff_freq);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_U_MIN, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->u_min);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_U_MAX, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->u_max);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_AD00, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Ad[0][0]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_AD01, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Ad[0][1]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_AD10, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Ad[1][0]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_AD11, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Ad[1][1]);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_BD0, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Bd[0]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_BD1, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Bd[1]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_CD0, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Cd[0]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_CONTROLLER_PARAMS_CD1, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Cd[1]);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_OBSERVER_PARAMS_AO00, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Ao[0][0]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_OBSERVER_PARAMS_AO01, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Ao[0][1]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_OBSERVER_PARAMS_AO10, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Ao[1][0]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_OBSERVER_PARAMS_AO11, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Ao[1][1]);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_OBSERVER_PARAMS_BO0, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Bo[0]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_OBSERVER_PARAMS_BO1, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Bo[1]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_OBSERVER_PARAMS_CO0, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Co[0]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_OBSERVER_PARAMS_CO1, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->Co[1]);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_OBSERVER_PARAMS_L0, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->L[0]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_OBSERVER_PARAMS_L1, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->L[1]);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_LQR_OBSERVER_PARAMS_CUTOFF_FREQ, MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET)], &config->obs_cutoff_freq);

        return SJC_RET_OK;
    }
    return SJC_RET_NOK;
}

eSjcReturnType_t SjcInterface::setLQRControllerParameters(const xSjcLQRControllerParameters_t &config)
{
    const int nb = 55;
    uint16_t tab_reg[nb];

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_ORDER,
                                MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)] = config.order;
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_USE_OBS,
                                MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)] = config.use_observer;

    compose_ui16_array_from_float(config.Kx0,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_KX0, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Kx1,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_KX1, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Ki,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_KI, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Kw,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_KW, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.cutoff_freq,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_CUTOFF_FREQ, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.u_min,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_U_MIN, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.u_max,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_U_MAX, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);

    compose_ui16_array_from_float(config.Ad[0][0],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_AD00, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Ad[0][1],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_AD01, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Ad[1][0],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_AD10, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Ad[1][1],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_AD11, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);

    compose_ui16_array_from_float(config.Bd[0],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_BD0, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Bd[1],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_BD1, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Cd[0],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_CD0, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Cd[1],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_PARAMS_CD1, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);

    compose_ui16_array_from_float(config.Ao[0][0],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_OBSERVER_PARAMS_AO00, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Ao[0][1],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_OBSERVER_PARAMS_AO01, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Ao[1][0],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_OBSERVER_PARAMS_AO10, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Ao[1][1],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_OBSERVER_PARAMS_AO11, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);

    compose_ui16_array_from_float(config.Bo[0],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_OBSERVER_PARAMS_BO0, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Bo[1],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_OBSERVER_PARAMS_BO1, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Co[0],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_OBSERVER_PARAMS_CO0, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.Co[1],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_OBSERVER_PARAMS_CO1, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);

    compose_ui16_array_from_float(config.L[0],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_OBSERVER_PARAMS_L0, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.L[1],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_OBSERVER_PARAMS_L1, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);

    compose_ui16_array_from_float(config.obs_cutoff_freq,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_OBSERVER_PARAMS_CUTOFF_FREQ, MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_LQR_CONTROLLER_UPDATE,
                                MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET)] = HOLDING_REG_APPLY_MSK;

    return (SJC_REGISTER_WRITING_PASS ==
            this->setHoldingRegisters(HOLDING_REG_LQR_CONTROLLER_PARAMS_ORDER, nb, tab_reg))
               ? SJC_RET_OK
               : SJC_RET_NOK;
}

/* ============================================================
 *  Discrete Controller
 * ============================================================ */

eSjcReturnType_t SjcInterface::getDiscreteControllerParameters(xSjcDiscreteControllerParameters_t *config)
{
    const int nb = 20;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS ==
        this->getInputRegisters(INPUT_REG_DIGITAL_CONTROLLER_PARAMS_A0, nb, tab_reg))
    {
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_DIGITAL_CONTROLLER_PARAMS_A0, MAP_INPUT_REG_DIGITAL_CONTROLLER_INFO_OFFSET)], &config->A[0]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_DIGITAL_CONTROLLER_PARAMS_A1, MAP_INPUT_REG_DIGITAL_CONTROLLER_INFO_OFFSET)], &config->A[1]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_DIGITAL_CONTROLLER_PARAMS_A2, MAP_INPUT_REG_DIGITAL_CONTROLLER_INFO_OFFSET)], &config->A[2]);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_DIGITAL_CONTROLLER_PARAMS_B0, MAP_INPUT_REG_DIGITAL_CONTROLLER_INFO_OFFSET)], &config->B[0]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_DIGITAL_CONTROLLER_PARAMS_B1, MAP_INPUT_REG_DIGITAL_CONTROLLER_INFO_OFFSET)], &config->B[1]);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_DIGITAL_CONTROLLER_PARAMS_B2, MAP_INPUT_REG_DIGITAL_CONTROLLER_INFO_OFFSET)], &config->B[2]);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_DIGITAL_CONTROLLER_PARAMS_U_MIN, MAP_INPUT_REG_DIGITAL_CONTROLLER_INFO_OFFSET)], &config->u_min);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_DIGITAL_CONTROLLER_PARAMS_U_MAX, MAP_INPUT_REG_DIGITAL_CONTROLLER_INFO_OFFSET)], &config->u_max);

        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_DIGITAL_CONTROLLER_PARAMS_K_AW, MAP_INPUT_REG_DIGITAL_CONTROLLER_INFO_OFFSET)], &config->K_aw);
        compose_float_from_ui16_array(
            &tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_DIGITAL_CONTROLLER_PARAMS_CFREQ, MAP_INPUT_REG_DIGITAL_CONTROLLER_INFO_OFFSET)], &config->cutoff_freq);

        return SJC_RET_OK;
    }
    return SJC_RET_NOK;
}

eSjcReturnType_t SjcInterface::setDiscreteControllerParameters(const xSjcDiscreteControllerParameters_t &config)
{
    const int nb = 21;
    uint16_t tab_reg[nb];

    compose_ui16_array_from_float(config.A[0],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_A0, MAP_HOLDING_REG_DIGITAL_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.A[1],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_A1, MAP_HOLDING_REG_DIGITAL_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.A[2],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_A2, MAP_HOLDING_REG_DIGITAL_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);

    compose_ui16_array_from_float(config.B[0],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_B0, MAP_HOLDING_REG_DIGITAL_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.B[1],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_B1, MAP_HOLDING_REG_DIGITAL_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.B[2],
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_B2, MAP_HOLDING_REG_DIGITAL_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);

    compose_ui16_array_from_float(config.u_min,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_U_MIN, MAP_HOLDING_REG_DIGITAL_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.u_max,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_U_MAX, MAP_HOLDING_REG_DIGITAL_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.K_aw,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_K_AW, MAP_HOLDING_REG_DIGITAL_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);
    compose_ui16_array_from_float(config.cutoff_freq,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_CFREQ, MAP_HOLDING_REG_DIGITAL_CONTROLLER_PARAMETERS_UPDATE_OFFSET)]);

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_DIGITAL_CONTROLLER_UPDATE,
                                MAP_HOLDING_REG_DIGITAL_CONTROLLER_PARAMETERS_UPDATE_OFFSET)] = HOLDING_REG_APPLY_MSK;

    return (SJC_REGISTER_WRITING_PASS ==
            this->setHoldingRegisters(HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_A0, nb, tab_reg))
               ? SJC_RET_OK
               : SJC_RET_NOK;
}

/* ============================================================
 *  Sensors
 * ============================================================ */

eSjcReturnType_t SjcInterface::getSensorsParameters(xSjcSensorsConfig_t *config)
{
    const int nb = 3;
    uint16_t tab_reg[nb];

    if (SJC_REGISTER_READING_PASS ==
        this->getInputRegisters(INPUT_REG_ACTUATOR_ENCODER_RESOLUTION_PPR, nb, tab_reg))
    {
        config->actuator_encoder_resolution_ppr =
            tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_ACTUATOR_ENCODER_RESOLUTION_PPR, MAP_INPUT_REG_SENSORS_INFO_OFFSET)];
        config->swing_thx_encoder_resolution_ppr =
            tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_SWING_THX_ENCODER_RESOLUTION_PPR, MAP_INPUT_REG_SENSORS_INFO_OFFSET)];
        config->swing_thy_encoder_resolution_ppr =
            tab_reg[GET_REG_TABLE_INDEX(INPUT_REG_SWING_THY_ENCODER_RESOLUTION_PPR, MAP_INPUT_REG_SENSORS_INFO_OFFSET)];

        return SJC_RET_OK;
    }
    return SJC_RET_NOK;
}

eSjcReturnType_t SjcInterface::setSensorsParameters(const xSjcSensorsConfig_t &config)
{
    const int nb = 4;
    uint16_t tab_reg[nb];

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_ACTUATOR_ENCODER_RESOLUTION_PPR,
                                MAP_HOLDING_REG_SENSORS_PARAMETERS_UPDATE_OFFSET)] = config.actuator_encoder_resolution_ppr;
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_SWING_THX_ENCODER_RESOLUTION_PPR,
                                MAP_HOLDING_REG_SENSORS_PARAMETERS_UPDATE_OFFSET)] = config.swing_thx_encoder_resolution_ppr;
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_SWING_THY_ENCODER_RESOLUTION_PPR,
                                MAP_HOLDING_REG_SENSORS_PARAMETERS_UPDATE_OFFSET)] = config.swing_thy_encoder_resolution_ppr;

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_SENSORS_SETUP_UPDATE,
                                MAP_HOLDING_REG_SENSORS_PARAMETERS_UPDATE_OFFSET)] = HOLDING_REG_APPLY_MSK;

    return (SJC_REGISTER_WRITING_PASS ==
            this->setHoldingRegisters(HOLDING_REG_ACTUATOR_ENCODER_RESOLUTION_PPR, nb, tab_reg))
               ? SJC_RET_OK
               : SJC_RET_NOK;
}

/* ============================================================
 *  Controller reference & config
 * ============================================================ */

eSjcReturnType_t SjcInterface::setControllerReference(const float &reference)
{
    const int nb = 3;
    uint16_t tab_reg[nb];

    compose_ui16_array_from_float(reference,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_CONTROLLER_REFERENCE_VALUE,
                                                               MAP_HOLDING_REG_CONTROLLER_REFERENCE_UPDATE_OFFSET)]);

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_CONTROLLER_REFERENCE_UPDATE,
                                MAP_HOLDING_REG_CONTROLLER_REFERENCE_UPDATE_OFFSET)] = HOLDING_REG_APPLY_MSK;

    return (SJC_REGISTER_WRITING_PASS ==
            this->setHoldingRegisters(HOLDING_REG_CONTROLLER_REFERENCE_VALUE, nb, tab_reg))
               ? SJC_RET_OK
               : SJC_RET_NOK;
}

eSjcReturnType_t SjcInterface::setControllerMode(const eSjcControlMode_t &mode)
{
    const int nb = 2;
    uint16_t tab_reg[nb];

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_CONTROL_MODE, MAP_HOLDING_REG_CONTROL_MODE_UPDATE_OFFSET)] = static_cast<uint16_t>(mode);
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_CONTROL_MODE_UPDATE, MAP_HOLDING_REG_CONTROL_MODE_UPDATE_OFFSET)] = HOLDING_REG_APPLY_MSK;

    return (SJC_REGISTER_WRITING_PASS ==
            this->setHoldingRegisters(HOLDING_REG_CONTROL_MODE, nb, tab_reg))
               ? SJC_RET_OK
               : SJC_RET_NOK;
}

eSjcReturnType_t SjcInterface::setControllerUpdateRate(const float &rate)
{
    const int nb = 3;
    uint16_t tab_reg[nb];

    compose_ui16_array_from_float(rate,
                                  &tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_CONTROL_FREQUENCY, MAP_HOLDING_REG_CONTROL_FREQUENCY_UPDATE_OFFSET)]);
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_CONTROL_FREQUENCY_UPDATE, MAP_HOLDING_REG_CONTROL_FREQUENCY_UPDATE_OFFSET)] = HOLDING_REG_APPLY_MSK;

    return (SJC_REGISTER_WRITING_PASS ==
            this->setHoldingRegisters(HOLDING_REG_CONTROL_FREQUENCY, nb, tab_reg))
               ? SJC_RET_OK
               : SJC_RET_NOK;
}

/* ============================================================
 *  Network Parameters
 * ============================================================ */

eSjcReturnType_t SjcInterface::setNetworkParameters(const uint16_t &address, const char &parity, const uint32_t &baudrate)

{
    const int nb = 5;
    uint16_t tab_reg[nb];

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_NETWORK_CONFIG_ADDRESS,
                                MAP_HOLDING_REG_NETWORK_PARAMETERS_UPDATE_OFFSET)] = address;
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_NETWORK_CONFIG_PARITY,
                                MAP_HOLDING_REG_NETWORK_PARAMETERS_UPDATE_OFFSET)] = static_cast<uint16_t>(parity);
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_NETWORK_CONFIG_BAUD_RATE,
                                MAP_HOLDING_REG_NETWORK_PARAMETERS_UPDATE_OFFSET)] = baudrate >> 16;
    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_NETWORK_CONFIG_BAUD_RATE_HI,
                                MAP_HOLDING_REG_NETWORK_PARAMETERS_UPDATE_OFFSET)] = baudrate & 0xFFFF;

    tab_reg[GET_REG_TABLE_INDEX(HOLDING_REG_NETWORK_CONFIG_UPDATE,
                                MAP_HOLDING_REG_NETWORK_PARAMETERS_UPDATE_OFFSET)] = HOLDING_REG_APPLY_MSK;

    return (SJC_REGISTER_WRITING_PASS ==
            this->setHoldingRegisters(HOLDING_REG_NETWORK_CONFIG_ADDRESS, nb, tab_reg))
               ? SJC_RET_OK
               : SJC_RET_NOK;
}

/* ============================================================
 *  Private helpers
 * ============================================================ */

eSjcReturnType_t SjcInterface::init()
{
    ctx = modbus_new_rtu(this->port_.c_str(), baudrate_, 'N', 8, 1);
    if (ctx == nullptr)
    {
        std::cerr << "[SJC] Unable to create libmodbus context" << std::endl;
        this->release();
        return SJC_CONNECTION_RELEASED;
    }
    return SJC_INITIATED;
}

eSjcReturnType_t SjcInterface::release()
{
    if (ctx)
        modbus_free(ctx);
    ctx = nullptr;
    return SJC_CONNECTION_RELEASED;
}

eSjcReturnType_t SjcInterface::getInputRegisters(const int initial_reg, const int num_of_registers, uint16_t *reg_table)
{
    int rc = modbus_read_input_registers(ctx,
                                         GET_INPUT_REG_ADDRESS(initial_reg),
                                         num_of_registers,
                                         reg_table);
    if (rc == -1)
    {
        std::cerr << "[SJC] Input register reading failed: " << modbus_strerror(errno) << std::endl;
        modbus_flush(ctx);
        return SJC_REGISTER_READING_FAIL;
    }
    waitTime_us(STD_WAIT_TIME_US);
    return SJC_REGISTER_READING_PASS;
}

eSjcReturnType_t SjcInterface::setHoldingRegisters(const int initial_reg, const int num_of_registers, uint16_t *reg_table)
{
    int rc = modbus_write_registers(ctx,
                                    GET_HOLDING_REG_ADDRESS(initial_reg),
                                    num_of_registers,
                                    reg_table);
    if (rc == -1)
    {
        std::cerr << "[SJC] Holding register writing failed: " << modbus_strerror(errno) << std::endl;
        modbus_flush(ctx);
        return SJC_REGISTER_WRITING_FAIL;
    }
    waitTime_us(STD_WAIT_TIME_US);
    return SJC_REGISTER_WRITING_PASS;
}

eSjcReturnType_t SjcInterface::waitTime_us(const unsigned int &wait_time_us)
{
    std::this_thread::sleep_for(std::chrono::microseconds(wait_time_us));
    return SJC_RET_OK;
}
