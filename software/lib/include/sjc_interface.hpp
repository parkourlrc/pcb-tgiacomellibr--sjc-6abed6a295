#ifndef SJC_INTERFACE_HPP_
#define SJC_INTERFACE_HPP_

#ifdef __cplusplus
extern "C"
{
#endif
#include "modbus/modbus.h"
#include "network_configs.h"
#ifdef __cplusplus
}
#endif

#include "data_masks.hpp"
#include "data_structures.hpp"
#include "helpers.hpp"

#include <string>

/**
 * @class SjcInterface
 * @brief Base interface for Modbus communication with the SJC firmware (STM32).
 *
 * This class encapsulates all Modbus RTU operations (connect, read, write)
 * for interacting with the firmware-defined register map. It provides high-level
 * accessors for board information, control measurements, actuator parameters,
 * and controller settings (PID, LQR, Digital).
 *
 * ### Notes
 * - Uses libmodbus (C API) internally.
 * - Register map is defined in @ref network_configs.h.
 * - All floating-point values are encoded in two consecutive 16-bit registers (low, high).
 */
class SjcInterface
{
public:
    /**
     * @brief Construct interface with default Modbus slave address (247).
     * @param port Serial port device path (e.g. "/dev/ttyUSB0").
     * @param baudrate Modbus RTU baudrate.
     */
    SjcInterface(const std::string &port, const int baudrate);

    /**
     * @brief Construct interface with explicit Modbus slave address.
     * @param port Serial port device path.
     * @param address Modbus slave address.
     * @param baudrate Modbus RTU baudrate.
     */
    SjcInterface(const std::string &port, const int address, const int baudrate);

    /**
     * @brief Destructor: releases Modbus context.
     * @note Virtual because this class is also used as a base for SjcRealtimeInterface.
     */
    virtual ~SjcInterface();

    /**
     * @brief Establish Modbus connection.
     * @return Status code (SJC_CONNECTED if successful).
     */
    eSjcReturnType_t connect();

    /**
     * @brief Disconnect Modbus connection.
     * @return Status code (SJC_DISCONNECTED if closed).
     */
    eSjcReturnType_t disconnect();

    /**
     * @brief Get current Modbus slave address.
     * @param addr Pointer to store the slave address.
     * @return Status code.
     */
    eSjcReturnType_t getSlaveAddress(int *addr);

    /**
     * @brief Set Modbus slave address.
     * @param address New slave address.
     * @return Status code.
     */
    eSjcReturnType_t setSlaveAddress(const int address);

    // ============================
    //   System commands
    // ============================

    /** @brief Reboot the device. */
    eSjcReturnType_t reboot();

    /** @brief Reset auxiliary encoders. */
    eSjcReturnType_t resetAuxEncoders();

    /** @brief Reset primary joint encoder. */
    eSjcReturnType_t resetJointEncoder();

    /** @brief Reset auxiliary velocity estimation. */
    eSjcReturnType_t resetAuxVelocity();

    /** @brief Reset primary joint velocity estimation. */
    eSjcReturnType_t resetJointVelocity();

    // ============================
    //   Get methods (read input registers)
    // ============================

    /**
     * @brief Get static board information (firmware, hardware, network).
     * @param config Output struct with board information.
     * @return Status code.
     */
    eSjcReturnType_t getBoardInfo(xSjcBoardInfo_t *config);

    /**
     * @brief Get control measurement info (positions, velocities, currents, voltages).
     * @param config Output struct with control measurements.
     * @return Status code.
     */
    eSjcReturnType_t getControlMeasInfo(xSjcControlMeasInfo_t *config);

    /**
     * @brief Get current digital input/output states.
     * @param config Output struct with digital I/O information.
     * @return Status code.
     */
    eSjcReturnType_t getDigitalIOInfo(xSjcDigitalIOInfo_t *config);

    /**
     * @brief Get actuator configuration and parameters.
     * @param config Output struct with actuator parameters.
     * @return Status code.
     */
    eSjcReturnType_t getActuatorParameters(xSjcActuatorInfo_t *config);

    /**
     * @brief Get PID controller parameters.
     * @param config Output struct with PID parameters.
     * @return Status code.
     */
    eSjcReturnType_t getPIDControllerParameters(xSjcPIDControllerParameters_t *config);

    /**
     * @brief Get LQR controller parameters.
     * @param config Output struct with LQR parameters.
     * @return Status code.
     */
    eSjcReturnType_t getLQRControllerParameters(xSjcLQRControllerParameters_t *config);

    /**
     * @brief Get discrete controller parameters.
     * @param config Output struct with discrete controller parameters.
     * @return Status code.
     */
    eSjcReturnType_t getDiscreteControllerParameters(xSjcDiscreteControllerParameters_t *config);

    /**
     * @brief Get sensors configuration.
     * @param config Output struct with sensors configuration.
     * @return Status code.
     */
    eSjcReturnType_t getSensorsParameters(xSjcSensorsConfig_t *config);

    // ============================
    //   Set methods (write holding registers)
    // ============================

    /**
     * @brief Set actuator configuration and parameters.
     * @param config Actuator configuration struct.
     * @return Status code.
     */
    eSjcReturnType_t setActuatorParameters(const xSjcActuatorInfo_t &config);

    /**
     * @brief Set reference value for controller.
     * @param reference Desired reference value.
     * @return Status code.
     */
    eSjcReturnType_t setControllerReference(const float &reference);

    /**
     * @brief Configure sensors parameters.
     * @param config Sensors configuration struct.
     * @return Status code.
     */
    eSjcReturnType_t setSensorsParameters(const xSjcSensorsConfig_t &config);

    /**
     * @brief Change controller mode (PID, LQR, Digital).
     * @param mode Controller mode enum.
     * @return Status code.
     */
    eSjcReturnType_t setControllerMode(const eSjcControlMode_t &mode);

    /**
     * @brief Set controller update rate (Hz).
     * @param rate Update frequency in Hz.
     * @return Status code.
     */
    eSjcReturnType_t setControllerUpdateRate(const float &rate);

    /**
     * @brief Set digital outputs states.
     * @param config Digital outputs struct.
     * @return Status code.
     */
    eSjcReturnType_t setDigitalOutputsStates(const xSjcOutputsStates_t &config);

    /**
     * @brief Set PID controller parameters.
     * @param config PID configuration struct.
     * @return Status code.
     */
    eSjcReturnType_t setPIDControllerParameters(const xSjcPIDControllerParameters_t &config);

    /**
     * @brief Set LQR controller parameters.
     * @param config LQR configuration struct.
     * @return Status code.
     */
    eSjcReturnType_t setLQRControllerParameters(const xSjcLQRControllerParameters_t &config);

    /**
     * @brief Set discrete controller parameters.
     * @param config Discrete controller configuration struct.
     * @return Status code.
     */
    eSjcReturnType_t setDiscreteControllerParameters(const xSjcDiscreteControllerParameters_t &config);

    /**
     * @brief Configure network parameters.
     * @param address New Modbus slave address.
     * @param parity Parity (e.g. 'N', 'E', 'O').
     * @param baudrate New baudrate.
     * @return Status code.
     */
    eSjcReturnType_t setNetworkParameters(const uint16_t &address, const char &parity, const uint32_t &baudrate);

private:
    int address_;      ///< Modbus slave address
    int baudrate_;     ///< Baudrate for serial communication
    std::string port_; ///< Serial port device path
    modbus_t *ctx;     ///< libmodbus context handle

    /**
     * @brief Initialize Modbus RTU context.
     * @return Status code.
     */
    eSjcReturnType_t init();

    /**
     * @brief Release Modbus context.
     * @return Status code.
     */
    eSjcReturnType_t release();

    /**
     * @brief Low-level helper: read input registers.
     * @param initial_reg Starting register index.
     * @param num_of_registers Number of registers to read.
     * @param reg_table Destination buffer.
     * @return Status code.
     */
    eSjcReturnType_t getInputRegisters(const int initial_reg, const int num_of_registers, uint16_t *reg_table);

    /**
     * @brief Low-level helper: write holding registers.
     * @param initial_reg Starting register index.
     * @param num_of_registers Number of registers to write.
     * @param reg_table Source buffer.
     * @return Status code.
     */
    eSjcReturnType_t setHoldingRegisters(const int initial_reg, const int num_of_registers, uint16_t *reg_table);

    /**
     * @brief Sleep for given microseconds (for Modbus pacing).
     * @param wait_time_us Time in microseconds.
     * @return Status code.
     */
    static eSjcReturnType_t waitTime_us(const unsigned int &wait_time_us);
};

#endif // SJC_INTERFACE_HPP_
