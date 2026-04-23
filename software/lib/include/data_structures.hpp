/*
 * sjc_data_structures.hpp
 *
 *  Created on: Sep 17, 2023
 *      Author: Tiago Giacomelli
 *
 *  @brief
 *  Data structures, enumerations, and configuration blocks
 *  used in the SJC interface. These map directly to the Modbus
 *  input and holding register layouts of the controller board.
 */

#ifndef SJC_DATA_STRUCTURES_HPP_
#define SJC_DATA_STRUCTURES_HPP_

#include <cstdbool>
#include <cstdint>

/* ================================================================
 * Enumerations
 * ================================================================ */

/**
 * @enum eSjcReturnType_t
 * @brief Generic return codes for SJC interface operations.
 *
 * These values indicate the result of Modbus communication
 * and register access operations.
 */
typedef enum
{
    SJC_RET_NOK = 0x0000,               ///< Operation failed
    SJC_RET_OK = 0x00ff,                ///< Operation successful
    SJC_REGISTER_READING_FAIL = 0xff00, ///< Failed to read register
    SJC_REGISTER_READING_PASS,          ///< Register read successful
    SJC_REGISTER_WRITING_FAIL,          ///< Failed to write register
    SJC_REGISTER_WRITING_PASS,          ///< Register write successful
    SJC_REGISTER_OUT_OF_RANGE,          ///< Register address out of range
    SJC_WRONG_REGISTER_TYPE,            ///< Wrong register type accessed
    SJC_CONNECTION_RELEASED,            ///< Connection closed/released
    SJC_SLAVE_ADDRESS_NOT_ASSIGNED,     ///< Slave address not assigned
    SJC_DISCONNECTED,                   ///< Device disconnected
    SJC_INITIATED,                      ///< Communication context initialized
    SJC_CONNECTED,                      ///< Connection established
    SJC_RET_SIZE                        ///< Enum size marker
} eSjcReturnType_t;

/**
 * @enum eSjcControlMode_t
 * @brief Controller operating modes.
 *
 * Defines the active control strategy on the device.
 */
typedef enum
{
    SJC_CONTROL_DISABLED = 0x0000,
    SJC_OPEN_LOOP_CONTROL,
    SJC_PID_SPEED_CONTROL,
    SJC_PID_CURRENT_CONTROL,
    SJC_LQR_CURRENT_CONTROL,
    SJC_DISCRETE_CURRENT_CONTROL,
    SJC_SLOW_PRBS_EXCITATION,
    SJC_MID_PRBS_EXCITATION,
    SJC_FAST_PRBS_EXCITATION,
    SJC_SINE_PRBS_EXCITATION,
    SJC_PULSE_FAST_EXCITATION,
    SJC_PULSE_SLOW_EXCITATION,
    SJC_ANALOG_CALIBRATION,
    SJC_CONTROL_MODE_SIZE
} eSjcControlMode_t;

/* ================================================================
 * Control Configuration
 * ================================================================ */

/**
 * @struct xSjcControlConfig_t
 * @brief Control configuration parameters (holding registers).
 */
typedef struct
{
    eSjcControlMode_t control_mode; ///< Controller operating mode (u16)
    float control_frequency;        ///< Control loop frequency [Hz] (float, 2 registers)
} xSjcControlConfig_t;

/* ================================================================
 * Digital I/O States
 * ================================================================ */

/**
 * @struct xSjcInputsStates_t
 * @brief Digital input states (read-only).
 */
typedef struct
{
    bool switch_a;    ///< State of switch A
    bool switch_b;    ///< State of switch B
    bool iso_input_a; ///< Isolated input A
    bool iso_input_b; ///< Isolated input B
} xSjcInputsStates_t;

/**
 * @struct xSjcOutputsStates_t
 * @brief Digital output states.
 */
typedef struct
{
    bool output_a; ///< Output A state
    bool output_b; ///< Output B state
} xSjcOutputsStates_t;

/**
 * @struct xSjcDigitalIOInfo_t
 * @brief Consolidated digital I/O structure.
 */
typedef struct
{
    xSjcInputsStates_t inputs;   ///< Input states
    xSjcOutputsStates_t outputs; ///< Output states
} xSjcDigitalIOInfo_t;

/* ================================================================
 * Measurement Information
 * ================================================================ */

/**
 * @struct xSjcControlMeasInfo_t
 * @brief Control loop measurement information (input registers).
 */
typedef struct
{
    uint32_t uptime;                  ///< System uptime [ms], 2 Modbus registers
    float controller_error;           ///< Control error
    float controller_ref;             ///< Reference value
    float actuator_current;           ///< Actuator current [A]
    float actuator_voltage;           ///< Actuator voltage [V]
    float actuator_angular_position;  ///< Actuator angular position [rad]
    float swing_angular_position_thx; ///< Swing angle θx [rad]
    float swing_angular_position_thy; ///< Swing angle θy [rad]
    float actuator_angular_velocity;  ///< Actuator angular velocity [rad/s]
    float swing_angular_velocity_thx; ///< Swing angular velocity θx [rad/s]
    float swing_angular_velocity_thy; ///< Swing angular velocity θy [rad/s]
} xSjcControlMeasInfo_t;

/* ================================================================
 * Board Information
 * ================================================================ */

/// @struct xSjcHwVersion_t
/// @brief Hardware version structure.
typedef struct
{
    uint8_t major; ///< Major version
    uint8_t minor; ///< Minor version
} xSjcHwVersion_t;

/// @struct xSjcFwVersion_t
/// @brief Firmware version structure.
typedef struct
{
    uint8_t major; ///< Major version
    uint8_t minor; ///< Minor version
    uint8_t patch; ///< Patch version
} xSjcFwVersion_t;

/**
 * @struct xSjcBoardInfo_t
 * @brief Board information (input registers).
 */
typedef struct
{
    xSjcHwVersion_t hardware_ver; ///< Hardware version
    xSjcFwVersion_t firmware_ver; ///< Firmware version
    uint16_t net_address;         ///< Network (slave) address
    char net_parity;              ///< Network parity (stored as u16 in Modbus)
    uint32_t net_baudrate;        ///< Baudrate [bps]
    uint16_t control_mode;        ///< Active control mode (see eSjcControlMode_t)
    float control_frequency;      ///< Control loop frequency [Hz]
    uint16_t error_flags;         ///< Error flags (bitfield)
} xSjcBoardInfo_t;

/* ================================================================
 * Actuator Parameters
 * ================================================================ */

/**
 * @struct xSjcActuatorInfo_t
 * @brief Actuator information and configuration.
 */
typedef struct
{
    bool state;           ///< Actuator enabled flag (bit0 of INPUT_REG_ACTUATOR_STATE)
    float system_voltage; ///< System supply voltage [V]
    float base_voltage;   ///< Actuator base voltage [V]
    float voltage_limit;  ///< Voltage limit [V]
    float current_limit;  ///< Current limit [A]
    float speed_limit;    ///< Speed limit [m/s] or [rad/s]
} xSjcActuatorInfo_t;

/* ================================================================
 * Controller Parameters
 * ================================================================ */

/**
 * @struct xSjcPIDControllerParameters_t
 * @brief PID controller parameters.
 */
typedef struct
{
    float Kp;    ///< Proportional gain
    float Ki;    ///< Integral gain
    float Kd;    ///< Derivative gain
    float K_aw;  ///< Anti-windup gain
    float u_min; ///< Lower saturation limit
    float u_max; ///< Upper saturation limit
} xSjcPIDControllerParameters_t;

/**
 * @struct xSjcLQRControllerParameters_t
 * @brief LQR controller and observer parameters.
 */
typedef struct
{
    /* ---- Controller ---- */
    uint16_t order;        ///< Model order (1 or 2)
    uint16_t use_observer; ///< Enable/disable observer

    float Kx0;         ///< State-feedback gain Kx0
    float Kx1;         ///< State-feedback gain Kx1
    float Ki;          ///< Integral gain
    float Kw;          ///< Anti-windup/weighting gain
    float cutoff_freq; ///< Controller cutoff frequency [Hz]
    float u_min;       ///< Lower saturation limit
    float u_max;       ///< Upper saturation limit

    /* ---- Discrete model ---- */
    float Ad[2][2]; ///< State transition matrix (up to 2x2)
    float Bd[2];    ///< Input matrix
    float Cd[2];    ///< Output matrix

    /* ---- Observer ---- */
    float Ao[2][2];        ///< Observer A matrix
    float Bo[2];           ///< Observer B matrix
    float Co[2];           ///< Observer C matrix
    float L[2];            ///< Observer gain vector
    float obs_cutoff_freq; ///< Observer cutoff frequency [Hz]
} xSjcLQRControllerParameters_t;

/**
 * @struct xSjcDiscreteControllerParameters_t
 * @brief Discrete-time digital controller parameters.
 */
typedef struct
{
    float A[3];        ///< Denominator coefficients [A0, A1, A2]
    float B[3];        ///< Numerator coefficients [B0, B1, B2]
    float u_min;       ///< Lower saturation limit
    float u_max;       ///< Upper saturation limit
    float K_aw;        ///< Anti-windup gain
    float cutoff_freq; ///< Filter cutoff frequency [Hz]
} xSjcDiscreteControllerParameters_t;

/* ================================================================
 * Sensor Parameters
 * ================================================================ */

/**
 * @struct xSjcSensorsConfig_t
 * @brief Encoder resolution parameters.
 *
 * CPR = Counts Per Revolution (quadrature mode, 4x the PPR value).
 * Example: A 1000 PPR encoder has 4000 CPR in quadrature mode.
 */
typedef struct
{
    uint16_t actuator_encoder_resolution_cpr;  ///< Main actuator encoder [counts/rev]
    uint16_t swing_thx_encoder_resolution_cpr; ///< Swing θx encoder [counts/rev]
    uint16_t swing_thy_encoder_resolution_cpr; ///< Swing θy encoder [counts/rev]
} xSjcSensorsConfig_t;

#endif // SJC_DATA_STRUCTURES_HPP_
