#ifndef SJC_DATA_STRUCTURES_HPP_
#define SJC_DATA_STRUCTURES_HPP_

#include "stdint.h"

/**
 * @brief Enumeration for selecting the ADC filtering mode.
 */
typedef enum
{
    ADC_FILTER_NONE = 0x00,         ///< No filtering, raw values used directly
    ADC_FILTER_EXPONENTIAL = 0x01,  ///< Exponential filtering on each sample
    ADC_FILTER_BOX = 0x02,          ///< Box filtering over a window
    ADC_FILTER_EXP_THEN_BOX = 0x03, ///< Exponential filter then box filter
    ADC_FILTER_BOX_THEN_EXP = 0x04, ///< Box filter then exponential filter
    ADC_FILTER_MODE_SIZE
} eSjcAdcFilterMode_t;

typedef enum
{
    SJC_RET_NOK = 0x0000,
    SJC_RET_OK = 0x00ff,
    SJC_REGISTER_READING_FAIL = 0xff00,
    SJC_REGISTER_READING_PASS,
    SJC_REGISTER_WRITING_FAIL,
    SJC_REGISTER_WRITING_PASS,
    SJC_REGISTER_OUT_OF_RANGE,
    SJC_WRONG_REGISTER_TYPE,
    SJC_CONNECTION_RELEASED,
    SJC_SLAVE_ADDRESS_NOT_ASSIGNED,
    SJC_DISCONNECTED,
    SJC_INITIATED,
    SJC_CONNECTED,
    SJC_RET_SIZE
} eSjcReturnType_t;

typedef enum
{
    SJC_CONTROL_DISABLED = 0x0000,
    SJC_OPEN_LOOP_CONTROL,
    SJC_SPEED_CONTROL,
    SJC_CURRENT_CONTROL,
    SJC_PRBS_EXCITATION,
    SJC_TRIANG_EXCITATION,
    SJC_PULSE_FAST_EXCITATION,
    SJC_PULSE_SLOW_EXCITATION,
    SJC_ANALOG_CALIBRATION,
    SJC_CONTROL_MODE_SIZE
} eSjcControlMode_t;

typedef struct
{
    bool switch_a;
    bool switch_b;
    bool iso_input_a;
    bool iso_input_b;
} xSjcInputsStates_t;

typedef struct
{
    bool output_a;
    bool output_b;
} xSjcOutputsStates_t;

typedef struct
{
    uint32_t uptime;
    float controller_error;
    float controller_ref;
    float actuator_current;
    float actuator_voltage;
    float actuator_angular_position;
    float swing_angular_position_thx;
    float swing_angular_position_thy;
    float actuator_angular_velocity;
    float swing_angular_velocity_thx;
    float swing_angular_velocity_thy;    
} xSjcControlMeasInfo_t;

typedef struct
{
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
} xSjcVersion_t;

typedef struct {
    xSjcInputsStates_t inputs;
    xSjcOutputsStates_t outputs;
} xSjcDigitalIOInfo_t;

typedef struct
{
    xSjcVersion_t hardware_ver;
    xSjcVersion_t firmware_ver;
    uint16_t net_address;
    char net_parity;
    uint32_t net_baudrate;
    uint16_t controller_mode;
    uint16_t error_flags;
} xSjcDeviceInfo_t;

typedef struct
{
    bool state;
    float system_voltage;
    float base_voltage;
    float voltage_limit;
    float current_limit;
    float speed_limit;
} xSjcActuatorConfig_t;

typedef struct
{
    float Kp;          // Proporcional gain
    float Ki;          // Integral gain
    float Kd;          // Derivative gain
    float Kt;          // Torque constant
    float Ka;          // Back-EMF constant
    float update_rate; // Update frequency
} xSjcControllerParameters_t;

typedef struct
{
    uint16_t actuator_sensor_resolution;
    uint16_t aux_sensor_resolution_a;
    uint16_t aux_sensor_resolution_b;
} xSjcSensorsConfig_t;

#endif // SJC_DATA_STRUCTURES_HPP_