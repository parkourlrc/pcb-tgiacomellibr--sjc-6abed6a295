#include "stdint.h"

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
	SJC_PULSE_SLOW_EXCITATION
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
    uint16_t controller_mode;
    float system_voltage;
    float controller_error;
    float controller_ref;
    float actuator_voltage;
    float actuator_current;
    float actuator_displacement;
    float aux_displacement_a;
    float aux_displacement_b;
    xSjcInputsStates_t inputs;
    xSjcOutputsStates_t outputs;
    uint16_t error_flags;
} xSjcRuntimeInfo_t;

typedef struct
{
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
} xSjcVersion_t;

typedef struct
{
    uint16_t address;
    uint32_t baudrate;
    char parity;
} xSjcNetworkConfig_t;

typedef struct
{
    xSjcVersion_t hardware_ver;
    xSjcVersion_t firmware_ver;
} xSjcDeviceInfo_t;

typedef struct
{
    bool state;
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

typedef struct
{
    uint16_t controller_mode_update;
    float controller_ref;
} xSjcControllerConfig_t;
