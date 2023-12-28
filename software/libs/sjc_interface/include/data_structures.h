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
    SJC_DISCONNECTED,
    SJC_INITIATED,
    SJC_CONNECTED,
    SJC_RET_SIZE
} eSjcReturnType_t;

typedef enum {
	SJC_CONTROL_DISABLED 	= 0x0000,
	SJC_OPEN_LOOP_CONTROL 	= 0x0001,
	SJC_PRBS_EXCITATION 	= 0x00ff,
	SJC_SPEED_CONTROL 		= 0xff00,
	SJC_TORQUE_CONTROL 		= 0xffff
} eSjcControlMode_t;

typedef struct
{
    bool switchA;
    bool switchB;
    bool iso_inputA;
    bool iso_inputB;
} xSjcInputsStates_t;

typedef struct
{
    bool outputA;
    bool outputB;
} xSjcOutputsStates_t;

typedef struct
{
    uint32_t uptime;
    float system_voltage;
    uint16_t controller_mode;
    float controller_error;
    float controller_ref;
    float actuator_voltage;
    float actuator_current;
    float actuator_displacement;
    float aux_A_displacement;
    float aux_B_displacement;
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
    float kp_parameter;
    float ki_parameter;
    float kd_parameter;
    float aux1_parameter;
    float aux2_parameter;
    float rate_parameter;
} xSjcControllerParameters_t;

typedef struct
{
    uint16_t actuator_sensor_res_ppr;
    uint16_t auxA_sensor_res_ppr;
    uint16_t auxB_sensor_res_ppr;
} xSjcSensorsConfig_t;

typedef struct
{
    uint16_t controller_mode_update;
    float controller_ref;
} xSjcControllerConfig_t;