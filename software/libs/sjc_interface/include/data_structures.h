#include "stdint.h"

typedef enum {
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
    uint8_t address;
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
} xSjcControllerConfig_t;

typedef struct
{
    uint16_t actuator_sensor_res_ppr;
    uint16_t auxA_sensor_res_ppr;
    uint16_t auxB_sensor_res_ppr;
} xSjcSensorsConfig_t;



/*
// Digital outputs update
	REG_HOLDING_DIGITAL_OUTPUTS_STATE_UPDATE			= ( (uint16_t) (  0UL + DIGITAL_OUTPUTS_OFFSET ) ),

	// Actuator controller update
	REG_HOLDING_CONTROLLER_REFERENCE					= ( (uint16_t) (  0UL + CONTROLLER_UPDATE_OFFSET ) ),
	REG_HOLDING_CONTROLLER_MODE_UPDATE					= ( (uint16_t) (  2UL + CONTROLLER_UPDATE_OFFSET ) ),

	// Actuator configs update
	REG_HOLDING_ACTUATOR_BASE_VOLTAGE					= ( (uint16_t) (  0UL + ACTUATOR_CONFIGS_UPDATE_OFFSET ) ),
	REG_HOLDING_ACTUATOR_LIMITS_OUTPUT_CURRENT			= ( (uint16_t) (  1UL + ACTUATOR_CONFIGS_UPDATE_OFFSET ) ),
	REG_HOLDING_ACTUATOR_LIMITS_OUTPUT_VOLTAGE			= ( (uint16_t) (  2UL + ACTUATOR_CONFIGS_UPDATE_OFFSET ) ),
	REG_HOLDING_ACTUATOR_LIMITS_OUTPUT_SPEED			= ( (uint16_t) (  3UL + ACTUATOR_CONFIGS_UPDATE_OFFSET ) ),
	REG_HOLDING_ACTUATOR_ENCODER_RESOLUTION_PPR			= ( (uint16_t) (  5UL + ACTUATOR_CONFIGS_UPDATE_OFFSET ) ),
	REG_HOLDING_ACTUATOR_SETUP_UPDATE					= ( (uint16_t) (  7UL + ACTUATOR_CONFIGS_UPDATE_OFFSET ) ),

	// Network parameters update
	REG_HOLDING_NETWORK_CONFIG_ADDRESS					= ( (uint16_t) (  0UL + NETWORK_PARAMETERS_UPDATE_OFFSET ) ),
	REG_HOLDING_NETWORK_CONFIG_PARITY					= ( (uint16_t) (  1UL + NETWORK_PARAMETERS_UPDATE_OFFSET ) ),
	REG_HOLDING_NETWORK_CONFIG_BAUD_RATE				= ( (uint16_t) (  2UL + NETWORK_PARAMETERS_UPDATE_OFFSET ) ),
	REG_HOLDING_NETWORK_CONFIG_UPDATE					= ( (uint16_t) (  4UL + NETWORK_PARAMETERS_UPDATE_OFFSET ) ),

	// Sensors parameters update
	REG_HOLDING_AUX_ENCODER_A_RESOLUTION_PPR			= ( (uint16_t) ( 0UL + SENSORS_PARAMETERS_UPDATE_OFFSET ) ),
	REG_HOLDING_AUX_ENCODER_B_RESOLUTION_PPR			= ( (uint16_t) ( 1UL + SENSORS_PARAMETERS_UPDATE_OFFSET ) ),
	REG_HOLDING_AUX_ENCODERS_SETUP_UPDATE				= ( (uint16_t) ( 2UL + SENSORS_PARAMETERS_UPDATE_OFFSET ) ),

	// Actuator controller parameters update
	REG_HOLDING_CONTROLLER_PARAMS_KP					= ( (uint16_t) (  0UL + CONTROLLER_PARAMETERS_UPDATE_OFFSET ) ),
	REG_HOLDING_CONTROLLER_PARAMS_KI					= ( (uint16_t) (  2UL + CONTROLLER_PARAMETERS_UPDATE_OFFSET ) ),
	REG_HOLDING_CONTROLLER_PARAMS_KD					= ( (uint16_t) (  4UL + CONTROLLER_PARAMETERS_UPDATE_OFFSET ) ),
	REG_HOLDING_CONTROLLER_PARAMS_AUX1					= ( (uint16_t) (  6UL + CONTROLLER_PARAMETERS_UPDATE_OFFSET ) ),
	REG_HOLDING_CONTROLLER_PARAMS_AUX2					= ( (uint16_t) (  8UL + CONTROLLER_PARAMETERS_UPDATE_OFFSET ) ),
	REG_HOLDING_CONTROLLER_PARAMS_RATE					= ( (uint16_t) ( 10UL + CONTROLLER_PARAMETERS_UPDATE_OFFSET ) ),
	REG_HOLDING_CONTROLLER_PARAMS_UPDATE				= ( (uint16_t) ( 12UL + CONTROLLER_PARAMETERS_UPDATE_OFFSET ) ),

	REG_HOLDING_SIZE									= ( (uint16_t) ( 20UL + CONTROLLER_PARAMETERS_UPDATE_OFFSET ) )*/