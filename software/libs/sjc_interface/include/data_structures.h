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
    uint32_t uptime;

    uint16_t controller_mode;
    float controller_error;
    float controller_ref;

    float actuator_displacement;
    float aux_A_displacement;
    float aux_B_displacement;

    uint16_t actuator_output_voltage;
    uint16_t actuator_current;

    uint16_t digital_inputs;
    uint16_t digital_outputs;

    uint16_t system_voltage;
    
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
} xSjcNetworkInfo_t;

typedef struct
{
    xSjcVersion_t hardware_ver;
    xSjcVersion_t firmware_ver;
} xSjcDeviceInfo_t;






