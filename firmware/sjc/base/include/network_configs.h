/*
 * network_configs.h
 *
 *  Firmware: SJC (STM32F446RE)
 *  Description: Modbus register map and network directives
 *
 *  Notes:
 *  - This header defines logical (0-based) register indices that are later
 *    translated to Modbus addresses using GET_INPUT_REG_ADDRESS() and
 *    GET_HOLDING_REG_ADDRESS().
 *  - Input registers (read-only) start at INPUT_REG_START (1001).
 *  - Holding registers (read/write) start at HOLDING_REG_START (1).
 *  - 32-bit values occupy two consecutive 16-bit registers (low word, high word).
 *    Low word at base index, high word at base+1.
 *  - The layout reserves contiguous blocks for controller parameter arrays:
 *      * PID controller block:      20 registers
 *      * LQR controller block:      40 registers
 *      * Digital controller block:  30 registers
 *    Adjusted offsets ensure these blocks do not overlap.
 *  - Units:
 *      * Currents in [A] (unless a device scaling is applied in firmware)
 *      * Voltages in [V]
 *      * Angular quantities in [rad] and [rad/s], linear in [m] and [m/s]
 */

#ifndef SJC__BASE__NETWORK_CONFIGS_H_
#define SJC__BASE__NETWORK_CONFIGS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/*==============================*/
/* Axis selection (exclusive)   */
/*==============================*/
/**
 * @brief Define exactly ONE axis. If none is defined, ROPE is used.
 *
 * Tip: Define one of these in your build system (e.g., compiler flags):
 *      -DBRIDGE_AXIS  or  -DTROLLEY_AXIS  or  -DROPE_AXIS
 */
// #define BRIDGE_AXIS
// #define TROLLEY_AXIS
#define ROPE_AXIS

#if defined(BRIDGE_AXIS)
  #define NETWORK_BASE_ADDRESS ((uint32_t)1UL)
#elif defined(TROLLEY_AXIS)
  #define NETWORK_BASE_ADDRESS ((uint32_t)2UL)
#else /* default */
  #ifndef ROPE_AXIS
    #define ROPE_AXIS
  #endif
  #define NETWORK_BASE_ADDRESS ((uint32_t)3UL)
#endif

/*==============================*/
/* Modbus / network parameters  */
/*==============================*/

/** @brief Number of retries/iterations used during USART+Modbus init */
#define MODBUS_USART_INIT_COUNT ((uint32_t)50UL)

/**
 * @brief Internal base codes (logical 0-based indices) that are converted
 *        to Modbus addresses using GET_INPUT_REG_ADDRESS() or
 *        GET_HOLDING_REG_ADDRESS().
 *
 * Input registers map to Modbus 3xxxx (device dependent).
 * Holding registers map to Modbus 4xxxx (device dependent).
 */
#define INPUT_REG_START   ((uint16_t)1001U)
#define HOLDING_REG_START ((uint16_t)1U)

/*==============================*/
/* Input block OFFSETS          */
/*==============================*/
/* Ranges (effective):
 *   BOARD_INFO:    0..9
 *   DIGITAL_IO:    10..19
 *   CONTROL_MEAS:  20..41 (42..49 reserved)
 *   ACTUATOR:      50..61
 *   PID:           70..89
 *   LQR:           100..139
 *   DIGITAL CTRL:  140..169
 *   SENSORS:       170..
 */
typedef enum
{
  MAP_INPUT_REG_BOARD_INFO_OFFSET               = (uint16_t)0U,
  MAP_INPUT_REG_DIGITAL_IO_INFO_OFFSET          = (uint16_t)10U,
  MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET        = (uint16_t)20U,
  MAP_INPUT_REG_ACTUATOR_INFO_OFFSET            = (uint16_t)50U,
  MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET      = (uint16_t)70U,
  MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET      = (uint16_t)100U,
  MAP_INPUT_REG_DIGITAL_CONTROLLER_INFO_OFFSET  = (uint16_t)140U,
  MAP_INPUT_REG_SENSORS_INFO_OFFSET             = (uint16_t)170U
} eInputRegisterOffsets_t;

/*==============================*/
/* Input REGISTERS (read-only)  */
/*==============================*/
typedef enum
{
  /* ---- Device/board info ---- */
  INPUT_REG_HW_VERSION                          = (uint16_t)(0U + MAP_INPUT_REG_BOARD_INFO_OFFSET),        /*  0          */
  INPUT_REG_FW_VERSION                          = (uint16_t)(1U + MAP_INPUT_REG_BOARD_INFO_OFFSET),        /*  1..2 (u32) */
  INPUT_REG_NETWORK_ADDRESS                     = (uint16_t)(3U + MAP_INPUT_REG_BOARD_INFO_OFFSET),        /*  3          */
  INPUT_REG_NETWORK_PARITY                      = (uint16_t)(4U + MAP_INPUT_REG_BOARD_INFO_OFFSET),        /*  4          */
  INPUT_REG_NETWORK_BAUD_RATE                   = (uint16_t)(5U + MAP_INPUT_REG_BOARD_INFO_OFFSET),        /*  5          */
  /* (reserved at +6) */
  INPUT_REG_CONTROL_MODE                        = (uint16_t)(7U + MAP_INPUT_REG_BOARD_INFO_OFFSET),        /*  7          */
  INPUT_REG_SYSTEM_ERROR_FLAGS                  = (uint16_t)(8U + MAP_INPUT_REG_BOARD_INFO_OFFSET),        /*  8          */

  /* ---- Digital IO info (16-bit) ---- */
  INPUT_REG_DIGITAL_INPUTS                      = (uint16_t)(0U + MAP_INPUT_REG_DIGITAL_IO_INFO_OFFSET),   /* 10 */
  INPUT_REG_DIGITAL_OUTPUTS                     = (uint16_t)(1U + MAP_INPUT_REG_DIGITAL_IO_INFO_OFFSET),   /* 11 */

  /* ---- Control & measurement data (32-bit pairs: lo,hi) ---- */
  INPUT_REG_SYSTEM_UPTIME                       = (uint16_t)(0U  + MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET),/* 20..21  */
  INPUT_REG_CONTROL_ERROR                       = (uint16_t)(2U  + MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET),/* 22..23  */
  INPUT_REG_CONTROL_REFERENCE                   = (uint16_t)(4U  + MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET),/* 24..25  */
  INPUT_REG_ACTUATOR_CURRENT                    = (uint16_t)(6U  + MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET),/* 26..27  */
  INPUT_REG_ACTUATOR_VOLTAGE                    = (uint16_t)(8U  + MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET),/* 28..29  */
  INPUT_REG_ACTUATOR_POSITION                   = (uint16_t)(10U + MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET),/* 30..31  */
  INPUT_REG_AUX_SWING_POSITION_THX              = (uint16_t)(12U + MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET),/* 32..33  */
  INPUT_REG_AUX_SWING_POSITION_THY              = (uint16_t)(14U + MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET),/* 34..35  */
  INPUT_REG_ACTUATOR_VELOCITY                   = (uint16_t)(16U + MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET),/* 36..37  */
  INPUT_REG_SWING_VELOCITY_THX                  = (uint16_t)(18U + MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET),/* 38..39  */
  INPUT_REG_SWING_VELOCITY_THY                  = (uint16_t)(20U + MAP_INPUT_REG_CONTROL_MEAS_DATA_OFFSET),/* 40..41  */

  /* ---- Actuator info (mixed width) ---- */
  INPUT_REG_ACTUATOR_STATE                      = (uint16_t)(0U  + MAP_INPUT_REG_ACTUATOR_INFO_OFFSET),    /* 50 (16-bit) */
  INPUT_REG_SYSTEM_VOLTAGE                      = (uint16_t)(2U  + MAP_INPUT_REG_ACTUATOR_INFO_OFFSET),    /* 52..53 (u32) */
  INPUT_REG_ACTUATOR_BASE_VOLTAGE               = (uint16_t)(4U  + MAP_INPUT_REG_ACTUATOR_INFO_OFFSET),    /* 54..55 (u32) */
  INPUT_REG_ACTUATOR_LIMITS_VOLTAGE             = (uint16_t)(6U  + MAP_INPUT_REG_ACTUATOR_INFO_OFFSET),    /* 56..57 (u32) */
  INPUT_REG_ACTUATOR_LIMITS_CURRENT             = (uint16_t)(8U  + MAP_INPUT_REG_ACTUATOR_INFO_OFFSET),    /* 58..59 (u32) */
  INPUT_REG_ACTUATOR_LIMITS_SPEED               = (uint16_t)(10U + MAP_INPUT_REG_ACTUATOR_INFO_OFFSET),    /* 60..61 (u32) */

  /* ---- PID Controller (readback; 32-bit pairs) ---- */
  INPUT_REG_CONTROLLER_PARAMS_KP                = (uint16_t)(0U  + MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET),  /* 70..71 */
  INPUT_REG_CONTROLLER_PARAMS_KI                = (uint16_t)(2U  + MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET),  /* 72..73 */
  INPUT_REG_CONTROLLER_PARAMS_KD                = (uint16_t)(4U  + MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET),  /* 74..75 */
  INPUT_REG_CONTROLLER_PARAMS_KT                = (uint16_t)(6U  + MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET),  /* 76..77 [N·m/A] */
  INPUT_REG_CONTROLLER_PARAMS_KA                = (uint16_t)(8U  + MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET),  /* 78..79 [V/(rad/s)] */
  INPUT_REG_CONTROLLER_PARAMS_RATE              = (uint16_t)(10U + MAP_INPUT_REG_PID_CONTROLLER_INFO_OFFSET),  /* 80..81 [Hz] */

  /* ---- LQR controller (array base) ---- */
  INPUT_REG_LQR_CONTROLLER_PARAMS_ARRAY         = (uint16_t)(0U + MAP_INPUT_REG_LQR_CONTROLLER_INFO_OFFSET),   /* 100..139 */

  /* ---- Digital controller (array base) ---- */
  INPUT_REG_DIGITAL_CONTROLLER_PARAMS_ARRAY     = (uint16_t)(0U + MAP_INPUT_REG_DIGITAL_CONTROLLER_INFO_OFFSET),/* 140..169 */

  /* ---- Sensors parameters (16-bit) ---- */
  INPUT_REG_ACTUATOR_ENCODER_RESOLUTION_PPR     = (uint16_t)(0U + MAP_INPUT_REG_SENSORS_INFO_OFFSET),          /* 170 */
  INPUT_REG_SWING_THX_ENCODER_RESOLUTION_PPR    = (uint16_t)(1U + MAP_INPUT_REG_SENSORS_INFO_OFFSET),          /* 171 */
  INPUT_REG_SWING_THY_ENCODER_RESOLUTION_PPR    = (uint16_t)(2U + MAP_INPUT_REG_SENSORS_INFO_OFFSET),          /* 172 */

  /* Table size: last reg + reserved tail (10) -> 172 + 10 = 182 */
  INPUT_REG_SIZE = (uint16_t)(10U + INPUT_REG_SWING_THY_ENCODER_RESOLUTION_PPR) /* 182 */
} eNetworkInputRegisters_t;

/*==============================*/
/* Holding block OFFSETS        */
/*==============================*/
/**
 * PID:      110..129
 * LQR:      130..169
 * DIGITAL:  170..199
 * ADC CFG:  200..
 */
typedef enum
{
  MAP_HOLDING_REG_SYSTEM_COMMANDS_OFFSET                      = (uint16_t)0U,
  MAP_HOLDING_REG_DIGITAL_OUTPUTS_OFFSET                      = (uint16_t)10U,   /*  10..19  */
  MAP_HOLDING_REG_CONTROLLER_MODE_UPDATE_OFFSET               = (uint16_t)20U,   /*  20..29  */
  MAP_HOLDING_REG_CONTROLLER_REFERENCE_UPDATE_OFFSET          = (uint16_t)30U,   /*  30..49  */
  MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET              = (uint16_t)50U,   /*  50..69  */
  MAP_HOLDING_REG_NETWORK_PARAMETERS_UPDATE_OFFSET            = (uint16_t)70U,   /*  70..89  */
  MAP_HOLDING_REG_SENSORS_PARAMETERS_UPDATE_OFFSET            = (uint16_t)90U,   /*  90..109 */
  MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET     = (uint16_t)110U,  /* 110..129 */
  MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET     = (uint16_t)130U,  /* 130..169 */
  MAP_HOLDING_REG_DIGITAL_CONTROLLER_PARAMETERS_UPDATE_OFFSET = (uint16_t)170U,  /* 170..199 */
  MAP_HOLDING_REG_ADC_FILTER_CONFIG_UPDATE_OFFSET             = (uint16_t)200U   /* 200..    */
} eHoldingRegisterOffsets_t;

/*==============================*/
/* Holding REGISTERS (R/W)      */
/*==============================*/
typedef enum
{
  /* ---- System commands (bitfield) ---- */
  HOLDING_REG_SYSTEM_COMMANDS                          = (uint16_t)(0U + MAP_HOLDING_REG_SYSTEM_COMMANDS_OFFSET), /* 0 */

  /* ---- Digital outputs ---- */
  HOLDING_REG_DIGITAL_OUTPUTS_STATE_UPDATE             = (uint16_t)(0U + MAP_HOLDING_REG_DIGITAL_OUTPUTS_OFFSET), /* 10 */

  /* ---- Controller mode ---- */
  HOLDING_REG_CONTROLLER_MODE_UPDATE                   = (uint16_t)(0U + MAP_HOLDING_REG_CONTROLLER_MODE_UPDATE_OFFSET), /* 20 */

  /* ---- Controller reference ---- */
  HOLDING_REG_CONTROLLER_REFERENCE_VALUE               = (uint16_t)(0U + MAP_HOLDING_REG_CONTROLLER_REFERENCE_UPDATE_OFFSET),  /* 30..31 (u32) */
  HOLDING_REG_CONTROLLER_REFERENCE_UPDATE              = (uint16_t)(2U + MAP_HOLDING_REG_CONTROLLER_REFERENCE_UPDATE_OFFSET),  /* 32 (apply) */

  /* ---- Actuator configs ---- */
  HOLDING_REG_ACTUATOR_BASE_VOLTAGE                    = (uint16_t)(0U + MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET),      /* 50 */
  HOLDING_REG_ACTUATOR_LIMITS_CURRENT                  = (uint16_t)(1U + MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET),      /* 51 */
  HOLDING_REG_ACTUATOR_LIMITS_VOLTAGE                  = (uint16_t)(2U + MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET),      /* 52 */
  HOLDING_REG_ACTUATOR_LIMITS_SPEED                    = (uint16_t)(3U + MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET),      /* 53 */
  HOLDING_REG_ACTUATOR_RESERVED                        = (uint16_t)(5U + MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET),      /* 55 */
  HOLDING_REG_ACTUATOR_SETUP_UPDATE                    = (uint16_t)(6U + MAP_HOLDING_REG_ACTUATOR_CONFIGS_UPDATE_OFFSET),      /* 56 (apply) */

  /* ---- Network parameters ---- */
  HOLDING_REG_NETWORK_CONFIG_ADDRESS                   = (uint16_t)(0U + MAP_HOLDING_REG_NETWORK_PARAMETERS_UPDATE_OFFSET),     /* 70 */
  HOLDING_REG_NETWORK_CONFIG_PARITY                    = (uint16_t)(1U + MAP_HOLDING_REG_NETWORK_PARAMETERS_UPDATE_OFFSET),     /* 71 */
  HOLDING_REG_NETWORK_CONFIG_BAUD_RATE                 = (uint16_t)(2U + MAP_HOLDING_REG_NETWORK_PARAMETERS_UPDATE_OFFSET),     /* 72 */
  HOLDING_REG_NETWORK_CONFIG_UPDATE                    = (uint16_t)(4U + MAP_HOLDING_REG_NETWORK_PARAMETERS_UPDATE_OFFSET),     /* 74 (apply) */

  /* ---- Sensors parameters ---- */
  HOLDING_REG_ACTUATOR_ENCODER_RESOLUTION_PPR          = (uint16_t)(0U + MAP_HOLDING_REG_SENSORS_PARAMETERS_UPDATE_OFFSET),     /* 90 */
  HOLDING_REG_SWING_THX_ENCODER_RESOLUTION_PPR         = (uint16_t)(1U + MAP_HOLDING_REG_SENSORS_PARAMETERS_UPDATE_OFFSET),     /* 91 */
  HOLDING_REG_SWING_THY_ENCODER_RESOLUTION_PPR         = (uint16_t)(2U + MAP_HOLDING_REG_SENSORS_PARAMETERS_UPDATE_OFFSET),     /* 92 */
  HOLDING_REG_SENSORS_SETUP_UPDATE                     = (uint16_t)(3U + MAP_HOLDING_REG_SENSORS_PARAMETERS_UPDATE_OFFSET),     /* 93 (apply) */

  /* ---- PID controller parameters ---- */
  HOLDING_REG_CONTROLLER_PARAMS_KP                     = (uint16_t)(0U  + MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET),   /* 110..111 (u32) */
  HOLDING_REG_CONTROLLER_PARAMS_KI                     = (uint16_t)(2U  + MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET),   /* 112..113 */
  HOLDING_REG_CONTROLLER_PARAMS_KD                     = (uint16_t)(4U  + MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET),   /* 114..115 */
  HOLDING_REG_CONTROLLER_PARAMS_KT                     = (uint16_t)(6U  + MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET),   /* 116..117 [N·m/A] */
  HOLDING_REG_CONTROLLER_PARAMS_KA                     = (uint16_t)(8U  + MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET),   /* 118..119 [V/(rad/s)] */
  HOLDING_REG_CONTROLLER_PARAMS_RATE                   = (uint16_t)(10U + MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET),   /* 120..121 [Hz] */
  HOLDING_REG_CONTROLLER_PARAMS_UPDATE                 = (uint16_t)(12U + MAP_HOLDING_REG_PID_CONTROLLER_PARAMETERS_UPDATE_OFFSET),   /* 122 (apply) */

  /* ---- LQR controller (array base) ---- */
  HOLDING_REG_LQR_CONTROLLER_PARAMS_ARRAY              = (uint16_t)(0U + MAP_HOLDING_REG_LQR_CONTROLLER_PARAMETERS_UPDATE_OFFSET),    /* 130..169 */

  /* ---- Digital controller (array base) ---- */
  HOLDING_REG_DIGITAL_CONTROLLER_PARAMS_ARRAY          = (uint16_t)(0U + MAP_HOLDING_REG_DIGITAL_CONTROLLER_PARAMETERS_UPDATE_OFFSET),/* 170..199 */

  /* ---- ADC filter config ---- */
  HOLDING_REG_ADC_FILTER_CONFIG_MODE                   = (uint16_t)(0U + MAP_HOLDING_REG_ADC_FILTER_CONFIG_UPDATE_OFFSET),            /* 200 */
  HOLDING_REG_ADC_FILTER_CONFIG_CUTOFF_FREQUENCY       = (uint16_t)(1U + MAP_HOLDING_REG_ADC_FILTER_CONFIG_UPDATE_OFFSET),            /* 201 [Hz] */
  HOLDING_REG_ADC_FILTER_CONFIG_UPDATE                 = (uint16_t)(3U + MAP_HOLDING_REG_ADC_FILTER_CONFIG_UPDATE_OFFSET),            /* 203 (apply) */

  /* Table size: last reg + reserved tail (10) -> 203 + 10 = 213 */
  HOLDING_REG_SIZE = (uint16_t)(10U + HOLDING_REG_ADC_FILTER_CONFIG_UPDATE) /* 213 */
} eNetworkHoldingRegisters_t;

/*==============================*/
/* Holding masks & SysCmd bits  */
/*==============================*/
typedef enum
{
  HOLDING_REG_APPLY_MSK = (uint16_t)(1U << 15)
} eHoldingRegisterMasks_t;

typedef enum
{
  SYS_CMD_REBOOT                          = (uint16_t)(1U << 0),
  SYS_CMD_RESET_PRIMARY_ENCODER_COUNTER   = (uint16_t)(1U << 1),
  SYS_CMD_RESET_AUX_ENCODER_COUNTER       = (uint16_t)(1U << 2),
  SYS_CMD_RESET_PRIMARY_JOINT_VELOCITY    = (uint16_t)(1U << 3),
  SYS_CMD_RESET_AUX_JOINT_VELOCITY        = (uint16_t)(1U << 4)
} eSystemCommandBits_t;

/*==============================*/
/* Helpers                      */
/*==============================*/
#define GET_INPUT_REG_ADDRESS(X)   ((uint16_t)(INPUT_REG_START   + (uint16_t)(X)))
#define GET_HOLDING_REG_ADDRESS(X) ((uint16_t)(HOLDING_REG_START + (uint16_t)(X)))
#define GET_REG_TABLE_INDEX(X, OFFS) ((uint16_t)((uint16_t)(X) - (uint16_t)(OFFS)))

#define INPUT_REG_NREGS   ((uint16_t)INPUT_REG_SIZE)
#define HOLDING_REG_NREGS ((uint16_t)HOLDING_REG_SIZE)

#ifdef __cplusplus
}
#endif

#endif /* SJC__BASE__NETWORK_CONFIGS_H_ */
