/*
 * data_structures.h
 *
 *  Created on: Sep 17, 2023
 *      Author: Tiago
 *
 *  @brief Core system-wide enumerations and data structures used across
 *         controllers, observers, actuators, and network interfaces.
 */

#ifndef SJC__SYSTEM_TYPES__DATA_STRUCTURES_H_
#define SJC__SYSTEM_TYPES__DATA_STRUCTURES_H_

// ------------------------------------------------------------------- //
// System includes
// ------------------------------------------------------------------- //
#include "base/include/configs.h"
#include "stm32f446xx.h"
#include <stdbool.h>

// ------------------------------------------------------------------- //
// Global definitions
// ------------------------------------------------------------------- //
#define ZERO_FLOAT ((float)0.0f)
#define LQR_CONTROLLER_STATE_SIZE 2u ///< Maximum supported states for LQR

// ------------------------------------------------------------------- //
// Enumerations
// ------------------------------------------------------------------- //

/**
 * @enum eSjcReturnType_t
 * @brief Generic return codes.
 */
typedef enum
{
    SJC_RET_NOK = 0x0000,
    SJC_RET_OK = 0x00ff,
    SJC_RET_SIZE
} eSjcReturnType_t;

/**
 * @enum eSjcDigitalPortType_t
 * @brief Types of digital ports.
 */
typedef enum
{
    SJC_DIGITAL_OUTPUT_PORT = 0,
    SJC_DIGITAL_INPUT_PORT,
    SJC_DIGITAL_PORT_TYPE_SIZE
} eSjcDigitalPortType_t;

/**
 * @enum eSjcTimeType_t
 * @brief Time unit specifier.
 */
typedef enum
{
    SJC_TIME_MS = 0,
    SJC_TIME_SEC,
    SJC_TIME_TYPE_SIZE
} eSjcTimeType_t;

/**
 * @enum eSjcJointType_t
 * @brief Differentiates between active and passive joints.
 */
typedef enum
{
    SJC_PASSIVE_JOINT = 0,
    SJC_ACTIVE_JOINT,
    SJC_JOINT_TYPE_SIZE
} eSjcJointType_t;

/**
 * @enum eSjcJoints_t
 * @brief Supported joints in the crane system.
 */
typedef enum
{
    SJC_ACTUATOR_JOINT = 0,
    SJC_AUX_JOINT_TH_X,
    SJC_AUX_JOINT_TH_Y,
    SJC_JOINTS_SIZE
} eSjcJoints_t;

/**
 * @enum eSjcControlMode_t
 * @brief Supported controller modes.
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

/**
 * @enum eSjcInputType_t
 * @brief Input signal generation modes.
 */
typedef enum
{
    INPUT_TYPE_NONE = 0,
    INPUT_TYPE_LOW_FREQ_PRBS,
    INPUT_TYPE_MID_FREQ_PRBS,
    INPUT_TYPE_FAST_FREQ_PRBS,
    INPUT_TYPE_SINE_PRBS,
    INPUT_TYPE_PULSE_FAST,
    INPUT_TYPE_PULSE_SLOW,
    INPUT_TYPE_SIZE
} eSjcInputType_t;

/**
 * @enum eSjcInputSignalState_t
 * @brief State of input signal generation.
 */
typedef enum
{
    INPUT_SIGNAL_STATE_INIT_CONFIG = 0,
    INPUT_SIGNAL_STATE_RUNNING,
    INPUT_SIGNAL_STATE_FINISHED,
    INPUT_SIGNAL_STATE_SIZE
} eSjcInputSignalState_t;

/**
 * @enum eSjcAnalogSensorType_t
 * @brief Types of analog sensors.
 */
typedef enum
{
    SJC_CURRENT_SENSOR_STD = 0,
    SJC_CURRENT_SENSOR_AUX,
#if SJC_HW_VERSION_MAJOR > 1
    SJC_CURRENT_SENSOR_INA240_16A,
    SJC_CURRENT_SENSOR_INA240_25A,
    SJC_VREF_SENSOR,
#endif
    SJC_VOLTAGE_SENSOR,
    SJC_ANALOG_SENSOR_SIZE
} eSjcAnalogSensorType_t;

/**
 * @enum eSjcDigitalInputs_t
 * @brief Digital input channels.
 */
typedef enum
{
    SJC_JOINT_SWITCH_A = 0,
    SJC_JOINT_SWITCH_B,
    SJC_ISOLATED_INPUT_A,
    SJC_ISOLATED_INPUT_B,
#if SJC_HW_VERSION_MAJOR > 1
    SJC_NET_ADDR_A_INPUT,
    SJC_NET_ADDR_B_INPUT,
    SJC_NET_ADDR_C_INPUT,
#endif
    SJC_DIGITAL_INPUTS_SIZE
} eSjcDigitalInputs_t;

/**
 * @enum eSjcDigitalOutputs_t
 * @brief Digital output channels.
 */
typedef enum
{
    SJC_DIGITAL_OUTPUT_A = 0,
    SJC_DIGITAL_OUTPUT_B,
    SJC_HEARTBEAT_OUTPUT,
    SJC_DIGITAL_OUTPUTS_SIZE
} eSjcDigitalOutputs_t;

/**
 * @enum eSjcPwmBaseFreq_t
 * @brief Supported PWM base frequencies.
 */
typedef enum
{
    PWM_FREQ_10kHz = 0,
    PWM_FREQ_12kHz,
    PWM_FREQ_14kHz,
    PWM_FREQ_15kHz,
    PWM_FREQ_16kHz,
    PWM_FREQ_18kHz,
    PWM_FREQ_20kHz
} eSjcPwmBaseFreq_t;

// ------------------------------------------------------------------- //
// Data structures
// ------------------------------------------------------------------- //

/**
 * @struct xSjcMeanSampleTimeMeasurement_t
 * @brief Stores mean sample time statistics.
 */
typedef struct
{
    uint32_t Samples;               ///< Number of samples measured
    uint32_t AccumulatedTimerTicks; ///< Accumulated timer ticks
} xSjcMeanSampleTimeMeasurement_t;
static const xSjcMeanSampleTimeMeasurement_t xSjcSampleTimeEmptyObj = {0};

/**
 * @struct xSjcRange_t
 * @brief Generic min-max range structure.
 */
typedef struct
{
    float Min; ///< Minimum value
    float Max; ///< Maximum value
} xSjcRange_t;
static const xSjcRange_t xSjcRangeEmptyObj = {0};

/**
 * @struct xSjcEncoder_t
 * @brief Encoder configuration and real-time state.
 */
typedef struct
{
    float Encoder;                      ///< Current encoder angle [rad]
    int32_t Counts;                     ///< Raw encoder counts
    TIM_TypeDef *Instance;              ///< Associated hardware timer
    uint16_t Resolution_CPR;            ///< Pulses per revolution
    float radians_per_quadrature_pulse; ///< Conversion factor [rad/pulse]
} xSjcEncoder_t;

/**
 * @struct xSjcLQRObserverParameters_t
 * @brief Observer matrices and configuration.
 */
typedef struct
{
    float Ao[LQR_CONTROLLER_STATE_SIZE][LQR_CONTROLLER_STATE_SIZE]; ///< Observer A matrix
    float Bo[LQR_CONTROLLER_STATE_SIZE];                            ///< Observer B matrix
    float Co[LQR_CONTROLLER_STATE_SIZE];                            ///< Observer C matrix
    float L[LQR_CONTROLLER_STATE_SIZE];                             ///< Observer gain vector
    float obs_cutoff_freq;                                          ///< Observer cutoff frequency [Hz]
} xSjcLQRObserverParameters_t;
static const xSjcLQRObserverParameters_t xSjcLQRObserverParametersEmptyObj = {0};

/**
 * @struct xSjcLQRObserver_t
 * @brief Observer state and parameters.
 */
typedef struct
{
    float x_hat[LQR_CONTROLLER_STATE_SIZE]; ///< Estimated states
    xSjcLQRObserverParameters_t parameters; ///< Observer parameters
} xSjcLQRObserver_t;
static const xSjcLQRObserver_t xSjcLQRObserverEmptyObj = {0};

/**
 * @struct xSjcLQRControllerParameters_t
 * @brief LQR controller configuration, gains, and model parameters.
 */
typedef struct
{
    /* ---- Configs ---- */
    uint16_t order;        ///< Model order (1 or 2 states)
    uint16_t use_observer; ///< Enable observer (only valid when order=2)

    /* ---- Controller ---- */
    float Kx[LQR_CONTROLLER_STATE_SIZE]; ///< State-feedback gains
    float Ki;                            ///< Integral gain
    float Kw;                            ///< Anti-windup/weighting gain
    float cutoff_freq;                   ///< Error filter cutoff [Hz]
    float u_min;                         ///< Lower saturation limit
    float u_max;                         ///< Upper saturation limit

    /* ---- Discrete model ---- */
    float Ad[LQR_CONTROLLER_STATE_SIZE][LQR_CONTROLLER_STATE_SIZE]; ///< State transition matrix
    float Bd[LQR_CONTROLLER_STATE_SIZE];                            ///< Input matrix
    float Cd[LQR_CONTROLLER_STATE_SIZE];                            ///< Output matrix
    bool configured;
} xSjcLQRControllerParameters_t;
static const xSjcLQRControllerParameters_t xSjcLQRControllerParametersEmptyObj = {0};

/**
 * @struct xSjcLQRController_t
 * @brief LQR/LQI controller with state, integrator, and parameters.
 */
typedef struct
{
    uint16_t order;        ///< Model order (1 or 2 states)
    float obs_cutoff_freq; ///< Observer cutoff frequency [Hz]
    bool initialized;      ///< Initialization flag

    float Reference;     ///< Latest reference
    float ControlAction; ///< Latest control action
    float Error;         ///< Latest control error

    float x[LQR_CONTROLLER_STATE_SIZE]; ///< State vector
    float e_int;                        ///< Integral state
    float u_last;                       ///< Last applied control effort

    xSjcLQRControllerParameters_t parameters; ///< Controller parameters
} xSjcLQRController_t;
static const xSjcLQRController_t xSjcLQRControllerEmptyObj = {0};

/**
 * @struct xSjcDiscreteControllerParameters_t
 * @brief Parameters for a 2nd-order discrete-time controller (Direct Form II).
 */
typedef struct
{
    float A[3];        ///< Denominator coefficients [A0, A1, A2]
    float B[3];        ///< Numerator coefficients [B0, B1, B2]
    float u_min;       ///< Lower saturation limit
    float u_max;       ///< Upper saturation limit
    float K_aw;        ///< Anti-windup gain
    float cutoff_freq; ///< Error filter cutoff frequency [Hz]
    bool configured;
} xSjcDiscreteControllerParameters_t;
static const xSjcDiscreteControllerParameters_t xSjcDiscreteControllerParametersEmptyObj = {0};

/**
 * @struct xSjcDiscreteController_t
 * @brief Discrete-time controller state and parameters.
 */
typedef struct
{
    float Reference;        ///< Latest reference
    float ControlAction[3]; ///< Control outputs [u[k], u[k-1], u[k-2]]
    float Error[3];         ///< Errors [e[k], e[k-1], e[k-2]]

    xSjcDiscreteControllerParameters_t parameters; ///< Controller parameters
} xSjcDiscreteController_t;
static const xSjcDiscreteController_t xSjcDiscreteControllerEmptyObj = {0};

/**
 * @struct xPidControllerErrors_t
 * @brief Error terms for a PID controller.
 */
typedef struct
{
    float Derivative;  ///< Derivative term
    float Integral;    ///< Integral term
    float ActualError; ///< Current error
    float PrevError;   ///< Previous error
} xPidControllerErrors_t;
static const xPidControllerErrors_t xControllerErrorsEmptyObj = {0};

/**
 * @struct xSjcPidParameters_t
 * @brief PID controller gains and saturation limits.
 */
typedef struct
{
    float Kp;    ///< Proportional gain
    float Ki;    ///< Integral gain
    float Kd;    ///< Derivative gain
    float K_aw;  ///< Anti-windup gain
    float u_min; ///< Lower saturation limit
    float u_max; ///< Upper saturation limit
    bool configured;
} xSjcPidParameters_t;
static const xSjcPidParameters_t xSjcPidParametersEmptyObj = {0};

/**
 * @struct xSjcPidController_t
 * @brief PID controller state and parameters.
 */
typedef struct
{
    float Reference;                ///< Desired reference
    float ControlAction;            ///< Current control output
    xPidControllerErrors_t Errors;  ///< Error tracking
    xSjcPidParameters_t Parameters; ///< PID parameters
} xSjcPidController_t;
static const xSjcPidController_t xPidControllerEmptyObj = {0};

/**
 * @struct xSjcControlConfigs_t
 * @brief Control loop configuration (rate and mode).
 */
typedef struct
{
    float UpdateRate; ///< Update rate [Hz]
    float Reference;
    eSjcControlMode_t Mode; ///< Active control mode
} xSjcControlConfigs_t;
static const xSjcControlConfigs_t xSjcControlConfigsEmptyObj = {0};

/**
 * @struct xSjcActuatorPwmTimerConfig_t
 * @brief Hardware-specific PWM timer configuration.
 */
typedef struct
{
    TIM_TypeDef *Instance; ///< Associated timer instance
    uint32_t CountPeriod;  ///< Timer count period
    uint32_t LeftChannel;
    uint32_t LeftChannelComp;
    uint32_t RightChannel;
    uint32_t RightChannelComp;
} xSjcActuatorPwmTimerConfig_t;

/**
 * @struct xSjcActuatorPwm_t
 * @brief PWM actuator state and configuration.
 */
typedef struct
{
    float DutyCycle;                          ///< Current duty cycle (0.0 to 1.0)
    uint32_t Frequency;                       ///< PWM frequency [Hz]
    uint32_t DutyCount;                       ///< Timer compare value
    xSjcRange_t DutyRange;                    ///< Allowed duty cycle range
    xSjcActuatorPwmTimerConfig_t TimerConfig; ///< Timer configuration
} xSjcActuatorPwm_t;

/**
 * @struct xSjcDigitalPort_t
 * @brief Configuration for a digital port pin.
 */
typedef struct
{
    eSjcDigitalPortType_t Type; ///< Input or output
    GPIO_TypeDef *Port;         ///< GPIO port
    uint32_t Pin;               ///< GPIO pin
} xSjcDigitalPort_t;

/**
 * @struct xSjcMotorDrive_t
 * @brief DC motor drive control state.
 */
typedef struct
{
    bool State;                        ///< Drive enable state
    float BaseVoltage;                 ///< Base voltage [V]
    float OutputVoltage;               ///< Applied output voltage [V]
    xSjcRange_t OutputVoltageRange;    ///< Voltage limits
    xSjcDigitalPort_t DriveEnablePort; ///< Associated enable GPIO
} xSjcMotorDrive_t;

/**
 * @struct xSjcMotorOutput_t
 * @brief Power output state and control.
 */
typedef struct
{
    bool State;                         ///< Output enable state
    xSjcDigitalPort_t OutputEnablePort; ///< Associated enable GPIO
} xSjcMotorOutput_t;

/**
 * @struct xSjcNetworkConfig_t
 * @brief Network configuration parameters for Modbus/RS485.
 */
typedef struct
{
    uint16_t Address;  ///< Network address
    uint32_t Baudrate; ///< Baudrate [bps]
    uint8_t Parity;    ///< Parity configuration
} xSjcNetworkConfig_t;

/**
 * @struct xSjcJointControllerMeasurements_t
 * @brief Logged measurements from the joint control loop.
 */
typedef struct
{
    uint32_t Timestamp_ms;         ///< Timestamp [ms]
    float SystemVoltage;           ///< Supply voltage [V]
    float ControlAction;           ///< Last control output
    float ControllerError;         ///< Current error
    float ControllerReference;     ///< Reference value
    float ActuatorAngularPosition; ///< Position [rad]
    float ActuatorAngularRate;     ///< Velocity [rad/s]
    float ActuatorSpeedRpm;        ///< Speed [RPM]
    float ActuatorVoltage;         ///< Voltage [V]
    float ActuatorCurrent;         ///< Current [A]
    float AuxPositionMeas01;       ///< Auxiliary position 1
    float AuxPositionMeas02;       ///< Auxiliary position 2
} xSjcJointControllerMeasurements_t;
static const xSjcJointControllerMeasurements_t xSjcJointControllerMeasurementsEmptyObj = {0};

#endif /* SJC__SYSTEM_TYPES__DATA_STRUCTURES_H_ */
