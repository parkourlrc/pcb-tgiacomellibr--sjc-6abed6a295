#ifndef SJC_INTERFACE_HPP_
#define SJC_INTERFACE_HPP_

#ifdef __cplusplus
extern "C"
{
#include "data_structures.hpp"
#include "modbus/modbus.h"
}
#endif

#include <string>

class SjcInterface
{

public:
    SjcInterface(const std::string port, const int baudrate);
    SjcInterface(const std::string port, const int address, const int baudrate);
    ~SjcInterface(void);

    eSjcReturnType_t connect(void);
    eSjcReturnType_t disconnect(void);
    eSjcReturnType_t getSlaveAddress(int *addr);
    eSjcReturnType_t setSlaveAddress(const int address);
    
    eSjcReturnType_t reboot(void);
    eSjcReturnType_t resetAuxEncoders(void);
    eSjcReturnType_t resetJointEncoder(void);
    eSjcReturnType_t resetAuxVelocity(void);
    eSjcReturnType_t resetJointVelocity(void);

    eSjcReturnType_t getBoardInfo(xSjcDeviceInfo_t *config);
    eSjcReturnType_t getControlMeasInfo(xSjcControlMeasInfo_t *config);
    eSjcReturnType_t getDigitalIOInfo(xSjcDigitalIOInfo_t *config);    
    eSjcReturnType_t getActuatorParameters(xSjcActuatorConfig_t *config);
    eSjcReturnType_t getControllerParameters(xSjcControllerParameters_t *config);
    eSjcReturnType_t getSensorsParameters(xSjcSensorsConfig_t *config);
    
    eSjcReturnType_t setActuatorParameters(const xSjcActuatorConfig_t config);
    eSjcReturnType_t setControllerReference(const float reference);
    eSjcReturnType_t setSensorsParameters(const xSjcSensorsConfig_t config);
    eSjcReturnType_t setControllerOperatingMode(const eSjcControlMode_t mode);
    eSjcReturnType_t setDigitalOutputsStates(const xSjcOutputsStates_t config);
    eSjcReturnType_t setControllerParameters(const xSjcControllerParameters_t config);
    eSjcReturnType_t setNetworkParameters(const uint16_t address, const char parity, const uint32_t baudrate);
    eSjcReturnType_t setAdcFilterMode(const eSjcAdcFilterMode_t mode, const float cutoff_frequency = 0.0f);


private:
    int address_;
    int baudrate_;
    modbus_t *ctx;
    std::string port_;

    eSjcReturnType_t init(void);
    eSjcReturnType_t release(void);
    eSjcReturnType_t getInputRegisters(const int initial_reg, const int num_of_registers, uint16_t *reg_table);
    eSjcReturnType_t setHoldingRegisters(const int initial_reg, const int num_of_registers, uint16_t *reg_table);
    eSjcReturnType_t waitTime_us(const unsigned int wait_time_us);
};
#endif // SJC_INTERFACE_HPP_