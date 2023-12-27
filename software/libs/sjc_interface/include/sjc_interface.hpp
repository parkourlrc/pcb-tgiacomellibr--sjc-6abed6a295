#ifndef _ZLETRIC_EVSE_INTERFACE_H_
#define _ZLETRIC_EVSE_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#include "modbus.h"
#include "data_structures.h"
}
#endif

#include <string>

class SjcInterface  
{
 
public: 

    SjcInterface(const std::string port, const int address, const int baudrate);
    ~SjcInterface(void); 
    
    eSjcReturnType_t connect(void);
    eSjcReturnType_t disconnect(void);

    eSjcReturnType_t getDeviceInfo(xSjcDeviceInfo_t * config);
	eSjcReturnType_t getRuntimeInfo(xSjcRuntimeInfo_t * config);

	eSjcReturnType_t getNetworkParameters(xSjcNetworkConfig_t * config);
    eSjcReturnType_t setNetworkParameters(const xSjcNetworkConfig_t config);

	eSjcReturnType_t getActuatorParameters(xSjcActuatorConfig_t * config);
    eSjcReturnType_t setActuatorParameters(const xSjcActuatorConfig_t config);

	eSjcReturnType_t getControllerParameters(xSjcControllerConfig_t * config);
    eSjcReturnType_t setControllerParameters(const xSjcControllerConfig_t config);

	eSjcReturnType_t getSensorsParameters(xSjcSensorsConfig_t * config);
    eSjcReturnType_t setSensorsParameters(const xSjcSensorsConfig_t config);

    eSjcReturnType_t setDigitalOutputsStates(const xSjcOutputsStates_t config);
               
private:
    modbus_t * ctx;
    int address_;
    int baudrate_;
    std::string port_;

    eSjcReturnType_t init(void);
    eSjcReturnType_t release(void);
    eSjcReturnType_t getInputRegisters(const int initial_reg, const int num_of_registers, uint16_t * reg_table);       
    eSjcReturnType_t setHoldingRegisters(const int initial_reg, const int num_of_registers, uint16_t * reg_table);     
};
#endif // _ZLETRIC_EVSE_INTERFACE_H_