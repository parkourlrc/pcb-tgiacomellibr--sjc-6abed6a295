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

    eSjcReturnType_t getDeviceInfo(void);
	eSjcReturnType_t getRuntimeInfo(void);

	eSjcReturnType_t getNetworkParameters(void);
    eSjcReturnType_t setNetworkParameters(void);

	eSjcReturnType_t getActuatorParameters(void);
    eSjcReturnType_t setActuatorParameters(void);

	eSjcReturnType_t getControllerParameters(void);
    eSjcReturnType_t setControllerParameters(void);

	eSjcReturnType_t getSensorsParameters(void);
    eSjcReturnType_t setSensorsParameters(void);

    eSjcReturnType_t setDigitalOutputsStates(void);
        
private:
    modbus_t * ctx;
    int address_;
    int baudrate_;
    std::string port_;

    eSjcReturnType_t init(void);
    eSjcReturnType_t release(void);

    eSjcReturnType_t getInputRegisters(const int initial_reg, const int num_of_registers, uint16_t * reg_table);    
    eSjcReturnType_t getHoldingRegisters(const int initial_reg, const int num_of_registers, uint16_t * reg_table);    
    eSjcReturnType_t setHoldingRegisters(const int initial_reg, const int num_of_registers, uint16_t * reg_table);    
    
};
#endif // _ZLETRIC_EVSE_INTERFACE_H_