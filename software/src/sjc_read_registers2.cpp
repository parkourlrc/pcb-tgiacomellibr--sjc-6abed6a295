#include <sjc_interface.hpp>
#include <iostream>

#include <stdexcept>
#include <string>
#include <cstring>
#include <chrono>
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#include "helpers.h"
}
#endif

int main(int argc, char *argv[])
{
    std::cout << "\n\n\r----------------------------------------------------------------------------------" << std::endl;
    std::cout << "- SJC debug initialization" << std::endl;
    std::cout << "----------------------------------------------------------------------------------" << std::endl;
    printf("\r\nProgram name is: %s\n", argv[0]);

    if (argc < 3)
    {
        printf("\r\n **** The standard call is: ./sjc_read_test_simp port address baudrate****\r\n");
        printf("\r\n ****  - port: COMX or /dev/ttyUSBX ****\r\n");
        printf("\r\n ****  - address: 1 to 246 ****\r\n");
        printf("\r\n ****  - baudrate: 115200 to 921600 ****\r\n");

        if (argc == 1)
            printf("\nNo Extra Command Line Argument Passed "
                   "Other Than Program Name");

        if (argc >= 2)
        {
            printf("\nNumber Of Arguments Passed: %d", argc);
            printf("\n----Following Are The Command Line "
                   "Arguments Passed----");

            for (int i = 0; i < argc; i++)
            {
                printf("\nargv[%d]: %s", i, argv[i]);
            }
        }

        return -1;
    }

    std::string port = argv[1];
    std::string address = argv[2];
    std::string baudrate = argv[3];

    std::cout << "\r\n\nModbus config:" << std::endl;
    std::cout << " - Port: " << port.c_str() << std::endl;
    std::cout << " - Address: " << std::stoi(address.c_str()) << std::endl;
    std::cout << " - Baudrate: " << std::stoi(baudrate.c_str()) << std::endl;

    SjcInterface Sjc(port.c_str(), std::stoi(address.c_str()), std::stoi(baudrate.c_str()));

    eSjcReturnType_t ret = Sjc.connect();
    
    std::chrono::duration<double, std::nano> dur;
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();

    uint32_t last_time = 0;
    float last_dp = 0;

    if (ret == SJC_CONNECTED)
    {
        do
        {
            static int cycle;

            xSjcRuntimeInfo_t RuntimeInfo;            
            xSjcActuatorConfig_t ActuatorInfo;            

            end = std::chrono::high_resolution_clock::now();
            dur = (end - start);
            start = end;

            system("clear");

            std::cout << "\n\n\r----------------------------------------------------------------------------------" << std::endl;
            std::cout << "- Cycle: " << cycle++ << std::endl;
            std::cout << "----------------------------------------------------------------------------------" << std::endl;
            auto tic = std::chrono::high_resolution_clock::now();            
            
            if (SJC_RET_OK == Sjc.getRuntimeInfo(&RuntimeInfo))
            {
                std::cout << "----------------------------------" << std::endl;
                std::cout << "System info: " << std::endl
                          << " * Uptime: " << RuntimeInfo.uptime;
                std::cout << " ms. - Input Voltage: " << std::to_string(RuntimeInfo.system_voltage) << " V - Error flags: " << RuntimeInfo.error_flags << std::endl;
                std::cout << "----------------------------------" << std::endl;
                std::cout << "Controller:" << std::endl
                          << " * Mode: " << RuntimeInfo.controller_mode << " - Ref.: " << RuntimeInfo.controller_ref << " - Error: " << RuntimeInfo.controller_error << std::endl;
                std::cout << "----------------------------------" << std::endl;
                std::cout << "Displacement sensors: " << std::endl
                          << " * Actuator: " << RuntimeInfo.actuator_displacement << " rd - Aux A: " << RuntimeInfo.aux_A_displacement << " rd - Aux B: " << RuntimeInfo.aux_B_displacement << " rd" << std::endl;
                float dt = (RuntimeInfo.uptime - last_time) / 1000.0f;
                last_time = RuntimeInfo.uptime;
                float ds = RuntimeInfo.actuator_displacement - last_dp;
                last_dp = RuntimeInfo.actuator_displacement;
                float vel = ds / dt;
                float vel_rpm = vel * (180 * 60) / (M_PI * 360);
                std::cout << " * Ds: " << std::to_string(ds) << " rad - Dt: " << std::to_string(dt) << " s" << std::endl;
                std::cout << " * Speed: " << (vel) << " rd/s (" << vel_rpm << " rpm)" << std::endl;

                std::cout << "----------------------------------" << std::endl;
                std::cout << "Actuator efforts: " << std::endl;
                std::cout << " * Voltage: " << std::to_string(RuntimeInfo.actuator_voltage) << " V";
                std::cout << " - Current: " << std::to_string(RuntimeInfo.actuator_current) << " A" << std::endl;
                std::cout << "----------------------------------" << std::endl;
                std::cout << "I/Os: " << std::endl;
                std::cout << " * SwA: " << RuntimeInfo.inputs.switchA;
                std::cout << " - SwB: " << RuntimeInfo.inputs.switchB;
                std::cout << " - Iso. InA: " << RuntimeInfo.inputs.iso_inputA;
                std::cout << " - Iso. InB: " << RuntimeInfo.inputs.iso_inputB;
                std::cout << " - OutA: " << RuntimeInfo.outputs.outputA;
                std::cout << " - OutB: " << RuntimeInfo.outputs.outputB << std::endl;
                std::cout << "----------------------------------" << std::endl;
            }
            else
            {                
                std::cout << "Runtime info aquisition failed! \nPlease check the modbuss slave address..." << std::endl;
                exit(-1);
            }            
            
            if (SJC_RET_OK == Sjc.getActuatorParameters(&ActuatorInfo))
            {
                std::cout << "----------------------------------" << std::endl;
                std::cout << "Actuator parameters: " << std::endl;
                std::cout << " * State: ";
                if (ActuatorInfo.state > 0)
                {
                    std::cout << "Enabled!";
                }
                else
                {
                    std::cout << "Disabled!";
                }
                std::cout << " - Base voltage: " << ActuatorInfo.base_voltage << " V";
                std::cout << " - Current limit: " << ActuatorInfo.current_limit << " A";
                std::cout << " - Voltage limit: " << ActuatorInfo.voltage_limit << " V";
                std::cout << " - Speed limit: " << ActuatorInfo.speed_limit << " rpm" << std::endl;
                std::cout << "----------------------------------" << std::endl;
            }
            else
            {                
                std::cout << "Actuator info aquisition failed! \nPlease check the modbuss slave address..." << std::endl;
                exit(-1);
            }

            auto toc = std::chrono::high_resolution_clock::now();            

            std::cout << "----------------------------------" << std::endl;
            std::chrono::duration<double, std::nano> AcquisitionTs = (toc-tic);
            
            std::cout << " * System Ts:  " << AcquisitionTs.count()*1.0e-6 << " ms." << std::endl;
            std::cout << " * Prev. Cycle duration: " << dur.count()*1.0e-6 << " ms." << std::endl;
            const auto end_cycle = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::nano> dt = (end_cycle - start);
            
            int TsTarget = 20;
            int delta = ((TsTarget*1.0e6) - (dt.count())) * 1.0e-3;
            uint32_t wait = (delta < 0) ? TsTarget*1e3 : delta;
            std::cout << " * Wait time: " << std::to_string(wait / 1000) << " ms" << std::endl;
            std::cout << "\n----------------------------------------------------------------------------------" << std::endl;
            usleep(wait);

        } while (1);
    }

    Sjc.~SjcInterface();

    return 0;
}