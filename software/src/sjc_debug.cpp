#include <sjc_interface.hpp>
#include <iostream>

#include <stdexcept>
#include <string>
#include <chrono>

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
        printf("\r\n **** The standard call is: ./sjc_debug port address baudrate****\r\n");
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

    if (ret == SJC_CONNECTED)
    {
        do
        {
            static int cycle;

            xSjcRuntimeInfo_t RuntimeInfo;
            xSjcDeviceInfo_t DeviceInfo;
            xSjcNetworkInfo_t NetworkInfo;

            system("clear");
            const auto start = std::chrono::steady_clock::now();

            std::cout << "\n\n\r----------------------------------------------------------------------------------" << std::endl;
            std::cout << "- Cycle: " << cycle++ << std::endl;
            std::cout << "----------------------------------------------------------------------------------" << std::endl;
            if (SJC_RET_OK == Sjc.getDeviceInfo(&DeviceInfo))
            {
                std::cout << "----------------------------------" << std::endl;
                std::cout << "Fw version: ";
                std::cout.operator<<(DeviceInfo.firmware_ver.major);  
                std::cout << ".";
                std::cout.operator<<(DeviceInfo.firmware_ver.minor);  
                std::cout << ".";
                std::cout.operator<<(DeviceInfo.firmware_ver.patch);  
                std::cout << " - Hw version: ";
                std::cout.operator<<(DeviceInfo.hardware_ver.major);  
                std::cout << ".";
                std::cout.operator<<(DeviceInfo.hardware_ver.minor);  
                std::cout << std::endl;
            }
            else
            {
                usleep(10e6);
                std::cout << "Device info aquisition failed! \nPlease check the modbuss slave address..." << std::endl;
            }

            if (SJC_RET_OK == Sjc.getNetworkParameters(&NetworkInfo))
            {
                std::cout << "----------------------------------" << std::endl;
                std::cout << "Address: ";
                std::cout.operator<<(NetworkInfo.address);  
                std::cout << " - Parity: " << NetworkInfo.parity;
                std::cout << " - Baudrate: ";
                std::cout.operator<<(NetworkInfo.baudrate);  
                std::cout << " bps" << std::endl;
            }
            else
            {
                usleep(10e6);
                std::cout << "Network info aquisition failed! \nPlease check the modbuss slave address..." << std::endl;
            }

            if (SJC_RET_OK == Sjc.getRuntimeInfo(&RuntimeInfo))
            {
                std::cout << "----------------------------------" << std::endl;
                std::cout << "uptime: " << RuntimeInfo.uptime << " ms." << std::endl;
                std::cout << "Input Voltage: " << RuntimeInfo.system_voltage << " V" << std::endl;
                std::cout << "Error flags: " << RuntimeInfo.error_flags << std::endl;
                std::cout << "----------------------------------" << std::endl;
                std::cout << "Ctrl. Mode: " << RuntimeInfo.controller_mode << std::endl;
                std::cout << "Ctrl. Ref.: " << RuntimeInfo.controller_ref << std::endl;
                std::cout << "Ctrl. Error: " << RuntimeInfo.controller_error << std::endl;
                std::cout << "----------------------------------" << std::endl;
                std::cout << "Jnt. Dp.: " << RuntimeInfo.actuator_displacement << " rd" << std::endl;
                std::cout << "Aux A. Dp.: " << RuntimeInfo.aux_A_displacement << " rd" << std::endl;
                std::cout << "Aux B. Dp.: " << RuntimeInfo.aux_B_displacement << " rd" << std::endl;
                std::cout << "----------------------------------" << std::endl;
                std::cout << "Jnt. Voltage: " << RuntimeInfo.actuator_output_voltage << " V" << std::endl;
                std::cout << "Jnt. Current: " << RuntimeInfo.actuator_current << " A" << std::endl;
                std::cout << "----------------------------------" << std::endl;
                std::cout << "Dig. Inputs: " << RuntimeInfo.digital_inputs << std::endl;
                std::cout << "Dig. Outputs: " << RuntimeInfo.digital_outputs << std::endl;
                std::cout << "----------------------------------" << std::endl;
            }
            else
            {
                usleep(10e6);
                std::cout << "Runtime info aquisition failed! \nPlease check the modbuss slave address..." << std::endl;
            }

            const auto end = std::chrono::steady_clock::now();
            const std::chrono::duration<double> diff = end - start;
            std::cout << "Com. time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms." << std::endl;
            std::cout << "----------------------------------" << std::endl;
            usleep(1e6);
            std::cout << "\n----------------------------------------------------------------------------------" << std::endl;
        }
        while (1);
    }
    Sjc.~SjcInterface();

    return 0;
}
