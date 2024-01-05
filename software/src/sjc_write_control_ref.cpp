#include <sjc_interface.hpp>
#include <iostream>

#include <stdexcept>
#include <string>
#include <cstring>
#include <chrono>

#ifdef __cplusplus
extern "C"
{
#include "helpers.h"
#include "network_configs.h"
}
#endif

int main(int argc, char *argv[])
{
    std::cout << "\n\n\r----------------------------------------------------------------------------------" << std::endl;
    std::cout << "- SJC debug initialization" << std::endl;
    std::cout << "----------------------------------------------------------------------------------" << std::endl;
    printf("\r\nProgram name is: %s\n", argv[0]);

    if (argc < 6)
    {
        printf("\r\n **** The standard call is: ./sjc_write_control_ref port address baudrate mode ref****\r\n");
        printf("\r\n ****  - port: COMX or /dev/ttyUSBX ****\r\n");
        printf("\r\n ****  - address: 1 to 246 ****\r\n");
        printf("\r\n ****  - baudrate: 115200 to 921600 ****\r\n");
        printf("\r\n ****  - mode: 0 to 3 ****\r\n");
        printf("\r\n ****  - ref: decimal reference ****\r\n");

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
    std::string mode = argv[4];
    std::string ref = argv[5];

    std::cout << "\r\n\nModbus config:" << std::endl;
    std::cout << " - Port: " << port.c_str() << std::endl;
    std::cout << " - Address: " << std::stoi(address.c_str()) << std::endl;
    std::cout << " - Baudrate: " << std::stoi(baudrate.c_str()) << std::endl;
    std::cout << " - Mode: " << std::stof(mode.c_str()) << std::endl;
    std::cout << " - Reference: " << std::stof(ref.c_str()) << std::endl;

    SjcInterface Sjc(port.c_str(), std::stoi(address.c_str()), std::stoi(baudrate.c_str()));

    eSjcReturnType_t ret = Sjc.connect();

    if (ret == SJC_CONNECTED)
    {
        // do
        {
            static int cycle;
            
            xSjcControllerConfig_t xControllerConfig;

            system("clear");

            std::cout << "\n\n\r----------------------------------------------------------------------------------" << std::endl;
            std::cout << "- Cycle: " << cycle++ << std::endl;
            std::cout << "----------------------------------------------------------------------------------" << std::endl;

            xControllerConfig.controller_mode_update = std::stof(mode.c_str());
            xControllerConfig.controller_ref = std::stof(ref.c_str());

            std::cout << "Controller config: " << std::endl;
            std::cout << " * Mode: ";
            switch (xControllerConfig.controller_mode_update)
            {
            case SJC_OPEN_LOOP_CONTROL:
                std::cout << "Open loop";
                break;
            case SJC_PRBS_EXCITATION:
                std::cout << "PRBS Excitation";
                break;
            case SJC_SPEED_CONTROL:
                std::cout << "Speed control";
                break;
            case SJC_TORQUE_CONTROL:
                std::cout << "Torque control";
                break;

            default: // SJC_CONTROL_DISABLED
                std::cout << "Control disabled";
                break;
            }
            std::cout << " - Ref: " << xControllerConfig.controller_ref << std::endl;

            if (SJC_RET_OK == Sjc.setControllerConfig(xControllerConfig))
            {
                std::cout << " <|> Controller configs setted" << std::endl;
                usleep(500e3);
            }
            else
            {
                usleep(10e6);
                std::cout << " <|> Controller configs not set! \nPlease check the modbuss slave address..." << std::endl;
            }
            std::cout << "\n----------------------------------------------------------------------------------" << std::endl;
        }
    }

    // while (1);

    Sjc.~SjcInterface();

    return 0;
}
