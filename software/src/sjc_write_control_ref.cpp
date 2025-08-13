#include <sjc_interface.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include <cstring>
#include <chrono>

#ifdef __cplusplus
extern "C"
{
#include "helpers.hpp"
#include "network_configs.hpp"
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
        printf("\r\n ****  - mode: 0 to 8 ****\r\n");
        printf("\r\n ****  - ref: decimal reference ****\r\n");

        if (argc == 1)
            printf("\nNo Extra Command Line Argument Passed Other Than Program Name");

        if (argc >= 2)
        {
            printf("\nNumber Of Arguments Passed: %d", argc);
            printf("\n----Following Are The Command Line Arguments Passed----");

            for (int i = 0; i < argc; i++)
                printf("\nargv[%d]: %s", i, argv[i]);
        }

        return -1;
    }

    std::string port = argv[1];
    int address = std::stoi(argv[2]);
    int baudrate = std::stoi(argv[3]);
    eSjcControlMode_t controller_mode_update = static_cast<eSjcControlMode_t>(std::stoi(argv[4]));
    float controller_ref = std::stof(argv[5]);

    std::cout << "\r\n\nModbus config:" << std::endl;
    std::cout << " - Port: " << port << std::endl;
    std::cout << " - Address: " << address << std::endl;
    std::cout << " - Baudrate: " << baudrate << std::endl;
    std::cout << " - Mode: " << static_cast<int>(controller_mode_update) << std::endl;
    std::cout << " - Reference: " << controller_ref << std::endl;

    SjcInterface sjc(port.c_str(), address, baudrate);

    if (sjc.connect() == SJC_CONNECTED)
    {
        std::cout << "\n\n\r----------------------------------------------------------------------------------" << std::endl;
        std::cout << "- Setting controller configuration..." << std::endl;
        std::cout << "----------------------------------------------------------------------------------" << std::endl;

        std::cout << " * Mode: ";
        switch (controller_mode_update)
        {
            case SJC_OPEN_LOOP_CONTROL:         std::cout << "Open loop"; break;
            case SJC_PRBS_EXCITATION:           std::cout << "PRBS Excitation"; break;
            case SJC_SPEED_CONTROL:             std::cout << "Speed control"; break;
            case SJC_CURRENT_CONTROL:           std::cout << "Torque control"; break;
            case SJC_TRIANG_EXCITATION:         std::cout << "Triangular signal excitation"; break;
            case SJC_PULSE_FAST_EXCITATION:     std::cout << "Fast pulse signal excitation"; break;
            case SJC_PULSE_SLOW_EXCITATION:     std::cout << "Slow pulse signal excitation"; break;
            case SJC_ANALOG_CALIBRATION:        std::cout << "Analog calibration"; break;
            default:                            std::cout << "Control disabled"; break;
        }
        std::cout << " - Ref: " << controller_ref << std::endl;

        const bool mode_ok = (sjc.setControllerOperatingMode(controller_mode_update) == SJC_RET_OK);
        const bool ref_ok = (sjc.setControllerReference(controller_ref) == SJC_RET_OK);

        if (mode_ok && ref_ok)
        {
            std::cout << "\n<✓> Controller configs set successfully." << std::endl;
        }
        else
        {
            std::cerr << "\n<✗> Failed to set controller configs. Please check the Modbus slave address or connection." << std::endl;
        }
    }
    else
    {
        std::cerr << "\n<✗> Failed to connect to device." << std::endl;
    }

    return 0;
}
