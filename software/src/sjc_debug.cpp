#include <sjc_interface.hpp>
#include <iostream>

#include <stdexcept>
#include <string>
#include <chrono>

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

            const auto start = std::chrono::steady_clock::now();

            if (SJC_RET_OK == Sjc.getRuntimeInfo())
            // if (SJC_RET_OK == Sjc.getDeviceInfo())
            {
                const auto end = std::chrono::steady_clock::now();

                system("clear");

                std::cout << "\n\n\r----------------------------------------------------------------------------------" << std::endl;
                std::cout << "- Cycle: " << cycle++ << std::endl;
                std::cout << "----------------------------------------------------------------------------------" << std::endl;

                // Modbus Address
                uint32_t net_addr = std::stoi(address.c_str());
                std::cout << "\n\rNetwork Address: " << net_addr << std::endl;
                const std::chrono::duration<double> diff = end - start;               

                std::cout << "It took me " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms.";

                // Evse.setChargeDisable();
                usleep(100e3);
            }
            else
            {
                usleep(10e6);
                std::cout << "Communication failed! \nPlease check the modbuss slave address..." << std::endl;
            }

            std::cout << "\n----------------------------------------------------------------------------------" << std::endl;

        } while (1);
    }
    Sjc.~SjcInterface();

    return 0;
}
