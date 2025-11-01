/**
 * @file sjc_write_reference.cpp
 * @brief Example application to configure controller reference
 *        on an SJC device via Modbus RTU.
 *
 * This tool connects to an SJC controller board and updates only
 * the controller reference value. It uses the `SjcInterface` wrapper.
 *
 * ## Usage:
 * ```
 * ./sjc_write_reference <port> <address> <baudrate> <ref>
 * ```
 *
 * - **port**     : COMx (Windows) or /dev/ttyUSBx (Linux)
 * - **address**  : Slave address [1–246]
 * - **baudrate** : Serial baudrate [115200–921600]
 * - **ref**      : Reference setpoint (float, units depend on mode)
 *
 * Example:
 * ```
 * ./sjc_write_reference /dev/ttyUSB0 1 921600 1.57
 * ```
 * → Connects to device at address 1, baudrate 921600, sets reference = 1.57.
 */

#include <sjc_interface.hpp>
#include <iostream>
#include <string>

#include "network_configs.h"
#include "helpers.hpp"

int main(int argc, char *argv[])
{
    std::cout << "\n----------------------------------------------------------------------------------\n";
    std::cout << "  SJC Write Reference Tool\n";
    std::cout << "----------------------------------------------------------------------------------\n";
    std::cout << "Program name: " << argv[0] << "\n\n";

    // -------------------------------------------------------------------------
    // Argument parsing and validation
    // -------------------------------------------------------------------------
    if (argc < 5)
    {
        std::cerr << "Usage:\n"
                  << "  ./sjc_write_reference <port> <address> <baudrate> <ref>\n\n"
                  << "Arguments:\n"
                  << "  - port      : COMx or /dev/ttyUSBx\n"
                  << "  - address   : 1 to 246\n"
                  << "  - baudrate  : 115200 to 921600\n"
                  << "  - ref       : reference (float)\n";
        return -1;
    }

    std::string port  = argv[1];
    int address       = std::stoi(argv[2]);
    int baudrate      = std::stoi(argv[3]);
    float controller_ref = std::stof(argv[4]);

    std::cout << "Connecting with configuration:\n"
              << " - Port     : " << port << "\n"
              << " - Address  : " << address << "\n"
              << " - Baudrate : " << baudrate << "\n"
              << " - Ref      : " << controller_ref << "\n";

    // -------------------------------------------------------------------------
    // Initialize SJC interface
    // -------------------------------------------------------------------------
    SjcInterface sjc(port, address, baudrate);

    if (sjc.connect() == SJC_CONNECTED)
    {
        std::cout << "\n----------------------------------------------------------------------------------\n";
        std::cout << "  Setting controller reference...\n";
        std::cout << "----------------------------------------------------------------------------------\n";

        bool ok = (sjc.setControllerReference(controller_ref) == SJC_RET_OK);
        std::cout << (ok ? " [✓] Reference update successful.\n" : " [✗] Failed to update reference.\n");

        return ok ? 0 : -1;
    }
    else
    {
        std::cerr << "\n<✗> Failed to connect to SJC device.\n";
        return -1;
    }
}
