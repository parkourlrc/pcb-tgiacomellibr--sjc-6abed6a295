/**
 * @file sjc_write_control_freq.cpp
 * @brief Example application to configure controller loop frequency
 *        on an SJC device via Modbus RTU.
 *
 * This tool connects to an SJC controller board and updates the control
 * loop frequency. It uses the `SjcInterface` wrapper (libmodbus).
 *
 * ## Usage:
 * ```
 * ./sjc_write_control_freq <port> <address> <baudrate> <frequency>
 * ```
 *
 * - **port**     : COMx (Windows) or /dev/ttyUSBx (Linux)
 * - **address**  : Slave address [1–246]
 * - **baudrate** : Serial baudrate [115200–921600]
 * - **frequency**: Control loop frequency [Hz, 1–1000.0]
 *
 * Example:
 * ```
 * ./sjc_write_control_freq /dev/ttyUSB0 1 921600 500
 * ```
 * → Connects to device at address 1, baudrate 921600, sets control frequency to 500 Hz.
 */

#include <sjc_interface.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#include "network_configs.h"
#include "helpers.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    std::cout << "\n----------------------------------------------------------------------------------\n";
    std::cout << "  SJC Write Control Frequency Tool\n";
    std::cout << "----------------------------------------------------------------------------------\n";
    std::cout << "Program name: " << argv[0] << "\n\n";

    // -------------------------------------------------------------------------
    // Argument parsing and validation
    // -------------------------------------------------------------------------
    if (argc < 5)
    {
        std::cerr << "Usage:\n"
                  << "  ./sjc_write_control_freq <port> <address> <baudrate> <frequency>\n\n"
                  << "Arguments:\n"
                  << "  - port      : COMx or /dev/ttyUSBx\n"
                  << "  - address   : 1 to 246\n"
                  << "  - baudrate  : 115200 to 921600\n"
                  << "  - frequency : 1 to 1000.0 Hz\n";
        return -1;
    }

    std::string port   = argv[1];
    int address        = std::stoi(argv[2]);
    int baudrate       = std::stoi(argv[3]);
    float controller_freq = std::stof(argv[4]);

    std::cout << "Connecting with configuration:\n"
              << " - Port     : " << port << "\n"
              << " - Address  : " << address << "\n"
              << " - Baudrate : " << baudrate << "\n"
              << " - Freq     : " << controller_freq << " Hz\n";

    // -------------------------------------------------------------------------
    // Initialize SJC interface
    // -------------------------------------------------------------------------
    SjcInterface sjc(port, address, baudrate);

    if (sjc.connect() == SJC_CONNECTED)
    {
        std::cout << "\n----------------------------------------------------------------------------------\n";
        std::cout << "  Setting controller frequency...\n";
        std::cout << "----------------------------------------------------------------------------------\n";

        bool ok = (sjc.setControllerUpdateRate(controller_freq) == SJC_RET_OK);
        std::cout << (ok ? " [✓] Frequency update successful.\n" : " [✗] Failed to update frequency.\n");

        return ok ? 0 : -1;
    }
    else
    {
        std::cerr << "\n<✗> Failed to connect to SJC device.\n";
        return -1;
    }
}
