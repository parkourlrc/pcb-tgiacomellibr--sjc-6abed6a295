/**
 * @file sjc_write_control_mode.cpp
 * @brief Example application to configure controller mode
 *        on an SJC device via Modbus RTU.
 *
 * This tool connects to an SJC controller board and updates the control
 * mode. It uses the `SjcInterface` wrapper (libmodbus).
 *
 * ## Usage:
 * ```
 * ./sjc_write_control_mode <port> <address> <baudrate> <mode>
 * ```
 *
 * - **port**     : COMx (Windows) or /dev/ttyUSBx (Linux)
 * - **address**  : Slave address [1–246]
 * - **baudrate** : Serial baudrate [115200–921600]
 * - **mode**     : Control mode (enum eSjcControlMode_t, 0–N)
 *
 * Example:
 * ```
 * ./sjc_write_control_mode /dev/ttyUSB0 1 921600 2
 * ```
 * → Connects to device at address 1, baudrate 921600, sets mode = PID_SPEED_CONTROL.
 */

#include <sjc_interface.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#include "network_configs.h"
#include "helpers.hpp"

using namespace std::chrono_literals;

// -----------------------------------------------------------------------------
// Print human-readable mode
// -----------------------------------------------------------------------------
static void print_mode_name(eSjcControlMode_t mode)
{
    switch (mode)
    {
        case SJC_CONTROL_DISABLED:          std::cout << "Control disabled"; break;
        case SJC_OPEN_LOOP_CONTROL:         std::cout << "Open loop"; break;
        case SJC_PID_SPEED_CONTROL:         std::cout << "PID Speed control"; break;
        case SJC_PID_CURRENT_CONTROL:       std::cout << "PID Current control"; break;
        case SJC_LQR_CURRENT_CONTROL:       std::cout << "LQR Current control"; break;
        case SJC_DISCRETE_CURRENT_CONTROL:  std::cout << "Discrete Current control"; break;

        case SJC_SLOW_PRBS_EXCITATION:      std::cout << "Slow PRBS excitation (2.5 Hz)"; break;
        case SJC_MID_PRBS_EXCITATION:       std::cout << "Mid PRBS excitation (5 Hz)"; break;
        case SJC_FAST_PRBS_EXCITATION:      std::cout << "Fast PRBS excitation (10 Hz)"; break;
        case SJC_SINE_PRBS_EXCITATION:      std::cout << "Sine+PRBS excitation (sines 1, 2.5, 5 Hz + PRBS 10 Hz)"; break;

        case SJC_PULSE_FAST_EXCITATION:     std::cout << "Fast pulse excitation (4 Hz)"; break;
        case SJC_PULSE_SLOW_EXCITATION:     std::cout << "Slow pulse excitation (2 Hz)"; break;

        case SJC_ANALOG_CALIBRATION:        std::cout << "Analog calibration"; break;
        default:                            std::cout << "Unknown"; break;
    }
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    std::cout << "\n----------------------------------------------------------------------------------\n";
    std::cout << "  SJC Write Control Mode Tool\n";
    std::cout << "----------------------------------------------------------------------------------\n";
    std::cout << "Program name: " << argv[0] << "\n\n";

    // -------------------------------------------------------------------------
    // Argument parsing and validation
    // -------------------------------------------------------------------------
    if (argc < 5)
    {
        std::cerr << "Usage:\n"
                  << "  ./sjc_write_control_mode <port> <address> <baudrate> <mode>\n\n"
                  << "Arguments:\n"
                  << "  - port      : COMx or /dev/ttyUSBx\n"
                  << "  - address   : 1 to 246\n"
                  << "  - baudrate  : 115200 to 921600\n"
                  << "  - mode      : 0 to N (see eSjcControlMode_t)\n";
        return -1;
    }

    std::string port = argv[1];
    int address      = std::stoi(argv[2]);
    int baudrate     = std::stoi(argv[3]);
    eSjcControlMode_t mode =
        static_cast<eSjcControlMode_t>(std::stoi(argv[4]));

    std::cout << "Connecting with configuration:\n"
              << " - Port     : " << port << "\n"
              << " - Address  : " << address << "\n"
              << " - Baudrate : " << baudrate << "\n"
              << " - Mode     : " << static_cast<int>(mode) << " (";
    print_mode_name(mode);
    std::cout << ")\n";

    // -------------------------------------------------------------------------
    // Initialize SJC interface
    // -------------------------------------------------------------------------
    SjcInterface sjc(port, address, baudrate);

    if (sjc.connect() == SJC_CONNECTED)
    {
        std::cout << "\n----------------------------------------------------------------------------------\n";
        std::cout << "  Setting controller mode...\n";
        std::cout << "----------------------------------------------------------------------------------\n";

        bool ok = (sjc.setControllerMode(mode) == SJC_RET_OK);
        std::cout << (ok ? " [✓] Mode update successful.\n" : " [✗] Failed to update mode.\n");

        return ok ? 0 : -1;
    }
    else
    {
        std::cerr << "\n<✗> Failed to connect to SJC device.\n";
        return -1;
    }
}
