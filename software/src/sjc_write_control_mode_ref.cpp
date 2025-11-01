/**
 * @file sjc_write_control_mode_ref.cpp
 * @brief Example application to configure control mode and reference
 *        on an SJC device via Modbus RTU.
 *
 * This tool connects to an SJC controller board, sets the control mode
 * and applies a reference value. It communicates with the device using
 * the `SjcInterface` wrapper (libmodbus).
 *
 * ## Usage:
 * ```
 * ./sjc_write_mode_ref <port> <address> <baudrate> <mode> <ref>
 * ```
 *
 * - **port**     : COMx (Windows) or /dev/ttyUSBx (Linux)
 * - **address**  : Slave address [1–246]
 * - **baudrate** : Serial baudrate [115200–921600]
 * - **mode**     : Control mode (enum eSjcControlMode_t, 0–…)  
 * - **ref**      : Reference setpoint (float, units depend on mode)
 *
 * Example:
 * ```
 * ./sjc_write_mode_ref /dev/ttyUSB0 1 115200 2 1.57
 * ```
 * → Connects to device at address 1, baudrate 115200, sets PID_SPEED_CONTROL mode
 *   with reference = 1.57 rad/s.
 */

#include <sjc_interface.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <chrono>

#include "network_configs.h"
#include "helpers.hpp"

using namespace std::chrono_literals;

// -----------------------------------------------------------------------------
// Human-readable conversion of control modes
// -----------------------------------------------------------------------------
static std::string modeToString(eSjcControlMode_t mode)
{
    switch (mode)
    {
        case SJC_OPEN_LOOP_CONTROL:        return "Open loop";
        case SJC_PID_SPEED_CONTROL:        return "PID Speed control";
        case SJC_PID_CURRENT_CONTROL:      return "PID Current control";
        case SJC_LQR_CURRENT_CONTROL:      return "LQR Current control";
        case SJC_DISCRETE_CURRENT_CONTROL: return "Discrete Current control";

        case SJC_SLOW_PRBS_EXCITATION:     return "Slow PRBS excitation (2.5 Hz)";
        case SJC_MID_PRBS_EXCITATION:      return "Mid PRBS excitation (5 Hz)";
        case SJC_FAST_PRBS_EXCITATION:     return "Fast PRBS excitation (10 Hz)";
        case SJC_SINE_PRBS_EXCITATION:     return "Sine+PRBS excitation (sines 1, 2.5, 5 Hz + PRBS 10 Hz)";

        case SJC_PULSE_FAST_EXCITATION:    return "Fast pulse excitation (4 Hz)";
        case SJC_PULSE_SLOW_EXCITATION:    return "Slow pulse excitation (2 Hz)";

        case SJC_ANALOG_CALIBRATION:       return "Analog calibration";
        case SJC_CONTROL_DISABLED:         return "Control disabled";
        default:                           return "Unknown";
    }
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    std::cout << "\n----------------------------------------------------------------------------------\n";
    std::cout << "  SJC Write Mode + Reference Tool\n";
    std::cout << "----------------------------------------------------------------------------------\n";
    std::cout << "Program name: " << argv[0] << "\n\n";

    // -------------------------------------------------------------------------
    // Argument parsing and validation
    // -------------------------------------------------------------------------
    if (argc < 6)
    {
        std::cerr << "Usage:\n"
                  << "  ./sjc_write_mode_ref <port> <address> <baudrate> <mode> <ref>\n\n"
                  << "Arguments:\n"
                  << "  - port      : COMx or /dev/ttyUSBx\n"
                  << "  - address   : 1 to 246\n"
                  << "  - baudrate  : 115200 to 921600\n"
                  << "  - mode      : 0 to N (see eSjcControlMode_t)\n"
                  << "  - ref       : reference (float)\n";
        return -1;
    }

    // Parse arguments
    std::string port = argv[1];
    int address = std::stoi(argv[2]);
    int baudrate = std::stoi(argv[3]);
    eSjcControlMode_t controller_mode_update =
        static_cast<eSjcControlMode_t>(std::stoi(argv[4]));
    float controller_ref = std::stof(argv[5]);

    // Print configuration summary
    std::cout << "Connecting with configuration:\n"
              << " - Port     : " << port << "\n"
              << " - Address  : " << address << "\n"
              << " - Baudrate : " << baudrate << "\n"
              << " - Mode     : " << static_cast<int>(controller_mode_update)
              << " (" << modeToString(controller_mode_update) << ")\n"
              << " - Ref      : " << controller_ref << "\n";

    // -------------------------------------------------------------------------
    // Initialize SJC interface
    // -------------------------------------------------------------------------
    SjcInterface sjc(port, address, baudrate);

    if (sjc.connect() == SJC_CONNECTED)
    {
        std::cout << "\n----------------------------------------------------------------------------------\n";
        std::cout << "  Setting controller mode + reference...\n";
        std::cout << "----------------------------------------------------------------------------------\n";

        bool ok = true;

        // ---------------------------------------------------------------------
        // 1. Disable controller
        // ---------------------------------------------------------------------
        std::cout << " [1/3] Disabling controller... ";
        ok = (sjc.setControllerMode(SJC_CONTROL_DISABLED) == SJC_RET_OK);
        std::cout << (ok ? "OK\n" : "FAIL\n");
        if (!ok) return -1;
        std::this_thread::sleep_for(200ms);

        // ---------------------------------------------------------------------
        // 2. Set reference
        // ---------------------------------------------------------------------
        std::cout << " [2/3] Setting reference: " << controller_ref << "... ";
        ok = (sjc.setControllerReference(controller_ref) == SJC_RET_OK);
        std::cout << (ok ? "OK\n" : "FAIL\n");
        if (!ok) return -1;
        std::this_thread::sleep_for(200ms);

        // ---------------------------------------------------------------------
        // 3. Apply final mode
        // ---------------------------------------------------------------------
        std::cout << " [3/3] Enabling mode (" << modeToString(controller_mode_update) << ")... ";
        ok = (sjc.setControllerMode(controller_mode_update) == SJC_RET_OK);
        std::cout << (ok ? "OK\n" : "FAIL\n");
        if (!ok) return -1;

        std::cout << "\n<✓> Mode + Reference configuration applied successfully.\n";
    }
    else
    {
        std::cerr << "\n<✗> Failed to connect to SJC device.\n";
        return -1;
    }

    return 0;
}
