/**
 * @file sjc_write_control_freq_mode_ref.cpp
 * @brief Example application to configure and update controller mode,
 *        control loop frequency, and reference on an SJC device via Modbus RTU.
 *
 * This tool connects to an SJC controller board, sets the control mode,
 * control loop frequency, and applies a reference value. It communicates
 * with the device using the `SjcInterface` wrapper (libmodbus).
 *
 * ## Usage:
 * ```
 * ./sjc_write_control_freq_mode_ref <port> <address> <baudrate> <mode> <frequency> <ref>
 * ```
 *
 * - **port**     : COMx (Windows) or /dev/ttyUSBx (Linux)
 * - **address**  : Slave address [1–246]
 * - **baudrate** : Serial baudrate [115200–921600]
 * - **mode**     : Control mode (enum eSjcControlMode_t, 0–N)
 * - **frequency**: Control loop frequency [Hz, 1–1000.0]
 * - **ref**      : Reference setpoint (float, units depend on mode)
 *
 * Example:
 * ```
 * ./sjc_write_control_freq_mode_ref /dev/ttyUSB0 1 115200 2 200 1.57
 * ```
 * → Connects to device at address 1, baudrate 115200, sets PID_SPEED_CONTROL mode
 *   with 200 Hz frequency and reference = 1.57 rad/s.
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

int main(int argc, char *argv[])
{
    std::cout << "\n----------------------------------------------------------------------------------\n";
    std::cout << "  SJC Write Control Mode + Frequency + Reference Tool\n";
    std::cout << "----------------------------------------------------------------------------------\n";
    std::cout << "Program name: " << argv[0] << "\n\n";

    // -------------------------------------------------------------------------
    // Argument parsing and validation
    // -------------------------------------------------------------------------
    if (argc < 7)
    {
        std::cerr << "Usage:\n"
                  << "  ./sjc_write_control_freq_mode_ref <port> <address> <baudrate> <mode> <frequency> <ref>\n\n"
                  << "Arguments:\n"
                  << "  - port      : COMx or /dev/ttyUSBx\n"
                  << "  - address   : 1 to 246\n"
                  << "  - baudrate  : 115200 to 921600\n"
                  << "  - mode      : 0 to N (see eSjcControlMode_t)\n"
                  << "  - frequency : 1 to 1000.0 Hz\n"
                  << "  - ref       : reference (float)\n";
        return -1;
    }

    // Parse arguments
    std::string port = argv[1];
    int address = std::stoi(argv[2]);
    int baudrate = std::stoi(argv[3]);
    eSjcControlMode_t controller_mode_update =
        static_cast<eSjcControlMode_t>(std::stoi(argv[4]));
    float controller_freq = std::stof(argv[5]);
    float controller_ref = std::stof(argv[6]);

    // Print configuration summary
    std::cout << "Connecting with configuration:\n"
              << " - Port     : " << port << "\n"
              << " - Address  : " << address << "\n"
              << " - Baudrate : " << baudrate << "\n"
              << " - Mode     : " << static_cast<int>(controller_mode_update) << "\n"
              << " - Freq     : " << controller_freq << " Hz\n"
              << " - Ref      : " << controller_ref << "\n";

    // -------------------------------------------------------------------------
    // Initialize SJC interface
    // -------------------------------------------------------------------------
    SjcInterface sjc(port, address, baudrate);

    if (sjc.connect() == SJC_CONNECTED)
    {
        std::cout << "\n----------------------------------------------------------------------------------\n";
        std::cout << "  Setting controller configuration...\n";
        std::cout << "----------------------------------------------------------------------------------\n";

        // Human-readable mode
        std::cout << " * Target mode: ";
        switch (controller_mode_update)
        {
            case SJC_OPEN_LOOP_CONTROL:        std::cout << "Open loop"; break;
            case SJC_PID_SPEED_CONTROL:        std::cout << "PID Speed control"; break;
            case SJC_PID_CURRENT_CONTROL:      std::cout << "PID Current control"; break;
            case SJC_LQR_CURRENT_CONTROL:      std::cout << "LQR Current control"; break;
            case SJC_DISCRETE_CURRENT_CONTROL: std::cout << "Discrete Current control"; break;

            // Excitations
            case SJC_SLOW_PRBS_EXCITATION:     std::cout << "Slow PRBS excitation (2.5 Hz)"; break;
            case SJC_MID_PRBS_EXCITATION:      std::cout << "Mid PRBS excitation (5 Hz)"; break;
            case SJC_FAST_PRBS_EXCITATION:     std::cout << "Fast PRBS excitation (10 Hz)"; break;
            case SJC_SINE_PRBS_EXCITATION:     std::cout << "Sine+PRBS excitation (sines 1, 2.5, 5 Hz + PRBS 10 Hz)"; break;
            case SJC_PULSE_FAST_EXCITATION:    std::cout << "Fast pulse excitation (4 Hz)"; break;
            case SJC_PULSE_SLOW_EXCITATION:    std::cout << "Slow pulse excitation (2 Hz)"; break;

            case SJC_ANALOG_CALIBRATION:       std::cout << "Analog calibration"; break;
            default:                           std::cout << "Control disabled"; break;
        }
        std::cout << " | Freq: " << controller_freq << " Hz"
                  << " | Ref: " << controller_ref << "\n";

        bool ok = true;

        // ---------------------------------------------------------------------
        // 1. Disable controller
        // ---------------------------------------------------------------------
        std::cout << " [1/4] Disabling controller... ";
        ok = (sjc.setControllerMode(SJC_CONTROL_DISABLED) == SJC_RET_OK);
        std::cout << (ok ? "OK\n" : "FAIL\n");
        if (!ok) return -1;
        std::this_thread::sleep_for(200ms);

        // ---------------------------------------------------------------------
        // 2. Set frequency
        // ---------------------------------------------------------------------
        std::cout << " [2/4] Setting control frequency: " << controller_freq << " Hz... ";
        ok = (sjc.setControllerUpdateRate(controller_freq) == SJC_RET_OK);
        std::cout << (ok ? "OK\n" : "FAIL\n");
        if (!ok) return -1;
        std::this_thread::sleep_for(200ms);

        // ---------------------------------------------------------------------
        // 3. Set reference
        // ---------------------------------------------------------------------
        std::cout << " [3/4] Setting reference: " << controller_ref << "... ";
        ok = (sjc.setControllerReference(controller_ref) == SJC_RET_OK);
        std::cout << (ok ? "OK\n" : "FAIL\n");
        if (!ok) return -1;
        std::this_thread::sleep_for(200ms);

        // ---------------------------------------------------------------------
        // 4. Apply final mode
        // ---------------------------------------------------------------------
        std::cout << " [4/4] Enabling mode... ";
        ok = (sjc.setControllerMode(controller_mode_update) == SJC_RET_OK);
        std::cout << (ok ? "OK\n" : "FAIL\n");
        if (!ok) return -1;

        std::cout << "\n<✓> Controller configuration applied successfully.\n";
    }
    else
    {
        std::cerr << "\n<✗> Failed to connect to SJC device.\n";
        return -1;
    }

    return 0;
}
