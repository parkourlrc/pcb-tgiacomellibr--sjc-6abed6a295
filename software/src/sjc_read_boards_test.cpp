/**
 * @file sjc_read_test.cpp
 * @brief Example program to connect to an SJC controller, read measurements,
 *        compute derived velocity, and configure controller parameters in a loop.
 *
 * Usage:
 *   ./sjc_read_test <port> <address> <baudrate>
 *
 * Example:
 *   ./sjc_read_test /dev/ttyUSB0 1 921600
 */

#include <chrono>
#include <cmath>
#include <iostream>
#include <sjc_interface.hpp>
#include <stdexcept>
#include <string>
#include <unistd.h> // for usleep()

#include "helpers.hpp" // already C++-protected

// Crane joints index (placeholder for future extension)
typedef enum
{
    CRANE_X_JOINT = 0,
    CRANE_Y_JOINT,
    CRANE_L_JOINT,
    CRANE_JOINTS_SIZE
} eCraneJoint_t;

int main(int argc, char *argv[])
{
    std::cout << "\n----------------------------------------------------------------------------------\n";
    std::cout << "  SJC Read Test Initialization\n";
    std::cout << "----------------------------------------------------------------------------------\n";
    std::cout << "Program name: " << argv[0] << "\n";

    // -------------------------------------------------------------------------
    // Argument parsing
    // -------------------------------------------------------------------------
    if (argc < 4)
    {
        std::cerr << "Usage: ./sjc_read_test <port> <address> <baudrate>\n";
        return -1;
    }

    std::string port = argv[1];
    int address = std::stoi(argv[2]);
    int baudrate = std::stoi(argv[3]);

    std::cout << "\nModbus configuration:\n"
              << " - Port    : " << port << "\n"
              << " - Address : " << address << "\n"
              << " - Baudrate: " << baudrate << "\n";

    // -------------------------------------------------------------------------
    // Initialize SJC interface
    // -------------------------------------------------------------------------
    SjcInterface sjc(port, address, baudrate);

    if (sjc.connect() != SJC_CONNECTED)
    {
        std::cerr << "<✗> Failed to connect to SJC device.\n";
        return -1;
    }

    // -------------------------------------------------------------------------
    // Configure sensors BEFORE entering main loop
    // -------------------------------------------------------------------------
    xSjcSensorsConfig_t sensor_cfg{};
    sensor_cfg.actuator_encoder_resolution_ppr = 32768; // motor joint encoder
    sensor_cfg.swing_thx_encoder_resolution_ppr = 8192; // swing THX encoder
    sensor_cfg.swing_thy_encoder_resolution_ppr = 8192; // swing THY encoder

    if (sjc.setSensorsParameters(sensor_cfg) != SJC_RET_OK)
    {
        std::cerr << "<✗> Failed to configure sensor parameters\n";
        return -1;
    }
    else
    {
        std::cout << "<✓> Sensor configuration written successfully\n";
    }

    uint32_t last_time = 0;
    float last_pos = 0.0f;
    int cycle = 0;

    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------
    while (true)
    {
        auto start = std::chrono::high_resolution_clock::now();

        // Clear console for readability (Linux)
        system("clear");

        std::cout << "\n----------------------------------------------------------------------------------\n";
        std::cout << " Cycle: " << cycle++ << "\n";
        std::cout << "----------------------------------------------------------------------------------\n";

        xSjcControlMeasInfo_t meas{};
        xSjcSensorsConfig_t sensor_cfg{};

        if (sjc.getControlMeasInfo(&meas) != SJC_RET_OK)
        {
            std::cerr << "<✗> Failed to read measurement info\n";
            break;
        }

        if (sjc.getSensorsParameters(&sensor_cfg) != SJC_RET_OK)
        {
            std::cerr << "<✗> Failed to read sensor configuration\n";
            break;
        }

        // ---------------------------------------------------------------------
        // Sensor configuration
        // ---------------------------------------------------------------------
        std::cout << "Sensor configuration:\n"
                  << " * Actuator encoder resolution : "
                  << sensor_cfg.actuator_encoder_resolution_ppr << " ppr\n"
                  << " * Swing θx encoder resolution : "
                  << sensor_cfg.swing_thx_encoder_resolution_ppr << " ppr\n"
                  << " * Swing θy encoder resolution : "
                  << sensor_cfg.swing_thy_encoder_resolution_ppr << " ppr\n";

        // ---------------------------------------------------------------------
        // Sensor values
        // ---------------------------------------------------------------------
        std::cout << "Displacement sensors:\n"
                  << " * Actuator: " << meas.actuator_angular_position << " rad\n"
                  << " * Swing THX: " << meas.swing_angular_position_thx << " rad\n"
                  << " * Swing THY: " << meas.swing_angular_position_thy << " rad\n";

        // ---------------------------------------------------------------------
        // Derived velocity
        // ---------------------------------------------------------------------
        float dt = (meas.uptime >= last_time) ? (meas.uptime - last_time) / 1000.0f : 0.0f;
        float ds = meas.actuator_angular_position - last_pos;
        last_time = meas.uptime;
        last_pos = meas.actuator_angular_position;

        float vel = (dt > 1e-6f) ? ds / dt : 0.0f;
        float vel_rpm = vel * 60.0f / (2.0f * M_PI);

        std::cout << "Δs: " << ds << " rad | Δt: " << dt << " s\n"
                  << "Velocity: " << vel << " rad/s (" << vel_rpm << " rpm)\n";

        // ---------------------------------------------------------------------
        // Electrical values
        // ---------------------------------------------------------------------
        std::cout << "Actuator efforts:\n"
                  << " * Voltage: " << meas.actuator_voltage << " V\n"
                  << " * Current: " << meas.actuator_current << " A\n";

        // ---------------------------------------------------------------------
        // Timing control
        // ---------------------------------------------------------------------
        auto end = std::chrono::high_resolution_clock::now();
        uint32_t cycle_duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        int Ts_target = 20; // ms (50 Hz loop)
        int delay_us = std::max(0, (Ts_target - static_cast<int>(cycle_duration)) * 1000);

        std::cout << "----------------------------------\n";
        std::cout << " Cycle duration: " << cycle_duration << " ms\n";
        std::cout << " Wait time     : " << delay_us / 1000 << " ms\n";
        std::cout << "----------------------------------------------------------------------------------\n";

        usleep(delay_us);
    }

    return 0;
}
