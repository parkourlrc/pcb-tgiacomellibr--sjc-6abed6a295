#include <sjc_interface.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include <cstring>
#include <chrono>
#include <cmath>
#include <unistd.h> // for usleep()

#ifdef __cplusplus
extern "C"
{
#include "helpers.hpp"
}
#endif

typedef enum
{
    CRANE_X_JOINT = 0,
    CRANE_Y_JOINT,
    CRANE_L_JOINT,
    CRANE_JOINTS_SIZE
} eCraneJoint_t;

int main(int argc, char *argv[])
{
    std::cout << "\n\n\r----------------------------------------------------------------------------------" << std::endl;
    std::cout << "- SJC debug initialization" << std::endl;
    std::cout << "----------------------------------------------------------------------------------" << std::endl;
    printf("\r\nProgram name is: %s\n", argv[0]);

    if (argc < 4)
    {
        std::cerr << "Usage: ./sjc_read_test <port> <address> <baudrate>\n";
        return -1;
    }

    std::string port = argv[1];
    int address = std::stoi(argv[2]);
    int baudrate = std::stoi(argv[3]);

    std::cout << "\r\n\nModbus config:" << std::endl;
    std::cout << " - Port: " << port << std::endl;
    std::cout << " - Address: " << address << std::endl;
    std::cout << " - Baudrate: " << baudrate << std::endl;

    SjcInterface sjc(port.c_str(), address, baudrate);

    if (sjc.connect() != SJC_CONNECTED)
    {
        std::cerr << "Failed to connect.\n";
        return -1;
    }

    uint32_t last_time = 0;
    float last_dp = 0;
    int cycle = 0;

    while (true)
    {
        auto start = std::chrono::high_resolution_clock::now();
        system("clear");

        std::cout << "\n\n\r----------------------------------------------------------------------------------" << std::endl;
        std::cout << "- Cycle: " << cycle++ << std::endl;
        std::cout << "----------------------------------------------------------------------------------" << std::endl;

        xSjcControlMeasInfo_t meas{};

        if (sjc.getControlMeasInfo(&meas) != SJC_RET_OK)
        {
            std::cerr << "Failed to read measurement info\n";
            break;
        }

        std::cout << "Displacement sensors: \n"
                  << " * Actuator: " << meas.actuator_angular_position
                  << " rad - Swing THX: " << meas.swing_angular_position_thx
                  << " rad - Swing THY: " << meas.swing_angular_position_thy
                  << " rad" << std::endl;

        float dt = (meas.uptime - last_time) / 1000.0f;
        float ds = meas.actuator_angular_position - last_dp;
        last_time = meas.uptime;
        last_dp = meas.actuator_angular_position;

        float vel = ds / dt;
        float vel_rpm = vel * (180.0f * 60.0f) / (M_PI * 360.0f);

        std::cout << " * Δs: " << ds << " rad - Δt: " << dt << " s\n";
        std::cout << " * Speed: " << vel << " rad/s (" << vel_rpm << " rpm)" << std::endl;

        std::cout << "Actuator efforts:\n"
                  << " * Voltage: " << meas.actuator_voltage << " V\n"
                  << " * Current: " << meas.actuator_current << " A\n";

        if (sjc.setControllerOperatingMode(SJC_OPEN_LOOP_CONTROL) != SJC_RET_OK ||
            sjc.setControllerReference(0.0f) != SJC_RET_OK)
        {
            std::cerr << "Controller configuration failed.\n";
            break;
        }

        auto end = std::chrono::high_resolution_clock::now();
        uint32_t cycle_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        int Ts_target = 50; // ms
        int delay_us = std::max(0, (Ts_target - (int)cycle_duration) * 1000);

        std::cout << "----------------------------------" << std::endl;
        std::cout << " * Cycle duration: " << cycle_duration << " ms" << std::endl;
        std::cout << " * Wait time: " << delay_us / 1000 << " ms" << std::endl;
        std::cout << "\n----------------------------------------------------------------------------------" << std::endl;

        usleep(delay_us);
    }

    return 0;
}
