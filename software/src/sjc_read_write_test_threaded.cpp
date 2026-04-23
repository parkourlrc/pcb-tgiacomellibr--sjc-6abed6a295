/**
 * @file sjc_read_write_test_threaded.cpp
 * @brief Threaded read/write test for multiple SJC boards in parallel.
 *
 * Uses SjcRealtimeInterface worker threads directly — one per joint — to
 * perform concurrent Modbus read and write phases with timing statistics.
 *
 * Usage:
 *   ./sjc_read_write_test_threaded <frequency Hz>
 *
 * Example:
 *   ./sjc_read_write_test_threaded 50
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "sjc_realtime_interface.hpp"

enum eCraneJoint_t
{
    CRANE_X_JOINT = 0,
    CRANE_Y_JOINT,
    CRANE_L_JOINT,
    CRANE_JOINTS_SIZE
};

static std::atomic<bool> g_run{true};

void sigint_handler(int)
{
    g_run = false;
}

int main(int argc, char *argv[])
{
    std::signal(SIGINT, sigint_handler);

    if (argc < 2)
    {
        std::cerr << "Usage: ./sjc_read_write_test_threaded <frequency Hz>\n";
        return -1;
    }

    std::array<std::string, CRANE_JOINTS_SIZE> ports = {
        "/dev/ttyUSB0", // X
        "/dev/ttyUSB1", // Y
        "/dev/ttyUSB2"  // L
    };
    std::array<int, CRANE_JOINTS_SIZE> addresses = {
        1, // USB0 - X
        2, // USB1 - Y
        3  // USB2 - L
    };
    constexpr int baudrate = 1'000'000;

    int freq_target = std::stoi(argv[1]);
    if (freq_target <= 0)
    {
        std::cerr << "Invalid frequency value.\n";
        return -1;
    }
    double Ts_target_ms = 1000.0 / freq_target;

    std::array<std::unique_ptr<SjcRealtimeInterface>, CRANE_JOINTS_SIZE> joints;

    for (int i = 0; i < CRANE_JOINTS_SIZE; i++)
    {
        joints[i] = std::make_unique<SjcRealtimeInterface>(ports[i], addresses[i], baudrate);
        if (joints[i]->connect() != SJC_CONNECTED)
        {
            std::cerr << "<x> Failed to connect joint " << i << " (" << ports[i] << ")\n";
            return -1;
        }
        joints[i]->start();
    }

    std::cout << "<ok> All SJC boards connected successfully.\n";
    std::cout << "Running loop at target " << freq_target << " Hz (" << Ts_target_ms << " ms)\n";

    std::array<xSjcControlMeasInfo_t, CRANE_JOINTS_SIZE> meas{};
    std::array<float, CRANE_JOINTS_SIZE> refs{};
    constexpr auto TIMEOUT = std::chrono::milliseconds(10);

    int cycle = 0;
    while (g_run)
    {
        auto loop_start = std::chrono::high_resolution_clock::now();

        // === READ PHASE =====================================================
        auto t0_read = std::chrono::high_resolution_clock::now();

        for (auto &j : joints)
            j->trigger_read();

        bool read_ok = true;
        for (int i = 0; i < CRANE_JOINTS_SIZE; i++)
        {
            if (!joints[i]->wait_done(TIMEOUT))
            {
                std::cerr << "[Joint " << i << "] Read timeout\n";
                read_ok = false;
            }
            else if (joints[i]->get_last_read_status() != SJC_RET_OK)
            {
                std::cerr << "[Joint " << i << "] Read error (status="
                          << joints[i]->get_last_read_status() << ")\n";
                read_ok = false;
            }
        }

        if (read_ok)
        {
            for (int i = 0; i < CRANE_JOINTS_SIZE; i++)
                meas[i] = joints[i]->get_runtime_info();
        }

        auto t1_read = std::chrono::high_resolution_clock::now();
        double t_read_phase = std::chrono::duration<double, std::milli>(t1_read - t0_read).count();

        // === WRITE PHASE ====================================================
        auto t0_write = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < CRANE_JOINTS_SIZE; i++)
        {
            refs[i] = 5.0f * std::sin(0.1f * cycle + i);
            joints[i]->trigger_write(refs[i]);
        }

        for (int i = 0; i < CRANE_JOINTS_SIZE; i++)
        {
            if (!joints[i]->wait_done(TIMEOUT))
            {
                std::cerr << "[Joint " << i << "] Write timeout\n";
            }
            else if (joints[i]->get_last_write_status() != SJC_RET_OK)
            {
                std::cerr << "[Joint " << i << "] Write error (status="
                          << joints[i]->get_last_write_status() << ")\n";
            }
        }

        auto t1_write = std::chrono::high_resolution_clock::now();
        double t_write_phase = std::chrono::duration<double, std::milli>(t1_write - t0_write).count();

        // === PRINT ==========================================================
        auto loop_end = std::chrono::high_resolution_clock::now();
        double cycle_duration_ms =
            std::chrono::duration<double, std::milli>(loop_end - loop_start).count();
        double sleep_ms = Ts_target_ms - cycle_duration_ms;
        if (sleep_ms < 0)
            sleep_ms = 0.0;

        double freq_real = 1000.0 / (cycle_duration_ms + sleep_ms);
        double jitter = cycle_duration_ms - Ts_target_ms;

        bool is_terminal = isatty(fileno(stdout));
        if (is_terminal)
            std::cout << "\033[2J\033[H";
        std::cout << "\n----------------------------------------------------------------------------------\n";
        std::cout << " Cycle: " << cycle++ << "\n";
        std::cout << "----------------------------------------------------------------------------------\n";

        for (int i = 0; i < CRANE_JOINTS_SIZE; i++)
        {
            auto &m = meas[i];
            std::cout << "[Joint " << i << "] Pos: " << m.actuator_angular_position
                      << " rad | Vel: " << m.actuator_angular_velocity
                      << " rad/s | V: " << m.actuator_voltage
                      << " V | I: " << m.actuator_current
                      << " A | Ref: " << refs[i] << "\n";

            std::cout << "   Times: Read=" << joints[i]->get_last_read_ms()
                      << " ms, Write=" << joints[i]->get_last_write_ms() << " ms\n";
        }

        std::cout << "----------------------------------------------------------------------------------\n";
        std::cout << "Phase Times: Read=" << t_read_phase << " ms | "
                  << "Write=" << t_write_phase << " ms | "
                  << "Sum=" << (t_read_phase + t_write_phase) << " ms\n";
        std::cout << "Loop Timing: Duration=" << cycle_duration_ms
                  << " ms | Sleep=" << sleep_ms
                  << " ms | Jitter=" << jitter
                  << " ms | Freq_real=" << freq_real << " Hz\n";

        std::this_thread::sleep_for(std::chrono::microseconds(
            static_cast<int>(sleep_ms * 1000)));
    }

    for (auto &j : joints)
        j->stop();

    return 0;
}
