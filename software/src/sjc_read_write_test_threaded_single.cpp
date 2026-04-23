/**
 * @file sjc_read_write_test_threaded_single.cpp
 * @brief Threaded read/write test for a single SJC board.
 *
 * Uses SjcRealtimeInterface to evaluate read/write timing on one board.
 *
 * Usage:
 *   ./sjc_read_write_test_threaded_single <port> <address> <frequency Hz>
 *
 * Example:
 *   ./sjc_read_write_test_threaded_single /dev/ttyUSB0 1 50
 */

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

static std::atomic<bool> g_run{true};

void sigint_handler(int)
{
    g_run = false;
}

int main(int argc, char *argv[])
{
    std::signal(SIGINT, sigint_handler);

    if (argc < 4)
    {
        std::cerr << "Usage: ./sjc_read_write_test_threaded_single <port> <address> <frequency Hz>\n";
        std::cerr << "Example: ./sjc_read_write_test_threaded_single /dev/ttyUSB0 1 50\n";
        return -1;
    }

    std::string port = argv[1];
    int address = std::stoi(argv[2]);
    int freq_target = std::stoi(argv[3]);
    constexpr int baudrate = 1'000'000;

    if (freq_target <= 0)
    {
        std::cerr << "Invalid frequency value.\n";
        return -1;
    }
    double Ts_target_ms = 1000.0 / freq_target;

    auto board = std::make_unique<SjcRealtimeInterface>(port, address, baudrate);
    if (board->connect() != SJC_CONNECTED)
    {
        std::cerr << "<x> Failed to connect (" << port << ", addr=" << address << ")\n";
        return -1;
    }
    board->start();

    std::cout << "<ok> SJC board connected successfully.\n";
    std::cout << "Running loop at target " << freq_target << " Hz (" << Ts_target_ms << " ms)\n";

    xSjcControlMeasInfo_t meas{};
    float ref = 0.0f;
    constexpr auto TIMEOUT = std::chrono::milliseconds(10);

    int cycle = 0;
    while (g_run)
    {
        auto loop_start = std::chrono::high_resolution_clock::now();

        // === READ PHASE =====================================================
        auto t0_read = std::chrono::high_resolution_clock::now();

        board->trigger_read();

        bool read_ok = true;
        if (!board->wait_done(TIMEOUT))
        {
            std::cerr << "[Read] Timeout\n";
            read_ok = false;
        }
        else if (board->get_last_read_status() != SJC_RET_OK)
        {
            std::cerr << "[Read] Error (status=" << board->get_last_read_status() << ")\n";
            read_ok = false;
        }

        if (read_ok)
            meas = board->get_runtime_info();

        auto t1_read = std::chrono::high_resolution_clock::now();
        double t_read_ms = std::chrono::duration<double, std::milli>(t1_read - t0_read).count();

        // === WRITE PHASE ====================================================
        auto t0_write = std::chrono::high_resolution_clock::now();

        ref = 5.0f * std::sin(0.1f * cycle);
        board->trigger_write(ref);

        if (!board->wait_done(TIMEOUT))
        {
            std::cerr << "[Write] Timeout\n";
        }
        else if (board->get_last_write_status() != SJC_RET_OK)
        {
            std::cerr << "[Write] Error (status=" << board->get_last_write_status() << ")\n";
        }

        auto t1_write = std::chrono::high_resolution_clock::now();
        double t_write_ms = std::chrono::duration<double, std::milli>(t1_write - t0_write).count();

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
        std::cout << " Cycle: " << cycle++ << "  |  " << port << " (addr=" << address << ")\n";
        std::cout << "----------------------------------------------------------------------------------\n";
        std::cout << "Pos: " << meas.actuator_angular_position
                  << " rad | Vel: " << meas.actuator_angular_velocity
                  << " rad/s | V: " << meas.actuator_voltage
                  << " V | I: " << meas.actuator_current
                  << " A | Ref: " << ref << "\n";
        std::cout << "----------------------------------------------------------------------------------\n";
        std::cout << "Times: Read=" << board->get_last_read_ms()
                  << " ms | Write=" << board->get_last_write_ms()
                  << " ms | Sum=" << (board->get_last_read_ms() + board->get_last_write_ms()) << " ms\n";
        std::cout << "Phase: Read=" << t_read_ms
                  << " ms | Write=" << t_write_ms
                  << " ms | Sum=" << (t_read_ms + t_write_ms) << " ms\n";
        std::cout << "Loop:  Duration=" << cycle_duration_ms
                  << " ms | Sleep=" << sleep_ms
                  << " ms | Jitter=" << jitter
                  << " ms | Freq_real=" << freq_real << " Hz\n";

        std::this_thread::sleep_for(std::chrono::microseconds(
            static_cast<int>(sleep_ms * 1000)));
    }

    board->stop();

    return 0;
}
