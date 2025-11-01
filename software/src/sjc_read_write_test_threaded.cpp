#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "helpers.hpp"
#include "sjc_realtime_interface.hpp"

enum eCraneJoint_t
{
    CRANE_X_JOINT = 0,
    CRANE_Y_JOINT,
    CRANE_L_JOINT,
    CRANE_JOINTS_SIZE
};

struct WorkerContext
{
    std::unique_ptr<SjcRealtimeInterface> iface;
    std::thread th;
    std::atomic<bool> run{true};

    std::mutex mtx;
    std::condition_variable cv;

    std::atomic<bool> do_read{false};
    std::atomic<bool> do_write{false};
    std::atomic<bool> done{false};

    xSjcControlMeasInfo_t meas{};
    float ref{0.0f};

    double t_read{0.0}, t_write{0.0}, t_sum{0.0};
};

static std::atomic<bool> g_run{true};

void sigint_handler(int)
{
    g_run = false;
}

void worker_loop(WorkerContext *ctx)
{
    while (ctx->run)
    {
        std::unique_lock<std::mutex> lk(ctx->mtx);
        ctx->cv.wait(lk, [&]
                     { return ctx->do_read || ctx->do_write || !ctx->run; });
        if (!ctx->run)
            break;

        if (ctx->do_read)
        {
            auto t0 = std::chrono::high_resolution_clock::now();
            ctx->iface->trigger_read();
            ctx->iface->wait_done();
            ctx->meas = ctx->iface->get_runtime_info();
            auto t1 = std::chrono::high_resolution_clock::now();
            ctx->t_read = std::chrono::duration<double, std::milli>(t1 - t0).count();
            ctx->do_read = false;
        }
        else if (ctx->do_write)
        {
            auto t0 = std::chrono::high_resolution_clock::now();
            ctx->iface->trigger_write(ctx->ref);
            ctx->iface->wait_done();
            auto t1 = std::chrono::high_resolution_clock::now();
            ctx->t_write = std::chrono::duration<double, std::milli>(t1 - t0).count();
            ctx->do_write = false;
        }

        ctx->t_sum = ctx->t_read + ctx->t_write;
        ctx->done = true;
        lk.unlock();
        ctx->cv.notify_all();
    }
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
    constexpr int baudrate = 1'000'000;
    constexpr int address = 1;

    int freq_target = std::stoi(argv[1]);
    if (freq_target <= 0)
    {
        std::cerr << "Invalid frequency value.\n";
        return -1;
    }
    double Ts_target_ms = 1000.0 / freq_target;

    std::vector<WorkerContext> workers(CRANE_JOINTS_SIZE);

    for (int i = 0; i < CRANE_JOINTS_SIZE; i++)
    {
        workers[i].iface = std::make_unique<SjcRealtimeInterface>(ports[i], address, baudrate);
        if (workers[i].iface->connect() != SJC_CONNECTED)
        {
            std::cerr << "<✗> Failed to connect joint " << i << " (" << ports[i] << ")\n";
            return -1;
        }
        workers[i].iface->start();
        workers[i].th = std::thread(worker_loop, &workers[i]);
    }

    std::cout << "<✓> All SJC boards connected successfully.\n";
    std::cout << "Running loop at target " << freq_target << " Hz (" << Ts_target_ms << " ms)\n";

    int cycle = 0;
    while (g_run)
    {
        auto loop_start = std::chrono::high_resolution_clock::now();

        // === READ PHASE =====================================================
        auto t0_read = std::chrono::high_resolution_clock::now();
        for (auto &w : workers)
        {
            std::lock_guard<std::mutex> lk(w.mtx);
            w.do_read = true;
            w.done = false;
            w.cv.notify_all();
        }
        for (auto &w : workers)
        {
            std::unique_lock<std::mutex> lk(w.mtx);
            w.cv.wait(lk, [&]
                      { return w.done.load(); });
        }
        auto t1_read = std::chrono::high_resolution_clock::now();
        double t_read_phase = std::chrono::duration<double, std::milli>(t1_read - t0_read).count();

        // === WRITE PHASE ====================================================
        auto t0_write = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < CRANE_JOINTS_SIZE; i++)
        {
            auto &w = workers[i];
            std::lock_guard<std::mutex> lk(w.mtx);
            w.ref = 5.0f * std::sin(0.1 * cycle + i);
            w.do_write = true;
            w.done = false;
            w.cv.notify_all();
        }
        for (auto &w : workers)
        {
            std::unique_lock<std::mutex> lk(w.mtx);
            w.cv.wait(lk, [&]
                      { return w.done.load(); });
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

        std::cout << "\033[2J\033[H"; // clear screen
        std::cout << "\n----------------------------------------------------------------------------------\n";
        std::cout << " Cycle: " << cycle++ << "\n";
        std::cout << "----------------------------------------------------------------------------------\n";

        for (int i = 0; i < CRANE_JOINTS_SIZE; i++)
        {
            auto &m = workers[i].meas;
            std::cout << "[Joint " << i << "] Pos: " << m.actuator_angular_position
                      << " rad | Vel: " << m.actuator_angular_velocity
                      << " rad/s | V: " << m.actuator_voltage
                      << " V | I: " << m.actuator_current
                      << " A | Ref: " << workers[i].ref << "\n";

            std::cout << "   Thread Times: Read=" << workers[i].t_read
                      << " ms, Write=" << workers[i].t_write
                      << " ms, Sum=" << workers[i].t_sum << " ms\n";
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

    for (auto &w : workers)
    {
        {
            std::lock_guard<std::mutex> lk(w.mtx);
            w.run = false;
            w.cv.notify_all();
        }
        if (w.th.joinable())
            w.th.join();
    }

    return 0;
}
