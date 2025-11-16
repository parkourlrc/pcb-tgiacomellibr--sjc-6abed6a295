#include "sjc_realtime_interface.hpp"
#include <chrono>
#include <iostream>

/**
 * @brief Construct a new SjcRealtimeInterface.
 */
SjcRealtimeInterface::SjcRealtimeInterface(const std::string &port,
                                           const int address,
                                           const int baudrate)
    : SjcInterface(port, address, baudrate) {}

/**
 * @brief Destroy the SjcRealtimeInterface object.
 *
 * Calls stop() to ensure worker thread exits cleanly.
 */
SjcRealtimeInterface::~SjcRealtimeInterface()
{
    stop();
}

/**
 * @brief Start the background worker thread.
 */
void SjcRealtimeInterface::start()
{
    {
        std::lock_guard<std::mutex> lock(mtx_);
        shutdown_ = false;
        done_ = false;
        last_read_status_ = SJC_RET_OK;
        last_write_status_ = SJC_RET_OK;
        last_read_ms_ = 0.0;
        last_write_ms_ = 0.0;
    }
    worker_ = std::thread(&SjcRealtimeInterface::loop, this);
}

/**
 * @brief Stop the background worker thread (blocking).
 */
void SjcRealtimeInterface::stop()
{
    {
        std::lock_guard<std::mutex> lock(mtx_);
        shutdown_ = true;
    }
    cv_.notify_all();

    if (worker_.joinable())
    {
        worker_.join();
    }
}

/**
 * @brief Trigger a read operation asynchronously.
 */
void SjcRealtimeInterface::trigger_read()
{
    {
        std::lock_guard<std::mutex> lock(mtx_);
        action_ = xSjcAction::READ;
        done_ = false;
    }
    cv_.notify_all();
}

/**
 * @brief Trigger a write operation asynchronously.
 */
void SjcRealtimeInterface::trigger_write(const float reference)
{
    {
        std::lock_guard<std::mutex> lock(mtx_);
        pending_ref_ = reference;
        action_ = xSjcAction::WRITE;
        done_ = false;
    }
    cv_.notify_all();
}

/**
 * @brief Wait indefinitely until the last action finishes.
 */
void SjcRealtimeInterface::wait_done()
{
    std::unique_lock<std::mutex> lock(mtx_);
    cv_done_.wait(lock, [&]
                  { return action_ == xSjcAction::NONE; });
}

/**
 * @brief Wait until the last action finishes or timeout expires.
 */
bool SjcRealtimeInterface::wait_done(std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(mtx_);
    return cv_done_.wait_for(lock, timeout, [&]
                             { return action_ == xSjcAction::NONE; });
}

/**
 * @brief Get the last runtime measurement info.
 */
xSjcControlMeasInfo_t SjcRealtimeInterface::get_runtime_info()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return runtime_info_;
}

/**
 * @brief Get duration of the last read cycle (ms).
 */
double SjcRealtimeInterface::get_last_read_ms()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return last_read_ms_;
}

/**
 * @brief Get duration of the last write cycle (ms).
 */
double SjcRealtimeInterface::get_last_write_ms()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return last_write_ms_;
}

/**
 * @brief Get status of the last read operation.
 */
eSjcReturnType_t SjcRealtimeInterface::get_last_read_status()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return last_read_status_;
}

/**
 * @brief Get status of the last write operation.
 */
eSjcReturnType_t SjcRealtimeInterface::get_last_write_status()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return last_write_status_;
}

/**
 * @brief Worker loop executed in a background thread.
 */
void SjcRealtimeInterface::loop()
{
    while (true)
    {
        xSjcAction current_action = xSjcAction::NONE;

        {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_.wait(lock, [&]
                     { return shutdown_ || action_ != xSjcAction::NONE; });

            if (shutdown_)
            {
                {
                    std::lock_guard<std::mutex> done_lock(mtx_);
                    action_ = xSjcAction::NONE;
                    done_ = true;
                    last_read_status_ = SJC_CONNECTION_RELEASED;
                    last_write_status_ = SJC_CONNECTION_RELEASED;
                }
                cv_done_.notify_all();
                break;
            }
            current_action = action_;
        }

        auto t0 = std::chrono::high_resolution_clock::now();

        if (current_action == xSjcAction::READ)
        {
            xSjcControlMeasInfo_t tmp;
            eSjcReturnType_t status = this->getControlMeasInfo(&tmp);
            auto t1 = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double, std::milli>(t1 - t0).count();

            {
                std::lock_guard<std::mutex> lock(mtx_);
                last_read_status_ = status;
                last_read_ms_ = elapsed;
                if (status == SJC_RET_OK)
                {
                    runtime_info_ = tmp;
                }
                else
                {
                    runtime_info_ = {};
                }
            }

            if (status != SJC_RET_OK)
            {
                std::cerr << "[SJC] Failed to read control measurement info (status=" << status << ")" << std::endl;
            }
        }
        else if (current_action == xSjcAction::WRITE)
        {
            eSjcReturnType_t status = this->setControllerReference(pending_ref_);

            auto t1 = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double, std::milli>(t1 - t0).count();

            std::lock_guard<std::mutex> lock(mtx_);
            last_write_status_ = status;
            last_write_ms_ = elapsed;

            if (status != SJC_RET_OK)
            {
                std::cerr << "[SJC] Failed to write controller reference (status=" << status << ")" << std::endl;
            }
        }

        {
            std::lock_guard<std::mutex> lock(mtx_);
            action_ = xSjcAction::NONE;
            done_ = true;
        }
        cv_done_.notify_all();
    }
}
