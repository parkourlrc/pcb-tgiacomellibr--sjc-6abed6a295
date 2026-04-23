#ifndef SJC_REALTIME_INTERFACE_HPP_
#define SJC_REALTIME_INTERFACE_HPP_

#include "sjc_interface.hpp"
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

/**
 * @class SjcRealtimeInterface
 * @brief Extends SjcInterface with a realtime worker thread.
 *
 * Provides asynchronous read/write cycles managed by a dedicated worker thread.
 * Synchronization is achieved via condition variables for triggering actions and
 * waiting for their completion.
 *
 * ### Usage
 * - Call start() to launch the background worker thread.
 * - Use trigger_read() / trigger_write() to enqueue operations.
 * - Use wait_done() (blocking) or wait_done(timeout) to synchronize.
 * - Use getters to retrieve runtime info and timing metrics safely.
 *
 * ### Threading model
 * - A background worker thread executes Modbus read/write operations.
 * - Actions are enqueued via trigger_* methods and dispatched by loop().
 * - Completion is signaled via cv_done_, allowing synchronous waiting.
 */
class SjcRealtimeInterface : public SjcInterface
{
public:
    /**
     * @brief Construct a new SjcRealtimeInterface.
     *
     * Initializes serial communication and prepares internal structures.
     *
     * @param port Serial port device path.
     * @param address Modbus slave address.
     * @param baudrate Modbus RTU baudrate.
     */
    SjcRealtimeInterface(const std::string &port, int address, int baudrate);

    /**
     * @brief Destructor.
     *
     * Ensures the worker thread is stopped cleanly before destruction.
     * @note Virtual for safe polymorphic destruction.
     */
    virtual ~SjcRealtimeInterface();

    /**
     * @brief Start the background worker thread.
     *
     * Allocates and runs the realtime loop() in a dedicated std::thread.
     * Must be called before issuing trigger_read()/trigger_write().
     */
    void start();

    /**
     * @brief Stop the background worker thread.
     *
     * Signals shutdown and joins the worker thread.
     * This call is blocking until loop() exits.
     */
    void stop();

    /**
     * @brief Trigger a read cycle asynchronously.
     *
     * Signals the worker thread to perform a Modbus read operation.
     * Non-blocking: returns immediately.
     * Use wait_done() or wait_done(timeout) to synchronize.
     */
    void trigger_read();

    /**
     * @brief Trigger a write cycle asynchronously.
     *
     * Signals the worker thread to perform a Modbus write operation.
     * Non-blocking: returns immediately.
     * Use wait_done() or wait_done(timeout) to synchronize.
     *
     * @param reference Controller reference value to be written.
     */
    void trigger_write(float reference);

    /**
     * @brief Wait until the worker finishes the current operation.
     *
     * Blocks the calling thread until the last trigger_read()/trigger_write()
     * operation completes.
     *
     * @note Safe to call after each trigger_*.
     */
    void wait_done();

    /**
     * @brief Wait until the worker finishes the current operation, with timeout.
     *
     * Blocks for at most @p timeout duration.
     *
     * @param timeout Maximum time to wait (e.g., std::chrono::milliseconds(5)).
     * @return true if the operation finished within timeout, false otherwise.
     *
     * @note Useful for detecting stalled or delayed workers.
     */
    bool wait_done(std::chrono::milliseconds timeout);

    /**
     * @brief Get the last runtime measurement info.
     *
     * Provides a copy of the latest measurement struct reported by the worker.
     *
     * @return xSjcControlMeasInfo_t copy of last runtime info.
     * @note Thread-safe.
     */
    xSjcControlMeasInfo_t get_runtime_info();

    /**
     * @brief Get duration of the last read cycle.
     *
     * @return Duration in milliseconds.
     * @note Thread-safe.
     */
    double get_last_read_ms();

    /**
     * @brief Get duration of the last write cycle.
     *
     * @return Duration in milliseconds.
     * @note Thread-safe.
     */
    double get_last_write_ms();

    /**
     * @brief Get the status code of the last read operation.
     *
     * @return Status returned by getControlMeasInfo().
     */
    eSjcReturnType_t get_last_read_status();

    /**
     * @brief Get the status code of the last write operation.
     *
     * @return Status returned by setControllerReference().
     */
    eSjcReturnType_t get_last_write_status();

private:
    /**
     * @brief Worker loop.
     *
     * Implements the realtime cycle:
     * - Waits for trigger signals (cv_).
     * - Performs the requested READ/WRITE action.
     * - Updates runtime_info_, timing metrics.
     * - Signals completion via cv_done_.
     */
    void loop();

    std::thread worker_;              ///< Worker thread handle.
    std::mutex mtx_;                  ///< Mutex protecting shared state.
    std::condition_variable cv_;      ///< Signals arrival of new action.
    std::condition_variable cv_done_; ///< Signals completion of action.
    bool shutdown_{false};            ///< True if stop() was requested.

    /**
     * @enum xSjcAction
     * @brief Possible worker actions.
     */
    enum class xSjcAction
    {
        NONE, ///< Idle, no pending action.
        READ, ///< Perform a read cycle.
        WRITE ///< Perform a write cycle.
    };

    xSjcAction action_{xSjcAction::NONE}; ///< Current action requested.

    float pending_ref_{0.0f}; ///< Pending reference for WRITE action.

    xSjcControlMeasInfo_t runtime_info_; ///< Last measurement info from worker.
    double last_read_ms_{0.0};           ///< Last read operation duration (ms).
    double last_write_ms_{0.0};          ///< Last write operation duration (ms).
    eSjcReturnType_t last_read_status_{SJC_RET_OK};   ///< Last read operation status.
    eSjcReturnType_t last_write_status_{SJC_RET_OK};  ///< Last write operation status.
};

#endif // SJC_REALTIME_INTERFACE_HPP_
