/**
 * @file data_logger.h
 * @brief Asynchronous data logger using USART + DMA for non-blocking serial output.
 *
 * Provides formatted logging via DMA transmission or blocking fallback using LL USART.
 *
 * Author: Tiago
 * Date: Sep 9, 2024
 */

#ifndef SJC__MODULES__DATA_LOGGER_H_
#define SJC__MODULES__DATA_LOGGER_H_

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initializes the data logger module and associated DMA configuration.
     */
    void DataLogger_init(void);

    /**
     * @brief Sends a formatted string asynchronously using the DMA circular buffer.
     *
     * If the current bank is full or nearly full, switches to the next bank.
     * Supports splitting messages across banks.
     *
     * @param format Format string (same as printf)
     * @return Number of characters written to the buffer
     */
    int DataLogger_transmit(const char *format, ...);

    /**
     * @brief Sends a formatted string directly using blocking USART transmission (debug mode).
     *
     * Uses a critical section to avoid USART/DMA conflicts.
     *
     * @param format Format string (same as printf)
     * @return Number of characters transmitted
     */
    int DataLogger_printf(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__DATA_LOGGER_H_ */
