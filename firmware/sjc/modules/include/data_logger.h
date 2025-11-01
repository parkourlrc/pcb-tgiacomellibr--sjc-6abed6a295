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
     * @brief Initializes the DataLogger module.
     *
     * Configures DMA, USART, and internal buffers. Must be called once during system startup.
     */
    void DataLogger_init(void);

    /**
     * @brief Queues a formatted message for asynchronous transmission.
     *
     * Uses internal double buffer to store messages. If message exceeds the remaining space
     * in the current bank, the bank is transmitted and the message is retried in the next bank.
     *
     * @param format Format string (printf-style)
     * @param ... Arguments for formatting
     * @return int Number of bytes written to the buffer
     */
    int DataLogger_transmit(const char *format, ...);

    /**
     * @brief Prints a formatted message immediately over USART using polling.
     *
     * Blocks until all characters are transmitted. Used for emergency or fault reporting.
     *
     * @param format Format string (printf-style)
     * @param ... Arguments for formatting
     * @return int Number of characters transmitted
     */
    int DataLogger_printf(const char *format, ...);

    /**
     * @brief Forces transmission of the current buffer if data is available.
     *
     * Can be used to flush logs before shutdown or in critical sections.
     */
    void DataLogger_flush(void);

    /**
     * @brief Checks whether there is pending data in the buffer.
     *
     * @return true if there is buffered data to be transmitted, false otherwise.
     */
    bool DataLogger_available(void);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__DATA_LOGGER_H_ */
