/*
 * debug_output.h
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  Thread-safe debug output via USB CDC.
 *  Provides mutex-protected formatted printing over USB Virtual COM Port.
 */

#ifndef SJC__MODULES__DEBUG_OUTPUT_H_
#define SJC__MODULES__DEBUG_OUTPUT_H_

#include "system_types/include/types.h"

/**
 * @brief Initializes the debug output module and creates the mutex.
 */
void DebugOutput_init(void);

/**
 * @brief Prints system information (HW/FW version, network config).
 */
void DebugOutput_presentSystemInfo(void);

/**
 * @brief Prints RTOS runtime statistics (task stack usage, execution times).
 */
void DebugOutput_presentRuntimeStatistics(void);

/**
 * @brief Thread-safe formatted print via USB CDC.
 *
 * @param format printf-style format string.
 * @return Number of characters written, or -1 on mutex timeout.
 */
int DebugOutput_systemPrintf(const char *format, ...);

#endif /* SJC__MODULES__DEBUG_OUTPUT_H_ */
