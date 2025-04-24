/*
 * data_logger.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__DATA_LOGGER_H_
#define SJC__MODULES__DATA_LOGGER_H_

#include "system_types/include/types.h"

void DataLogger_init(void);
int DataLogger_transmit(const char *format, ...);
int DataLogger_printf(const char *format, ...);

#endif /* SJC__MODULES__DATA_LOGGER_H_ */
