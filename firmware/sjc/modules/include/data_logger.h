/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__DATA_LOGGER_H_
#define SJC__MODULES__DATA_LOGGER_H_

#include "types.h"

void SJC_DataLogger_Init(void);
int SJC_DataLogger_Transmit(const char *format, ...);

#endif /* SJC__MODULES__DATA_LOGGER_H_ */
