/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__DEBUG_OUTPUT_H_
#define SJC__MODULES__DEBUG_OUTPUT_H_

#include "types.h"

void DebugOutput_init(void);
void DebugOutput_presentSystemInfo(void);
int DebugOutput_systemPrintf(const char *format, ...);

#endif /* SJC__MODULES__DEBUG_OUTPUT_H_ */
