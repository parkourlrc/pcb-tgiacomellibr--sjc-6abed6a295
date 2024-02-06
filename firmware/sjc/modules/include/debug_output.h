/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__DEBUG_OUTPUT_H_
#define SJC__MODULES__DEBUG_OUTPUT_H_

#include "types.h"

void SJC_DebugOutput_Init(void);
void SJC_DebugOutput_PresentSystemInfo(void);
void SJC_DebugOutput_PresentRuntimeStatistics(void);
int SJC_DebugOutput_SystemPrintf(const char *format, ...);
int SJC_DebugOutput_LoggerPrintf(const char *format, ...);

#endif /* SJC__MODULES__DEBUG_OUTPUT_H_ */
