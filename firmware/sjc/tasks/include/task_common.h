/*
 * common.h
 *
 *  Created on: Oct 4, 2023
 *      Author: Tiago
 */

#ifndef SJC__TASKS__COMMON_H_
#define SJC__TASKS__COMMON_H_

#include "types.h"

eSjcReturnType_t SJC_TaskCommon_InitThreads(void);

osStatus_t SJC_TaskCommon_SharedDataSemaphoreTake(const uint32_t timeout);
osStatus_t SJC_TaskCommon_SharedDataSemaphoreGive(void);

osStatus_t SJC_TaskCommon_SerialDebugSemaphoreTake(const uint32_t timeout);
osStatus_t SJC_TaskCommon_SerialDebugSemaphoreGive(void);

osStatus_t SJC_TaskCommon_UsbDebugSemaphoreTake(const uint32_t timeout);
osStatus_t SJC_TaskCommon_UsbDebugSemaphoreGive(void);


#endif /* SJC__TASKS__COMMON_H_ */
