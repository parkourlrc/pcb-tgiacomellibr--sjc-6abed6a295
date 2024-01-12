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

osStatus_t SJC_TaskCommon_SharedDataMutexTake(const uint32_t timeout);
osStatus_t SJC_TaskCommon_SharedDataMutexGive(void);

osStatus_t SJC_TaskCommon_SerialDebugMutexTake(const uint32_t timeout);
osStatus_t SJC_TaskCommon_SerialDebugMutexGive(void);

osStatus_t SJC_TaskCommon_UsbDebugMutexTake(const uint32_t timeout);
osStatus_t SJC_TaskCommon_UsbDebugMutexGive(void);


#endif /* SJC__TASKS__COMMON_H_ */
