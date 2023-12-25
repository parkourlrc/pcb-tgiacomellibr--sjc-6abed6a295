/*
 * control_task.h
 *
 *  Created on: Oct 4, 2023
 *      Author: Tiago
 */

#ifndef SJC__TASKS__ACTUATOR_CONTROLLER_TASK_H_
#define SJC__TASKS__ACTUATOR_CONTROLLER_TASK_H_

#include "types.h"

eSjcReturnType_t SJC_TaskActuatorController_Init(void);
eSjcReturnType_t SJC_TaskActuatorController_SetSharedDataMutex(osMutexId_t MutexHandle);

#endif /* SJC__TASKS__ACTUATOR_CONTROLLER_TASK_H_ */
