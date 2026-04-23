/*
 * rtos_types.h
 *
 *  Created on: Sep 17, 2023
 *      Author: Tiago Giacomelli
 *
 *  RTOS semaphore and mutex type definitions and configuration macros.
 */

#ifndef SJC__SYSTEM_TYPES__RTOS_TYPES_H_
#define SJC__SYSTEM_TYPES__RTOS_TYPES_H_

// System includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "cmsis_os.h"
#include "semphr.h"
// ------------------------------------------------------------------- //

// Definitions includes ---------------------------------------------- //
// ------------------------------------------------------------------- //
#define BINARY_SEMPHR_COUNT_SIZE ((uint32_t)1U)
#define BINARY_SEMPHR_INITIAL_COUNT ((uint32_t)1U)
// ------------------------------------------------------------------- //

#endif /* SJC__SYSTEM_TYPES__RTOS_TYPES_H_ */
