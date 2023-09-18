/*
 * types.h
 *
 *  Created on: Sep 17, 2023
 *      Author: tiago
 */

#ifndef SYSTEM_TYPES_INCLUDE_TYPES_H_
#define SYSTEM_TYPES_INCLUDE_TYPES_H_

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "math.h"

#include "data_structures.h"

typedef enum {
	SJC_RET_SJC_NOK = 0x0000,
	SJC_RET_SJC_OK = 0xffff,
	SJC_RET_SIZE
} eSjcReturnType_t;

#endif /* SYSTEM_TYPES_INCLUDE_TYPES_H_ */
