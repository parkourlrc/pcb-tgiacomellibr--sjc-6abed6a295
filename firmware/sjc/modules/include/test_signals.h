/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__TEST_SIGNALS_H_
#define SJC__MODULES__TEST_SIGNALS_H_

#include "types.h"

void SJC_TestSignals_Init(void);
void SJC_TestSignals_Reset(void);
eSjcReturnType_t SJC_TestSignals_SetConfig(const eSjcInputType_t type, const float time_factor);
eSjcReturnType_t SJC_TestSignals_GetState(eSjcInputSignalState_t *state);
eSjcReturnType_t SJC_TestSignals_GetValue(float *value);

#endif /* SJC__MODULES__TEST_SIGNALS_H_ */
