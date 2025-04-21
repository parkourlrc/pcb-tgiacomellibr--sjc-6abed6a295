/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__TEST_SIGNALS_H_
#define SJC__MODULES__TEST_SIGNALS_H_

#include "system_types/include/types.h"

void TestSignals_init(void);
void TestSignals_reset(void);
eSjcReturnType_t TestSignals_setConfig(const eSjcInputType_t type, const float time_factor);
eSjcReturnType_t TestSignals_getState(eSjcInputSignalState_t *state);
eSjcReturnType_t TestSignals_getValue(float *value);

#endif /* SJC__MODULES__TEST_SIGNALS_H_ */
