/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__ANALOG_INTERFACES_H_
#define SJC__MODULES__ANALOG_INTERFACES_H_

#include "system_types/include/types.h"

void AnalogInterfaces_init(void);
void AnalogInterfaces_enable(void);
void AnalogInterfaces_disable(void);
eSjcReturnType_t AnalogInterfaces_updateConversionValues(void);
eSjcReturnType_t AnalogInterfaces_checkUpdateConversionTime(void);
eSjcReturnType_t AnalogInterfaces_getMeanConversionDelay(xSjcMeanSampleTimeMeasurement_t *Ts);
eSjcReturnType_t AnalogInterfaces_getAnalogConversion(const eSjcAnalogSensorType_t sensor, float *value);

void SJC_AnalogInterfaces_AdcConversionIsr(void);

#endif /* SJC__MODULES__ANALOG_INTERFACES_H_ */
