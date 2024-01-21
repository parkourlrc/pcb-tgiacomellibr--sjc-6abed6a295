/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__ANALOG_INTERFACES_H_
#define SJC__MODULES__ANALOG_INTERFACES_H_

#include "types.h"

void SJC_AnalogInterfaces_Init(void);
void SJC_AnalogInterfaces_MeasurementEnable(void);
void SJC_AnalogInterfaces_MeasurementDisable(void);
eSjcReturnType_t SJC_AnalogInterfaces_UpdateConversionValues(void);
eSjcReturnType_t SJC_AnalogInterfaces_GetMeanConversionDelay(xSjcMeanSampleTimeMeasurement_t * Ts);
eSjcReturnType_t SJC_AnalogInterfaces_GetAnalogConversion(const eSjcAnalogSensorType_t sensor, float * value);

void SJC_AnalogInterfaces_AdcConversionIsr(void);

#endif /* SJC__MODULES__ANALOG_INTERFACES_H_ */
