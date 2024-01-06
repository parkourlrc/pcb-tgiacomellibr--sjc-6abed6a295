/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__ANALOG_INTERFACES_H_
#define SJC__MODULES__ANALOG_INTERFACES_H_

#include "types.h"

eSjcReturnType_t SJC_AnalogInterfaces_Init(void);
eSjcReturnType_t SJC_AnalogInterfaces_ConfigureInterface(xSjcAnalogSensor_t *Sensor);
eSjcReturnType_t SJC_AnalogInterfaces_MeasurementEnable(void);
eSjcReturnType_t SJC_AnalogInterfaces_MeasurementDisable(void);
eSjcReturnType_t SJC_AnalogInterfaces_SetSharedDataMutex(osMutexId_t MutexHandle);
eSjcReturnType_t SJC_AnalogInterfaces_GetMeanSampleTime(xSjcSampleTimeMeasurement_t * Ts);
eSjcReturnType_t SJC_AnalogInterfaces_GetAnalogConversion(const eSjcAnalogSensorType_t sensor, float * value);

#endif /* SJC__MODULES__ANALOG_INTERFACES_H_ */
