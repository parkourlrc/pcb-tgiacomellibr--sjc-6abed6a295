/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__MODULES__ANALOG_SENSORS_H_
#define SJC__MODULES__ANALOG_SENSORS_H_

#include "types.h"

eSjcReturnType_t SJC_AnalogSensors_Init(xSjcAnalogSensor_t *Sensor);
eSjcReturnType_t SJC_AnalogSensors_Configure(uint32_t * buffer, const uint32_t buffer_size);
eSjcReturnType_t SJC_AnalogSensors_StartMeasurements(void);
eSjcReturnType_t SJC_AnalogSensors_StopMeasurements(void);
eSjcReturnType_t SJC_AnalogSensors_ComputeConversionValue( const eSjcAnalogSensor_t input, const uint16_t conversion, float * value );
eSjcReturnType_t SJC_AnalogSensors_ComputeFilteredValue( const eSjcAnalogSensor_t input, const float previous, const float actual, float * value );

#endif /* SJC__MODULES__ANALOG_SENSORS_H_ */
