/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__ANALOG_SENSORS_H_
#define SJC__ANALOG_SENSORS_H_

typedef enum {
	SJC_CURRENT_SENSOR_1 = 0,
	SJC_CURRENT_SENSOR_2,
	SJC_VOLTAGE_SENSOR,
	SJC_ANALOG_SENSOR_SIZE
} eSjcAnalogSensor_t;

void SJC_AnalogSensorsInit(void);
float SJC_AnalogSensors_GetConversion(const eSjcAnalogSensor_t sensor);

#endif /* SJC__ANALOG_SENSORS_H_ */
