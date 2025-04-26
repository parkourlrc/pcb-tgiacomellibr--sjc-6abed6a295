/**
 * @file lqi_controller.h
 * @brief Header for compatible LQI controller with anti-windup and exponential filter.
 *
 * Author: Tiago
 * Date: Apr 23, 2025
 */

#ifndef MODULES_INCLUDE_LQI_CONTROLLER_H_
#define MODULES_INCLUDE_LQI_CONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "modules/include/current_observer.h"

#define LQI_STATE_SIZE 2

typedef struct
{
    float Ad[LQI_STATE_SIZE][LQI_STATE_SIZE];
    float Bd[LQI_STATE_SIZE];
    float Cd[LQI_STATE_SIZE];

    float Kx[LQI_STATE_SIZE];
    float Ki;
    float Kw;

    float alpha;
    float u_min;
    float u_max;

    bool use_observer;
    CurrentObserver_t *observer;
} LqiControllerParams_t;

void LqiController_init(const LqiControllerParams_t *params);
float LqiController_compute(float ref, float y_meas, float u_prev);
void LqiController_reset(void);
void LqiController_setSaturation(float u_min, float u_max);


#ifdef __cplusplus
}
#endif

#endif /* MODULES_INCLUDE_LQI_CONTROLLER_H_ */
