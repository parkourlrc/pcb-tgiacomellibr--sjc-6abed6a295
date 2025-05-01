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
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

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

    /**
     * @brief Initializes the LQI controller with the provided parameters.
     */
    void LqiController_init(const LqiControllerParams_t *params);

    /**
     * @brief Computes the control effort based on reference and measurement.
     *
     * @param ref Desired reference value.
     * @param y_meas Measured process variable.
     * @return Control effort (saturated output).
     */
    float LqiController_compute(float ref, float y_meas);

    /**
     * @brief Resets the internal controller states.
     */
    void LqiController_reset(void);

    /**
     * @brief Sets the control output saturation limits.
     *
     * @param u_min Minimum allowed output.
     * @param u_max Maximum allowed output.
     */
    void LqiController_setSaturation(float u_min, float u_max);

    /**
     * @brief Returns whether the controller has been initialized.
     */
    bool LqiController_isInitialized(void);

#ifdef __cplusplus
}
#endif

#endif /* MODULES_INCLUDE_LQI_CONTROLLER_H_ */
