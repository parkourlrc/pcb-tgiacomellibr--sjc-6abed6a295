/*
 * motor_output.h
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  Controls the digital enable signal for the power output stage.
 */

#ifndef SJC__MODULES__MOTOR_OUTPUT_H_
#define SJC__MODULES__MOTOR_OUTPUT_H_

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initializes the power output control interface.
     */
    void MotorOutput_init(void);

    /**
     * @brief Enables the power output (activates the digital signal).
     */
    void MotorOutput_enable(void);

    /**
     * @brief Disables the power output.
     */
    void MotorOutput_disable(void);

    /**
     * @brief Returns the current state of the output (enabled or not).
     *
     * @param state Pointer where the state will be stored.
     * @return SJC_RET_OK if successful, SJC_RET_NOK if pointer is null.
     */
    eSjcReturnType_t MotorOutput_getState(bool *state);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__MOTOR_OUTPUT_H_ */
