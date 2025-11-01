/*
 * motor_drive.h
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago
 */

#ifndef SJC__MODULES__DC_MOTOR_DRIVE_H_
#define SJC__MODULES__DC_MOTOR_DRIVE_H_

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initializes the motor drive module.
     *
     * @param BaseFrequency PWM base frequency.
     * @param BaseVoltage Nominal DC bus voltage.
     */
    void MotorDrive_init(eSjcPwmBaseFreq_t BaseFrequency, float BaseVoltage);

    /**
     * @brief Enables the motor drive (activates PWM and hardware enable).
     */
    void MotorDrive_enable(void);

    /**
     * @brief Disables the motor drive and sets output voltage to zero.
     */
    void MotorDrive_disable(void);

    /**
     * @brief Retrieves the drive enable state.
     *
     * @param state Pointer to receive the state (true = enabled).
     * @return SJC_RET_OK on success.
     */
    eSjcReturnType_t MotorDrive_getState(bool *state);

    /**
     * @brief Gets the measured power supply voltage.
     *
     * @param voltage Pointer to store the voltage (V).
     * @return SJC_RET_OK on success.
     */
    eSjcReturnType_t MotorDrive_getPowerSupplyVoltage(float *voltage);

    /**
     * @brief Gets the configured base voltage for the drive.
     *
     * @param voltage Pointer to store the base voltage.
     * @return SJC_RET_OK on success.
     */
    eSjcReturnType_t MotorDrive_getBaseVoltage(float *voltage);

    /**
     * @brief Sets the base voltage and updates internal range.
     *
     * @param voltage New base voltage.
     * @return SJC_RET_OK on success.
     */
    eSjcReturnType_t MotorDrive_setBaseVoltage(float voltage);

    /**
     * @brief Gets the output current from the motor drive.
     *
     * @param current Pointer to store the current (A).
     * @return SJC_RET_OK on success.
     */
    eSjcReturnType_t MotorDrive_getOutputCurrent(float *current);

    /**
     * @brief Gets the currently applied output voltage.
     *
     * @param voltage Pointer to store the voltage (V).
     * @return SJC_RET_OK on success.
     */
    eSjcReturnType_t MotorDrive_getOutputVoltage(float *voltage);

    /**
     * @brief Sets the motor output voltage (clamped and converted to PWM).
     *
     * @param voltage Desired output voltage (V).
     * @return SJC_RET_OK on success.
     */
    eSjcReturnType_t MotorDrive_setOutputVoltage(const float voltage);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__DC_MOTOR_DRIVE_H_ */
