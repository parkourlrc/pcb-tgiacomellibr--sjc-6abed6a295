/*
 * motor_drive.h
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  DC motor drive control and measurement interface.
 *  Provides PWM-based voltage control, current/voltage measurement,
 *  bias calibration, and enable/disable sequencing for an H-bridge driver.
 */

#ifndef SJC__MODULES__DC_MOTOR_DRIVE_H_
#define SJC__MODULES__DC_MOTOR_DRIVE_H_

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Motor drive electrical measurements snapshot.
     */
    typedef struct
    {
        float current;        ///< Output current (bias compensated) [A]
        float voltage;        ///< Output voltage [V]
        float supply_voltage; ///< DC bus voltage [V]
        float base_voltage;   ///< Configured base voltage [V]
        bool state;           ///< Drive output state (true = enabled)
    } xMotorDriveMeasurements_t;

    /**
     * @brief Initializes the motor drive module.
     *
     * @param BaseFrequency PWM base frequency selection.
     * @param BaseVoltage Maximum symmetric output voltage [V]. Must be > 0.
     */
    void MotorDrive_init(eSjcPwmBaseFreq_t BaseFrequency, float BaseVoltage);

    /**
     * @brief Enables the motor drive.
     *
     * Sequence: set duty to 50% (0V) -> enable PWM -> enable driver GPIO.
     */
    void MotorDrive_enable(void);

    /**
     * @brief Disables the motor drive.
     *
     * Sequence: zero internal voltage -> disable driver GPIO -> disable PWM.
     */
    void MotorDrive_disable(void);

    /**
     * @brief Retrieves the drive enable state.
     *
     * @param state Pointer to receive the state (true = enabled).
     * @return SJC_RET_OK on success, SJC_RET_NOK otherwise.
     */
    eSjcReturnType_t MotorDrive_getState(bool *state);

    /**
     * @brief Gets the measured DC bus voltage.
     *
     * @param voltage Pointer to store the voltage [V].
     * @return SJC_RET_OK on success, SJC_RET_NOK otherwise.
     */
    eSjcReturnType_t MotorDrive_getPowerSupplyVoltage(float *voltage);

    /**
     * @brief Gets the configured base voltage.
     *
     * @param voltage Pointer to store the base voltage [V].
     * @return SJC_RET_OK on success, SJC_RET_NOK otherwise.
     */
    eSjcReturnType_t MotorDrive_getBaseVoltage(float *voltage);

    /**
     * @brief Sets the base voltage and updates internal output range.
     *
     * Rejects voltage <= 0.
     *
     * @param voltage New base voltage [V].
     * @return SJC_RET_OK on success, SJC_RET_NOK if voltage <= 0.
     */
    eSjcReturnType_t MotorDrive_setBaseVoltage(float voltage);

    /**
     * @brief Gets the raw output current (without bias compensation).
     *
     * @param current Pointer to store the current [A].
     * @return SJC_RET_OK on success, SJC_RET_NOK otherwise.
     */
    eSjcReturnType_t MotorDrive_getOutputCurrent(float *current);

    /**
     * @brief Gets the output current with bias compensation.
     *
     * @param current Pointer to store the unbiased current [A].
     * @return SJC_RET_OK on success, SJC_RET_NOK otherwise.
     */
    eSjcReturnType_t MotorDrive_getOutputCurrentUnbiased(float *current);

    /**
     * @brief Sets the current sensor bias for zero-current calibration.
     *
     * @param bias Current bias value [A].
     */
    void MotorDrive_setCurrentBias(const float bias);

    /**
     * @brief Gets the currently applied output voltage.
     *
     * @param voltage Pointer to store the voltage [V].
     * @return SJC_RET_OK on success, SJC_RET_NOK otherwise.
     */
    eSjcReturnType_t MotorDrive_getOutputVoltage(float *voltage);

    /**
     * @brief Sets the motor output voltage (clamped to [-base, +base] and converted to PWM).
     *
     * @param voltage Desired output voltage [V].
     */
    void MotorDrive_setOutputVoltage(const float voltage);

    /**
     * @brief Gets all electrical measurements in a single call.
     *
     * Current is returned with bias compensation.
     *
     * @param measurements Pointer to store the measurements snapshot.
     * @return SJC_RET_OK if all readings succeeded, SJC_RET_NOK otherwise.
     */
    eSjcReturnType_t MotorDrive_getMeasurements(xMotorDriveMeasurements_t *measurements);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__DC_MOTOR_DRIVE_H_ */
