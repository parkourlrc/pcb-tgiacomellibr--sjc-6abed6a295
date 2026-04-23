/*
 * velocity_measurement.h
 *
 *  Created on: Mar 2026
 *      Author: Tiago Giacomelli
 *
 *  Filtered angular velocity measurement for each joint.
 *  Computes angular velocity from position updates (displacement / dt)
 *  and applies exponential IIR filtering. Tracks previous position
 *  internally for each joint.
 */

#ifndef SJC__MODULES__VELOCITY_MEASUREMENT_H_
#define SJC__MODULES__VELOCITY_MEASUREMENT_H_

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initializes the velocity measurement module for all joints.
     *
     * Sets up exponential filters and zeroes all internal state.
     *
     * @param fc Cutoff frequency in Hz.
     * @param fs Sampling frequency in Hz.
     */
    void VelocityMeasurement_init(float fc, float fs);

    /**
     * @brief Sets the initial position for a specific joint.
     *
     * Must be called before the first update to avoid a large
     * displacement spike on the first cycle.
     *
     * @param joint Joint identifier
     * @param position Initial angular position [rad]
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise
     */
    eSjcReturnType_t VelocityMeasurement_setInitialPosition(const eSjcJoints_t joint, const float position);

    /**
     * @brief Sets the initial positions for all joints.
     *
     * @param positions Array of size SJC_JOINTS_SIZE with initial angular positions [rad]
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise
     */
    eSjcReturnType_t VelocityMeasurement_setAllInitialPositions(const float positions[SJC_JOINTS_SIZE]);

    /**
     * @brief Resets the filtered velocity and previous position of a specific joint.
     *
     * @param joint Joint identifier
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise
     */
    eSjcReturnType_t VelocityMeasurement_reset(const eSjcJoints_t joint);

    /**
     * @brief Resets the filtered velocity and previous position for selected joints in a single mutex-protected call.
     *
     * @param joints Array of joint identifiers to reset.
     * @param count Number of joints in the array.
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise
     */
    eSjcReturnType_t VelocityMeasurement_resetMultiple(const eSjcJoints_t joints[], uint8_t count);

    /**
     * @brief Resets the filtered velocity and previous position of all joints.
     *
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise
     */
    eSjcReturnType_t VelocityMeasurement_resetAll(void);

    /**
     * @brief Updates the velocity for a specific joint from a new position sample.
     *
     * Computes displacement from previous position, divides by dt,
     * applies exponential filter, and stores result.
     *
     * @param joint Joint identifier
     * @param position Current angular position [rad]
     * @param dt Time since last update [s]
     * @param velocity Pointer to store computed filtered velocity [rad/s] (can be NULL if not needed)
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise
     */
    eSjcReturnType_t VelocityMeasurement_update(const eSjcJoints_t joint, const float position, const float dt, float *velocity);

    /**
     * @brief Updates velocities for all joints from new position samples.
     *
     * @param positions Array of size SJC_JOINTS_SIZE with current angular positions [rad]
     * @param dt Time since last update [s]
     * @param velocities Array of size SJC_JOINTS_SIZE to store computed filtered velocities [rad/s] (can be NULL)
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise
     */
    eSjcReturnType_t VelocityMeasurement_updateAll(const float positions[SJC_JOINTS_SIZE], const float dt, float *velocities);

    /**
     * @brief Retrieves the filtered angular velocity for a specific joint.
     *
     * @param joint Joint identifier
     * @param velocity Pointer to store filtered angular velocity [rad/s]
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise
     */
    eSjcReturnType_t VelocityMeasurement_get(const eSjcJoints_t joint, float *velocity);

    /**
     * @brief Retrieves filtered angular velocities for all joints in a single call.
     *
     * @param velocities Array of size SJC_JOINTS_SIZE to store filtered velocities [rad/s]
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise
     */
    eSjcReturnType_t VelocityMeasurement_getAll(float velocities[SJC_JOINTS_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__VELOCITY_MEASUREMENT_H_ */
