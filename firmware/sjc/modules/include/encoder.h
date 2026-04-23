/*
 * encoder.h
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  Quadrature encoder interface for joint position sensing.
 *  Provides thread-safe access to encoder counters and angular position
 *  for each joint. Supports encoder reset, enable/disable, resolution
 *  configuration, and real-time reading.
 */

#ifndef SJC__MODULES__ENCODER_H_
#define SJC__MODULES__ENCODER_H_

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initializes all encoder peripherals and internal state.
     *
     * Must be called during system startup before using other encoder functions.
     */
    void Encoder_init(void);

    /**
     * @brief Resets the encoder counter of a specific joint.
     *
     * @param Joint Joint identifier
     */
    void Encoder_reset(eSjcJoints_t Joint);

    /**
     * @brief Resets encoder counters for selected joints in a single mutex-protected call.
     *
     * @param joints Array of joint identifiers to reset.
     * @param count Number of joints in the array.
     */
    void Encoder_resetMultiple(const eSjcJoints_t joints[], uint8_t count);

    /**
     * @brief Resets all encoder counters in a single mutex-protected call.
     */
    void Encoder_resetAll(void);

    /**
     * @brief Enables the encoder hardware for a specific joint.
     *
     * @param Joint Joint identifier
     */
    void Encoder_enable(eSjcJoints_t Joint);

    /**
     * @brief Disables the encoder hardware for a specific joint.
     *
     * @param Joint Joint identifier
     */
    void Encoder_disable(eSjcJoints_t Joint);

    /**
     * @brief Retrieves the current counter value from a specific encoder.
     *
     * @param Joint Joint identifier
     * @param counts Pointer to store raw encoder counts
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise
     */
    eSjcReturnType_t Encoder_getCounter(eSjcJoints_t Joint, int32_t *counts);

    /**
     * @brief Retrieves the angular position (in radians) for a joint.
     *
     * @param Joint Joint identifier
     * @param position Pointer to store angular position [rad]
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise
     */
    eSjcReturnType_t Encoder_getAngularPosition(eSjcJoints_t Joint, float *position);

    /**
     * @brief Retrieves the angular position (in radians) for all joints in a single mutex-protected call.
     *
     * @param positions Array of size SJC_JOINTS_SIZE to store angular positions [rad]
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise
     */
    eSjcReturnType_t Encoder_getAllAngularPositions(float positions[SJC_JOINTS_SIZE]);

    /**
     * @brief Retrieves the resolution (CPR) for all joints in a single mutex-protected call.
     *
     * @param resolutions Array of size SJC_JOINTS_SIZE to store resolution values [CPR]
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise
     */
    eSjcReturnType_t Encoder_getAllResolutions(uint16_t resolutions[SJC_JOINTS_SIZE]);

    /**
     * @brief Gets the configured resolution (CPR) of the encoder.
     *
     * @param Joint Joint identifier
     * @param resolution Pointer to store the resolution value
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise
     */
    eSjcReturnType_t Encoder_getResolution(eSjcJoints_t Joint, uint16_t *resolution);

    /**
     * @brief Sets the resolution (CPR) of the encoder for a specific joint.
     *
     * Updates the internal conversion factor to compute angular displacement.
     * CPR = Counts Per Revolution (quadrature mode, 4x the PPR value).
     *
     * @param Joint Joint identifier
     * @param resolution New encoder resolution in counts per revolution (CPR)
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise
     */
    eSjcReturnType_t Encoder_setResolution(eSjcJoints_t Joint, const uint16_t resolution);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__ENCODER_H_ */
