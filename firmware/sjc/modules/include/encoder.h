/**
 * @file encoder.h
 * @brief Quadrature encoder interface for joint position sensing.
 *
 * Provides thread-safe access to encoder counters and angular displacement for each joint.
 * Supports encoder reset, enable/disable, resolution configuration, and real-time reading.
 *
 * Author: Tiago
 * Date: Sep 9, 2023
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
     * @brief Retrieves the angular displacement (in radians) for a joint.
     *
     * @param Joint Joint identifier
     * @param angular_dp Pointer to store angular displacement
     * @return SJC_RET_OK if successful, SJC_RET_NOK otherwise
     */
    eSjcReturnType_t Encoder_getAngularDisplacement(eSjcJoints_t Joint, float *angular_dp);

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
