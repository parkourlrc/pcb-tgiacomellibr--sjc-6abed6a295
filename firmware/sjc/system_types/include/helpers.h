/**
 * @file helpers.h
 * @brief Common math and byte manipulation utilities for embedded systems.
 *
 * This header provides macros and utility functions for value clamping,
 * unit conversions, and memory-safe manipulation of numeric values
 * for transmission or processing.
 *
 * @author Tiago
 * @date Sep 17, 2023
 */

#ifndef SJC__SYSTEM_TYPES__HELPERS_H_
#define SJC__SYSTEM_TYPES__HELPERS_H_

#include "math.h"
#include "stdint.h"

/** @name Conversion constants */
/** @{ */
#define RAD_TO_DEG (180.0f / M_PI)                  ///< Convert radians to degrees
#define DEG_TO_RAD (M_PI / 180.0f)                  ///< Convert degrees to radians
#define DEF_RAD_PER_SEC_TO_RPM (30.0f / M_PI)       ///< Convert rad/s to RPM
#define DEF_RPM_TO_RAD_PER_SEC (M_PI / 30.0f)       ///< Convert RPM to rad/s
#define CONV_MILLI_TO_BASE_FLOAT(X) ((X) / 1000.0f) ///< Convert millivalue to float

#define MS_TO_SEC (1.0f / 1000.0f)
#define CONVERT_MS_TO_SEC(X) ((X) / 1000.0f)


/** @} */

/** @name Integer comparison macros */
/** @{ */
#define INT_MIN(a, b) ((int32_t)(a) < (int32_t)(b) ? (int32_t)(a) : (int32_t)(b))      ///< Minimum of two int32_t
#define INT_MAX(a, b) ((int32_t)(a) > (int32_t)(b) ? (int32_t)(a) : (int32_t)(b))      ///< Maximum of two int32_t
#define UINT_MIN(a, b) ((uint32_t)(a) < (uint32_t)(b) ? (uint32_t)(a) : (uint32_t)(b)) ///< Minimum of two uint32_t
#define UINT_MAX(a, b) ((uint32_t)(a) > (uint32_t)(b) ? (uint32_t)(a) : (uint32_t)(b)) ///< Maximum of two uint32_t
/** @} */

/**
 * @brief Check for NULL pointer and return SJC_RET_NOK with log.
 *
 * Use this macro when the function returns a status code (eSjcReturnType_t).
 */
#define SJC_RETURN_IF_NULL_LOG_RET(ptr, msg)              \
    do {                                                  \
        if ((ptr) == NULL) {                              \
            DebugOutput_systemPrintf("\r\n[NULL] %s\n", msg); \
            return SJC_RET_NOK;                           \
        }                                                 \
    } while (0)

/**
 * @brief Check for NULL pointer and return void with log.
 *
 * Use this macro when the function returns void.
 */
#define SJC_RETURN_IF_NULL_LOG_VOID(ptr, msg)             \
    do {                                                  \
        if ((ptr) == NULL) {                              \
            DebugOutput_systemPrintf("\r\n[NULL] %s\n", msg); \
            return;                                       \
        }                                                 \
    } while (0)

/** @name Clamp function declarations */
/** @{ */
/**
 * @brief Clamp a float value between min and max.
 */
float SJC_Helpers_RealClamp(const float var, const float min, const float max);

/**
 * @brief Clamp an int32_t value between min and max.
 */
int32_t SJC_Helpers_IntClamp(const int32_t var, const int32_t min, const int32_t max);

/**
 * @brief Clamp a uint32_t value between min and max.
 */
uint32_t SJC_Helpers_UintClamp(const uint32_t var, const uint32_t min, const uint32_t max);
/** @} */

/** @name Byte composition utilities */
/** @{ */
/**
 * @brief Convert a uint32_t to two 16-bit words in big-endian order.
 */
void compose_byte_array_from_ui32(const uint32_t src, uint16_t *dst);

/**
 * @brief Convert two 16-bit words to a uint32_t in big-endian order.
 */
void compose_ui32_from_byte_array(const uint16_t *src, uint32_t *dst);

/**
 * @brief Convert a float to two 16-bit words in big-endian order.
 */
void compose_byte_array_from_float(const float src, uint16_t *dst);

/**
 * @brief Convert two 16-bit words to a float in big-endian order.
 */
void compose_float_from_byte_array(const uint16_t *src, float *dst);
/** @} */

#endif /* SJC__SYSTEM_TYPES__HELPERS_H_ */
