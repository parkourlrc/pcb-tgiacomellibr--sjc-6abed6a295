/**
 * @file helpers.h
 * @brief Common math and word/byte manipulation utilities for embedded systems.
 *
 * This header provides macros and utility functions for value clamping,
 * unit conversions, and manipulation of numeric values for transmission
 * or processing (Modbus register order: high 16-bit word first).
 *
 * All comments and documentation in English.
 */

#ifndef SJC__SYSTEM_TYPES__HELPERS_H_
#define SJC__SYSTEM_TYPES__HELPERS_H_

#include <math.h>
#include <stdint.h>

/* Optional C++ compatibility */
#ifdef __cplusplus
extern "C"
{
#endif

/* ============================== */
/* Conversion constants           */
/* ============================== */
/** @name Conversion constants */
/** @{ */
#define RAD_TO_DEG (180.0f / M_PI)                  ///< Convert radians to degrees
#define DEG_TO_RAD (M_PI / 180.0f)                  ///< Convert degrees to radians
#define DEF_RAD_PER_SEC_TO_RPM (30.0f / M_PI)       ///< Convert rad/s to RPM
#define DEF_RPM_TO_RAD_PER_SEC (M_PI / 30.0f)       ///< Convert RPM to rad/s
#define CONV_MILLI_TO_BASE_FLOAT(X) ((X) / 1000.0f) ///< Convert milli-units to float
#define CONV_PERIOD_TO_RATE(X) ((1.0f) / (X))       ///< Convert period to rate

#define MS_TO_SEC (1.0f / 1000.0f)           ///< Milliseconds to seconds factor
#define CONVERT_MS_TO_SEC(X) ((X) / 1000.0f) ///< Milliseconds to seconds
/** @} */

/* ============================== */
/* Integer comparison macros      */
/* ============================== */
/** @name Integer comparison macros (fully parenthesized) */
/** @{ */
#define SJC_INT_MIN(a, b) (((int32_t)(a) < (int32_t)(b)) ? (int32_t)(a) : (int32_t)(b))
#define SJC_INT_MAX(a, b) (((int32_t)(a) > (int32_t)(b)) ? (int32_t)(a) : (int32_t)(b))
#define SJC_UINT_MIN(a, b) (((uint32_t)(a) < (uint32_t)(b)) ? (uint32_t)(a) : (uint32_t)(b))
#define SJC_UINT_MAX(a, b) (((uint32_t)(a) > (uint32_t)(b)) ? (uint32_t)(a) : (uint32_t)(b))

    extern int DebugOutput_systemPrintf(const char *format, ...);
/** @} */

/* ============================== */
/* NULL-check helpers (with log)  */
/* ============================== */
/*
 * NOTE: These macros call DebugOutput_systemPrintf(). Ensure its declaration is visible
 * where they are used; otherwise provide a stub or include the appropriate header.
 */

/**
 * @brief Check for NULL pointer and return SJC_RET_NOK with log.
 * Use this macro when the function returns a status code (eSjcReturnType_t).
 */
#define SJC_RETURN_IF_NULL_LOG_RET(ptr, msg)                    \
    do                                                          \
    {                                                           \
        if ((ptr) == NULL)                                      \
        {                                                       \
            DebugOutput_systemPrintf("\r\n[NULL] %s\n", (msg)); \
            return SJC_RET_NOK;                                 \
        }                                                       \
    } while (0)

/**
 * @brief Check for NULL pointer and return void with log.
 * Use this macro when the function returns void.
 */
#define SJC_RETURN_IF_NULL_LOG_VOID(ptr, msg)                   \
    do                                                          \
    {                                                           \
        if ((ptr) == NULL)                                      \
        {                                                       \
            DebugOutput_systemPrintf("\r\n[NULL] %s\n", (msg)); \
            return;                                             \
        }                                                       \
    } while (0)

    /* ============================== */
    /* Clamp function declarations    */
    /* ============================== */
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

    /* ============================== */
    /* Word/byte composition utils    */
    /* ============================== */
    /** @name Modbus register packing (high 16-bit word first, then low) */
    /** @{ */
    /**
     * @brief Split a uint32_t into two 16-bit words (Modbus register order).
     * @param src 32-bit input value.
     * @param dst Out array of two words: dst[0] = high word, dst[1] = low word.
     */
    void compose_ui16_array_from_ui32(const uint32_t src, uint16_t *dst);

    /**
     * @brief Join two 16-bit words into a uint32_t (Modbus register order).
     * @param src In array where src[0] = high word, src[1] = low word.
     * @param dst Resulting 32-bit value.
     */
    void compose_ui32_from_ui16_array(const uint16_t *src, uint32_t *dst);

    /**
     * @brief Split a float into two 16-bit words (Modbus register order).
     * @param src 32-bit IEEE-754 float.
     * @param dst Out array: dst[0] = high word, dst[1] = low word.
     */
    void compose_ui16_array_from_float(const float src, uint16_t *dst);

    /**
     * @brief Join two 16-bit words into a float (Modbus register order).
     * @param src In array where src[0] = high word, src[1] = low word.
     * @param dst Resulting 32-bit IEEE-754 float.
     */
    void compose_float_from_ui16_array(const uint16_t *src, float *dst);
    /** @} */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SJC__SYSTEM_TYPES__HELPERS_H_ */
