#ifndef SJC__MODULES__CURRENT_OBSERVER_H_
#define SJC__MODULES__CURRENT_OBSERVER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "system_types/include/types.h"
#include <stdbool.h>

#define CURRENT_OBSERVER_STATE_SIZE 2

    /**
     * @brief Structure representing a second-order state observer for current and its derivative.
     */
    typedef struct
    {
        float A[CURRENT_OBSERVER_STATE_SIZE][CURRENT_OBSERVER_STATE_SIZE]; ///< System matrix
        float B[CURRENT_OBSERVER_STATE_SIZE];                              ///< Input matrix
        float C[CURRENT_OBSERVER_STATE_SIZE];                              ///< Output matrix
        float L[CURRENT_OBSERVER_STATE_SIZE];                              ///< Observer gain
        float x_hat[CURRENT_OBSERVER_STATE_SIZE];                          ///< Estimated states [current, current_derivative]
        bool initialized;                                                  ///< Indicates whether the observer is active
    } CurrentObserver_t;

    /**
     * @brief Initializes the current observer with the provided parameters.
     *
     * @param observer Pointer to the observer structure.
     * @param A State transition matrix.
     * @param B Input matrix.
     * @param C Output matrix.
     * @param L Observer gain vector.
     */
    void CurrentObserver_init(CurrentObserver_t *observer,
                              const float A[CURRENT_OBSERVER_STATE_SIZE][CURRENT_OBSERVER_STATE_SIZE],
                              const float B[CURRENT_OBSERVER_STATE_SIZE],
                              const float C[CURRENT_OBSERVER_STATE_SIZE],
                              const float L[CURRENT_OBSERVER_STATE_SIZE]);

    /**
     * @brief Updates the internal state estimate using control input and measured output.
     *
     * @param observer Pointer to the observer.
     * @param u Control input (e.g., voltage).
     * @param y_meas Measured output (e.g., current).
     */
    void CurrentObserver_update(CurrentObserver_t *observer, float u, float y_meas);

    /**
     * @brief Resets the observer state and re-enables it.
     *
     * This function clears the internal state and reactivates the observer.
     * It should be called after fault recovery or system reinitialization.
     *
     * @param observer Pointer to the observer.
     */
    void CurrentObserver_reset(CurrentObserver_t *observer);

    /**
     * @brief Returns the estimated current.
     *
     * @param observer Pointer to the observer.
     * @return Estimated current (x_hat[0]).
     */
    static inline float CurrentObserver_get_current(const CurrentObserver_t *observer)
    {
        return observer->x_hat[0];
    }

    /**
     * @brief Returns the estimated derivative of the current.
     *
     * @param observer Pointer to the observer.
     * @return Estimated current derivative (x_hat[1]).
     */
    static inline float CurrentObserver_get_current_derivative(const CurrentObserver_t *observer)
    {
        return observer->x_hat[1];
    }

#ifdef __cplusplus
}
#endif

#endif // SJC__MODULES__CURRENT_OBSERVER_H_
