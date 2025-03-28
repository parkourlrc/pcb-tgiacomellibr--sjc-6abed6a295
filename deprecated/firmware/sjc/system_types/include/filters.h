#ifndef SJC__SYSTEM_TYPES__FILTERS_H
#define SJC__SYSTEM_TYPES__FILTERS_H

#include "stdint.h"

#define BOX_FILTER_MAX_SIZE 64 // Safety margin to prevent overflow

typedef struct
{
    float sum;
    uint16_t count;
    uint16_t window_size;
} BoxFilter_t;

typedef struct
{
    float alpha;
    float y_prev;
    float fs;
} ExponentialFilter_t;

/**
 * @brief Initializes the box (mean) filter
 * @param filter Pointer to the filter structure
 * @param window_size Number of samples for the averaging window
 */
void BoxFilter_init(BoxFilter_t *filter, uint16_t window_size);

/**
 * @brief Adds a new sample to the box filter
 * @param filter Pointer to the filter structure
 * @param x New input sample
 */
void BoxFilter_add_sample(BoxFilter_t *filter, float x);

/**
 * @brief Returns true if the box filter has accumulated enough samples
 * @param filter Pointer to the filter structure
 * @return 1 if ready, 0 otherwise
 */
int BoxFilter_ready(BoxFilter_t *filter);

/**
 * @brief Returns the average and resets the box filter
 * @param filter Pointer to the filter structure
 * @return The average value of the accumulated samples
 */
float BoxFilter_get_average_and_reset(BoxFilter_t *filter);

/**
 * @brief Initializes the exponential filter with zero initial value
 * @param filter Pointer to the filter structure
 * @param fc Cutoff frequency in Hz
 * @param fs Sampling frequency in Hz
 */
void ExponentialFilter_init(ExponentialFilter_t *filter, float fc, float fs);

/**
 * @brief Initializes the exponential filter with an initial value
 * @param filter Pointer to the filter structure
 * @param fc Cutoff frequency in Hz
 * @param fs Sampling frequency in Hz
 * @param initial_value Initial value of the filtered signal
 */
void ExponentialFilter_init_with_value(ExponentialFilter_t *filter, float fc, float fs, float initial_value);

/**
 * @brief Resets the filter's internal state
 * @param filter Pointer to the filter structure
 */
void ExponentialFilter_reset(ExponentialFilter_t *filter);

/**
 * @brief Updates the exponential filter cutoff frequency
 * @param filter Pointer to the filter structure
 * @param fc New cutoff frequency in Hz
 */
void ExponentialFilter_set_cutoff(ExponentialFilter_t *filter, float fc);

/**
 * @brief Applies the exponential filter to a new sample
 * @param filter Pointer to the filter structure
 * @param x New input sample
 * @return Filtered output
 */
float ExponentialFilter_update(ExponentialFilter_t *filter, float x);

#endif // SJC__SYSTEM_TYPES__FILTERS_H
