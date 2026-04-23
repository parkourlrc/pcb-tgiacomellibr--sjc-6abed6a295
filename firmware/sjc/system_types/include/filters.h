/*
 * filters.h
 *
 *  Created on: Nov 17, 2024
 *      Author: Tiago Giacomelli
 *
 *  Box (batch average) and exponential filters for embedded applications.
 *  Provides simple, low-memory filters designed for real-time
 *  signal conditioning and noise reduction in microcontroller-based systems.
 *
 *   - BoxFilter: batch average filter with configurable window size.
 *     Accumulates N samples, computes the mean, then resets.
 *   - ExponentialFilter: first-order IIR filter with configurable cutoff.
 */

#ifndef SJC__SYSTEM_TYPES__FILTERS_H
#define SJC__SYSTEM_TYPES__FILTERS_H

#include <stdbool.h>
#include <stdint.h>

#define BOX_FILTER_MAX_SIZE 64 ///< Maximum window size to bound memory use in static buffers

// -------------------------------------------------------------------
// Box Filter Data Structure
// -------------------------------------------------------------------
typedef struct
{
    float sum;            ///< Accumulated sum of samples
    uint16_t count;       ///< Current number of valid samples
    uint16_t window_size; ///< Number of samples in the averaging window
} BoxFilter_t;

// -------------------------------------------------------------------
// Exponential Filter Data Structure
// -------------------------------------------------------------------
typedef struct
{
    float alpha;  ///< Filter coefficient (0 < alpha <= 1)
    float y_prev; ///< Last output value
    float fs;     ///< Sampling frequency in Hz
    float fc;     ///< Cutoff frequency in Hz
} ExponentialFilter_t;

// -------------------------------------------------------------------
// Box Filter API
// -------------------------------------------------------------------

/**
 * @brief Initializes a box (batch average) filter.
 *
 * @param filter Pointer to the BoxFilter_t structure.
 * @param window_size Number of samples for the averaging window.
 */
void BoxFilter_init(BoxFilter_t *filter, uint16_t window_size);

/**
 * @brief Adds a new sample to the box filter.
 *
 * Samples beyond the window size are silently discarded.
 *
 * @param filter Pointer to the BoxFilter_t structure.
 * @param x New input sample.
 */
void BoxFilter_add_sample(BoxFilter_t *filter, float x);

/**
 * @brief Checks if the box filter has accumulated a full window of samples.
 *
 * @param filter Pointer to the BoxFilter_t structure.
 * @return true if ready to compute average, false otherwise.
 */
bool BoxFilter_ready(BoxFilter_t *filter);

/**
 * @brief Gets the average of the accumulated samples without resetting.
 *
 * @param filter Pointer to the BoxFilter_t structure.
 * @return Average of the samples, or 0.0f if no samples.
 */
float BoxFilter_get_average(BoxFilter_t *filter);

/**
 * @brief Gets the average of the accumulated samples and resets the filter.
 *
 * @param filter Pointer to the BoxFilter_t structure.
 * @return Average of the samples, or 0.0f if no samples.
 */
float BoxFilter_get_average_and_reset(BoxFilter_t *filter);

// -------------------------------------------------------------------
// Exponential Filter API
// -------------------------------------------------------------------

/**
 * @brief Initializes an exponential filter with zero initial value.
 *
 * @param filter Pointer to the ExponentialFilter_t structure.
 * @param fc Cutoff frequency in Hz.
 * @param fs Sampling frequency in Hz.
 */
void ExponentialFilter_init(ExponentialFilter_t *filter, float fc, float fs);

/**
 * @brief Initializes an exponential filter with a defined initial output value.
 *
 * @param filter Pointer to the ExponentialFilter_t structure.
 * @param fc Cutoff frequency in Hz.
 * @param fs Sampling frequency in Hz.
 * @param initial_value Initial output value.
 */
void ExponentialFilter_init_with_value(ExponentialFilter_t *filter, float fc, float fs, float initial_value);

/**
 * @brief Forces the internal state of the exponential filter to a specific value.
 *
 * This sets the last output (y_prev) directly, without applying filtering.
 * Useful for realigning the filter state after external averaging (e.g. BoxFilter).
 *
 * @param filter Pointer to the ExponentialFilter_t structure.
 * @param value Value to set as the current filter state.
 */
void ExponentialFilter_force_set(ExponentialFilter_t *filter, float value);

/**
 * @brief Resets the internal state of the filter (output = 0).
 *
 * @param filter Pointer to the ExponentialFilter_t structure.
 */
void ExponentialFilter_reset(ExponentialFilter_t *filter);

/**
 * @brief Sets a new cutoff frequency and updates alpha accordingly.
 *
 * @param filter Pointer to the ExponentialFilter_t structure.
 * @param fc New cutoff frequency in Hz.
 */
void ExponentialFilter_set_cutoff(ExponentialFilter_t *filter, float fc);

/**
 * @brief Sets a new sampling frequency and updates alpha accordingly.
 *
 * @param filter Pointer to the ExponentialFilter_t structure.
 * @param fs New sampling frequency in Hz.
 */
void ExponentialFilter_set_sampling_frequency(ExponentialFilter_t *filter, float fs);

/**
 * @brief Computes the alpha coefficient based on cutoff and sampling frequencies.
 *
 * @param fc Cutoff frequency in Hz.
 * @param fs Sampling frequency in Hz.
 * @return Computed alpha coefficient.
 */
float ExponentialFilter_compute_alpha(float fc, float fs);

/**
 * @brief Updates the exponential filter with a new input sample.
 *
 * @param filter Pointer to the ExponentialFilter_t structure.
 * @param x New input sample.
 * @return Filtered output.
 */
float ExponentialFilter_update(ExponentialFilter_t *filter, float x);

#endif // SJC__SYSTEM_TYPES__FILTERS_H
