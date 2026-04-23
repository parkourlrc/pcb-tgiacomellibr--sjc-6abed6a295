/*
 * filters.c
 *
 *  Created on: Mar 2025
 *      Author: Tiago Giacomelli
 *
 *  Box filter and exponential filter implementations.
 */

#include "system_types/include/filters.h"
#include "system_types/include/helpers.h"
#include <math.h>

// -------------------------------------------------------------------
// Box Filter Implementation
// -------------------------------------------------------------------

void BoxFilter_init(BoxFilter_t *filter, uint16_t window_size)
{
    SJC_RETURN_IF_NULL_LOG_VOID(filter, "BoxFilter_init");

    if (window_size > BOX_FILTER_MAX_SIZE)
    {
        window_size = BOX_FILTER_MAX_SIZE;
    }

    filter->sum = 0.0f;
    filter->count = 0;
    filter->window_size = window_size;
}

void BoxFilter_add_sample(BoxFilter_t *filter, float x)
{
    SJC_RETURN_IF_NULL_LOG_VOID(filter, "BoxFilter_add_sample");

    if (filter->count >= filter->window_size)
    {
        return;
    }

    filter->sum += x;
    filter->count++;
}

bool BoxFilter_ready(BoxFilter_t *filter)
{
    SJC_RETURN_IF_NULL_LOG_VAL(filter, "BoxFilter_ready", false);
    return (filter->count >= filter->window_size);
}

float BoxFilter_get_average(BoxFilter_t *filter)
{
    SJC_RETURN_IF_NULL_LOG_VAL(filter, "BoxFilter_get_average", 0.0f);
    return (filter->count > 0) ? (filter->sum / filter->count) : 0.0f;
}

float BoxFilter_get_average_and_reset(BoxFilter_t *filter)
{
    SJC_RETURN_IF_NULL_LOG_VAL(filter, "BoxFilter_get_average_and_reset", 0.0f);

    float avg = (filter->count > 0) ? (filter->sum / filter->count) : 0.0f;
    filter->sum = 0.0f;
    filter->count = 0;
    return avg;
}

// -------------------------------------------------------------------
// Exponential Filter Implementation
// -------------------------------------------------------------------

void ExponentialFilter_init(ExponentialFilter_t *filter, float fc, float fs)
{
    ExponentialFilter_init_with_value(filter, fc, fs, 0.0f);
}

void ExponentialFilter_init_with_value(ExponentialFilter_t *filter, float fc, float fs, float initial_value)
{
    SJC_RETURN_IF_NULL_LOG_VOID(filter, "ExponentialFilter_init_with_value");

    filter->fs = fs;
    ExponentialFilter_set_cutoff(filter, fc);
    filter->y_prev = initial_value;
}

void ExponentialFilter_force_set(ExponentialFilter_t *filter, float value)
{
    SJC_RETURN_IF_NULL_LOG_VOID(filter, "ExponentialFilter_force_set");

    filter->y_prev = value;
}

void ExponentialFilter_reset(ExponentialFilter_t *filter)
{
    SJC_RETURN_IF_NULL_LOG_VOID(filter, "ExponentialFilter_reset");

    filter->y_prev = 0.0f;
}

void ExponentialFilter_set_cutoff(ExponentialFilter_t *filter, float fc)
{
    SJC_RETURN_IF_NULL_LOG_VOID(filter, "ExponentialFilter_set_cutoff");

    filter->fc = fc;
    filter->alpha = ExponentialFilter_compute_alpha(fc, filter->fs);
}

void ExponentialFilter_set_sampling_frequency(ExponentialFilter_t *filter, float fs)
{
    SJC_RETURN_IF_NULL_LOG_VOID(filter, "ExponentialFilter_set_sampling_frequency");

    filter->fs = fs;
    filter->alpha = ExponentialFilter_compute_alpha(filter->fc, fs);
}

float ExponentialFilter_compute_alpha(float fc, float fs)
{
    if (fc <= 0.0f || fs <= 0.0f)
    {
        return 1.0f;
    }

    float ts = 1.0f / fs;
    float tau = 1.0f / (2.0f * M_PI * fc);
    return 1.0f - expf(-ts / tau);
}

float ExponentialFilter_update(ExponentialFilter_t *filter, float x)
{
    SJC_RETURN_IF_NULL_LOG_VAL(filter, "ExponentialFilter_update", 0.0f);

    if (!isfinite(x) || !isfinite(filter->alpha))
    {
        return filter->y_prev;
    }

    float y = filter->alpha * x + (1.0f - filter->alpha) * filter->y_prev;
    filter->y_prev = y;
    return y;
}
