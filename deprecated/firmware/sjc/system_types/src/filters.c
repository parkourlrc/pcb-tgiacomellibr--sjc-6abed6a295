#include "system_types/include/filters.h"
#include <math.h>

// -------------------------------------------------------------------
// Exponential Filter
// -------------------------------------------------------------------
void ExponentialFilter_init(ExponentialFilter_t *filter, float fc, float fs)
{
    filter->fs = fs;
    ExponentialFilter_set_cutoff(filter, fc);
    filter->y_prev = 0.0f;
}

void ExponentialFilter_init_with_value(ExponentialFilter_t *filter, float fc, float fs, float initial_value)
{
    filter->fs = fs;
    ExponentialFilter_set_cutoff(filter, fc);
    filter->y_prev = initial_value;
}

void ExponentialFilter_reset(ExponentialFilter_t *filter)
{
    filter->y_prev = 0.0f;
}

void ExponentialFilter_set_cutoff(ExponentialFilter_t *filter, float fc)
{
    if (fc <= 0.0f || filter->fs <= 0.0f)
    {
        filter->alpha = 1.0f; // No filtering (instant response)
        return;
    }
    float ts = 1.0f / filter->fs;
    float tau = 1.0f / (2.0f * M_PI * fc);
    filter->alpha = 1.0f - expf(-ts / tau);
}

float ExponentialFilter_update(ExponentialFilter_t *filter, float x)
{
    float y = filter->alpha * x + (1.0f - filter->alpha) * filter->y_prev;
    filter->y_prev = y;
    return y;
}

// -------------------------------------------------------------------
// Box Filter (mean of N samples)
// -------------------------------------------------------------------
void BoxFilter_init(BoxFilter_t *filter, uint16_t window_size)
{
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
    filter->sum += x;
    filter->count++;
}

int BoxFilter_ready(BoxFilter_t *filter)
{
    return (filter->count >= filter->window_size);
}

float BoxFilter_get_average_and_reset(BoxFilter_t *filter)
{
    float avg = (filter->count > 0) ? (filter->sum / filter->count) : 0.0f;
    filter->sum = 0.0f;
    filter->count = 0;
    return avg;
}
