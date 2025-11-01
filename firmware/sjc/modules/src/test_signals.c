/**
 * @file test_signals.c
 * @brief Implementation of test signal generator.
 *
 * Signals implemented:
 *   - PRBS (2.5, 5, 10 Hz)
 *   - Pulse trains (2 Hz, 4 Hz)
 *   - Sine + PRBS (sines 1, 2.5, 5 Hz plus PRBS at 10 Hz)
 *
 * Timing is based on the configured controller sampling period Ts (s),
 * ensuring consistent real-time durations regardless of loop frequency.
 *
 * Output values are normalized in [-1.0, +1.0].
 */

#include "modules/include/test_signals.h"
#include "base/include/configs.h"
#include "modules/include/prbs.h"
#include <math.h>

// ------------------------------------------------------------------- //
// Module state
// ------------------------------------------------------------------- //
static eSjcInputSignalState_t xSjcInputSignalState = INPUT_SIGNAL_STATE_INIT_CONFIG;
static eSjcInputType_t        xSjcSignalType = INPUT_TYPE_NONE;

static float    xSjcSignalTs   = 0.001f; // default 1 kHz
static float    xSjcSignalValue = 0.0f;
static float    xSjcElapsedTime = 0.0f;
static uint32_t xSjcSignalRepeatCounter = 0;
static float    xSjcTotalTime = 0.0f;

// ------------------------------------------------------------------- //
// PRBS channels
// ------------------------------------------------------------------- //
static PRBS_Channel slow_prbs;
static PRBS_Channel mid_prbs;
static PRBS_Channel fast_prbs;
static PRBS_Channel sine_prbs_ch;

static float xPrbsValueSlow = -1.0f;
static float xPrbsValueMid  = -1.0f;
static float xPrbsValueFast = -1.0f;
static float xSinePrbsValue = -1.0f;

// ------------------------------------------------------------------- //
// Private helper
// ------------------------------------------------------------------- //
static void TestSignals_setState(const eSjcInputSignalState_t state)
{
    xSjcInputSignalState = state;
}

// ------------------------------------------------------------------- //
// Public API
// ------------------------------------------------------------------- //
void TestSignals_init(void)
{
    xSjcInputSignalState = INPUT_SIGNAL_STATE_INIT_CONFIG;
    xSjcSignalType = INPUT_TYPE_NONE;

    // Init PRBS channels (7-bit LFSRs with different seeds)
    PRBS_init(&slow_prbs,     0x5Au, 7);
    PRBS_init(&mid_prbs,      0xA3u, 7);
    PRBS_init(&fast_prbs,     0x7Fu, 7);
    PRBS_init(&sine_prbs_ch,  0x55u, 7);

    xPrbsValueSlow = -1.0f;
    xPrbsValueMid  = -1.0f;
    xPrbsValueFast = -1.0f;
    xSinePrbsValue = -1.0f;

    xSjcTotalTime = 0.0f;
}

void TestSignals_reset(void)
{
    xSjcInputSignalState = INPUT_SIGNAL_STATE_INIT_CONFIG;
    xSjcSignalType = INPUT_TYPE_NONE;
    xSjcSignalValue = 0.0f;
    xSjcElapsedTime = 0.0f;
    xSjcSignalRepeatCounter = 0;
    xSjcTotalTime = 0.0f;

    xPrbsValueSlow = -1.0f;
    xPrbsValueMid  = -1.0f;
    xPrbsValueFast = -1.0f;
    xSinePrbsValue = -1.0f;
}

eSjcReturnType_t TestSignals_setConfig(const eSjcInputType_t type,
                                       const float Ts)
{
    xSjcSignalType = type;
    xSjcSignalTs   = Ts;

    xSjcSignalValue = -1.0f;
    xSjcElapsedTime = 0.0f;
    xSjcSignalRepeatCounter = 0;
    xSjcTotalTime   = 0.0f;

    TestSignals_setState(INPUT_SIGNAL_STATE_RUNNING);
    return SJC_RET_OK;
}

eSjcReturnType_t TestSignals_getState(eSjcInputSignalState_t *state)
{
    *state = xSjcInputSignalState;
    return SJC_RET_OK;
}

eSjcReturnType_t TestSignals_getValue(float *value)
{
    if (xSjcInputSignalState == INPUT_SIGNAL_STATE_FINISHED)
    {
        *value = ZERO_FLOAT;
        return SJC_RET_OK;
    }

    // Advance time
    xSjcTotalTime   += xSjcSignalTs;
    xSjcElapsedTime += xSjcSignalTs;

    switch (xSjcSignalType)
    {
    case INPUT_TYPE_PULSE_FAST: {
        const float Tpulse = 1.0f / 4.0f;   // 4 Hz
        const float half_period = Tpulse / 2.0f;

        if (xSjcElapsedTime >= half_period) {
            xSjcSignalValue *= -1.0f;
            xSjcElapsedTime = 0.0f;
            xSjcSignalRepeatCounter++;
        }
        if (xSjcSignalRepeatCounter >= 40) {
            xSjcSignalValue = ZERO_FLOAT;
            TestSignals_setState(INPUT_SIGNAL_STATE_FINISHED);
        }
    } break;

    case INPUT_TYPE_PULSE_SLOW: {
        const float Tpulse = 1.0f / 2.0f;   // 2 Hz
        const float half_period = Tpulse / 2.0f;

        if (xSjcElapsedTime >= half_period) {
            xSjcSignalValue *= -1.0f;
            xSjcElapsedTime = 0.0f;
            xSjcSignalRepeatCounter++;
        }
        if (xSjcSignalRepeatCounter >= 40) {
            xSjcSignalValue = ZERO_FLOAT;
            TestSignals_setState(INPUT_SIGNAL_STATE_FINISHED);
        }
    } break;

    case INPUT_TYPE_LOW_FREQ_PRBS: {
        const float Tbit = 1.0f / 2.5f;   // 2.5 Hz
        static uint32_t last_bit_index = 0;
        uint32_t idx = (uint32_t)(xSjcTotalTime / Tbit);
        if (idx != last_bit_index) {
            xPrbsValueSlow = (2.0f * PRBS_next(&slow_prbs)) - 1.0f;
            last_bit_index = idx;
            if (PRBS_getEoS(&slow_prbs)) {
                xSjcSignalValue = ZERO_FLOAT;
                TestSignals_setState(INPUT_SIGNAL_STATE_FINISHED);
            }
        }
        xSjcSignalValue = xPrbsValueSlow;
    } break;

    case INPUT_TYPE_MID_FREQ_PRBS: {
        const float Tbit = 1.0f / 5.0f;   // 5 Hz
        static uint32_t last_bit_index = 0;
        uint32_t idx = (uint32_t)(xSjcTotalTime / Tbit);
        if (idx != last_bit_index) {
            xPrbsValueMid = (2.0f * PRBS_next(&mid_prbs)) - 1.0f;
            last_bit_index = idx;
            if (PRBS_getEoS(&mid_prbs)) {
                xSjcSignalValue = ZERO_FLOAT;
                TestSignals_setState(INPUT_SIGNAL_STATE_FINISHED);
            }
        }
        xSjcSignalValue = xPrbsValueMid;
    } break;

    case INPUT_TYPE_FAST_FREQ_PRBS: {
        const float Tbit = 1.0f / 10.0f;   // 10 Hz
        static uint32_t last_bit_index = 0;
        uint32_t idx = (uint32_t)(xSjcTotalTime / Tbit);
        if (idx != last_bit_index) {
            xPrbsValueFast = (2.0f * PRBS_next(&fast_prbs)) - 1.0f;
            last_bit_index = idx;
            if (PRBS_getEoS(&fast_prbs)) {
                xSjcSignalValue = ZERO_FLOAT;
                TestSignals_setState(INPUT_SIGNAL_STATE_FINISHED);
            }
        }
        xSjcSignalValue = xPrbsValueFast;
    } break;

    case INPUT_TYPE_SINE_PRBS: {
        float sine01 = 0.2f * sinf(2.0f * M_PI * 1.0f  * xSjcTotalTime);
        float sine02 = 0.5f * sinf(2.0f * M_PI * 2.5f * xSjcTotalTime);
        float sine03 = 0.2f * sinf(2.0f * M_PI * 5.0f * xSjcTotalTime);

        const float Tbit = 1.0f / 10.0f;   // PRBS at 10 Hz
        static uint32_t last_bit_index = 0;
        uint32_t idx = (uint32_t)(xSjcTotalTime / Tbit);
        if (idx != last_bit_index) {
            xSinePrbsValue = (2.0f * PRBS_next(&sine_prbs_ch)) - 1.0f;
            last_bit_index = idx;
        }
        float prbs04 = 0.1f * xSinePrbsValue;

        xSjcSignalValue = sine01 + sine02 + sine03 + prbs04;

        if (xSjcTotalTime >= 10.0f) {
            xSjcSignalValue = ZERO_FLOAT;
            TestSignals_setState(INPUT_SIGNAL_STATE_FINISHED);
        }
    } break;

    default:
        xSjcSignalValue = ZERO_FLOAT;
        TestSignals_setState(INPUT_SIGNAL_STATE_FINISHED);
        break;
    }

    *value = xSjcSignalValue;
    return SJC_RET_OK;
}
