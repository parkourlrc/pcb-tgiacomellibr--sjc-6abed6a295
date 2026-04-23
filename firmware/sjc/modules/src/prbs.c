/*
 * prbs.c
 *
 *  Created on: Sep 9, 2023
 *      Author: Tiago Giacomelli
 *
 *  Implementation of Pseudo-Random Binary Sequence (PRBS) generator.
 */

// HAL includes ------------------------------------------------------ //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module includes --------------------------------------------------- //
// ------------------------------------------------------------------- //
#include "modules/include/prbs.h"
#include "system_types/include/helpers.h"
// ------------------------------------------------------------------- //

// Module defines ---------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module data structures -------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Module variables -------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Private function prototypes ---------------------------------------- //
// ------------------------------------------------------------------- //
static uint32_t PRBS_get_taps(int num_bits);
static inline uint32_t PRBS_feedback(uint32_t lfsr, uint32_t taps);
// ------------------------------------------------------------------- //

// Public functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
void PRBS_init(PRBS_Channel *ch, uint32_t seed, int num_bits)
{
    SJC_RETURN_IF_NULL_LOG_VOID(ch, "PRBS_init: ch");

    if (num_bits <= 0 || num_bits > 31)
    {
        num_bits = 5;
    }

    uint32_t max_val = (1u << num_bits);

    if (seed == 0u || seed >= max_val)
    {
        seed = 1u;
    }

    ch->seed = seed;
    ch->lfsr = seed;
    ch->num_bits = num_bits;
    ch->taps_mask = PRBS_get_taps(num_bits);
    ch->end_sequence = 0u;
}

uint32_t PRBS_next(PRBS_Channel *ch)
{
    uint32_t fb = PRBS_feedback(ch->lfsr, ch->taps_mask);

    ch->lfsr = (ch->lfsr >> 1) | (fb << (ch->num_bits - 1));

    if (ch->lfsr == ch->seed)
    {
        ch->end_sequence = 1u;
    }

    return (ch->lfsr >> (ch->num_bits - 1)) & 1u;
}

uint8_t PRBS_getEoS(PRBS_Channel *ch)
{
    uint8_t flag = ch->end_sequence;
    ch->end_sequence = 0u;
    return flag;
}

// ------------------------------------------------------------------- //

// Private functions ------------------------------------------------- //
// ------------------------------------------------------------------- //

/**
 * @brief Select primitive polynomial taps for a given LFSR length.
 * @param num_bits LFSR register length.
 * @return Tap mask for maximal-length sequence.
 */
static uint32_t PRBS_get_taps(int num_bits)
{
    switch (num_bits)
    {
        case 5:  return 0x12u;    // x^5 + x^2 + 1
        case 6:  return 0x21u;    // x^6 + x^1 + 1
        case 7:  return 0x41u;    // x^7 + x^1 + 1
        case 8:  return 0x8Eu;    // x^8 + x^6 + x^5 + x^4 + 1
        case 15: return 0x4001u;  // x^15 + x^1 + 1
        case 16: return 0x8016u;  // x^16 + x^14 + x^13 + x^11 + 1
        default: return 0x12u;    // fallback: 5-bit taps
    }
}

/**
 * @brief Compute parity feedback bit from LFSR state and tap mask.
 * @param lfsr Current LFSR state.
 * @param taps Tap mask.
 * @return Feedback bit (0 or 1).
 */
static inline uint32_t PRBS_feedback(uint32_t lfsr, uint32_t taps)
{
    uint32_t x = lfsr & taps;
    uint32_t parity = 0u;
    while (x)
    {
        parity ^= (x & 1u);
        x >>= 1;
    }
    return parity;
}
// ------------------------------------------------------------------- //
