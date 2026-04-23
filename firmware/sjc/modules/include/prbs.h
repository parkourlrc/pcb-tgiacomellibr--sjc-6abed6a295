/*
 * prbs.h
 *
 *  Created on: Apr 2025
 *      Author: Tiago Giacomelli
 *
 *  Pseudo-Random Binary Sequence (PRBS) generator using maximal-length LFSR.
 *
 *  Implementation details:
 *   - Uses Linear Feedback Shift Register (LFSR).
 *   - Polynomial taps are automatically selected from known primitive polynomials.
 *   - Output is binary {0,1}, taken from the MSB.
 *   - Provides end-of-sequence (EoS) detection when cycling back to the initial seed.
 */

#ifndef SJC__MODULES__PRBS_H_
#define SJC__MODULES__PRBS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief PRBS channel state */
typedef struct
{
    uint32_t seed;        ///< Initial seed (must be nonzero)
    uint32_t lfsr;        ///< Current LFSR state
    int      num_bits;    ///< Number of bits in LFSR (register length)
    uint32_t taps_mask;   ///< Mask of primitive polynomial taps
    uint8_t  end_sequence;///< Flag: end-of-sequence detected
} PRBS_Channel;

/**
 * @brief Initialize PRBS channel with given seed and register size.
 * @param ch        Pointer to PRBS channel object
 * @param seed      Initial state (must be != 0 and < 2^num_bits)
 * @param num_bits  LFSR length (e.g. 5, 6, 7, 8, 15, 16)
 */
void PRBS_init(PRBS_Channel *ch, uint32_t seed, int num_bits);

/**
 * @brief Generate next PRBS bit.
 * @param ch    Pointer to PRBS channel object
 * @return      Next binary output {0,1}
 */
uint32_t PRBS_next(PRBS_Channel *ch);

/**
 * @brief Get and clear End-of-Sequence (EoS) flag.
 * @param ch    Pointer to PRBS channel object
 * @return      1 if sequence returned to seed since last call, else 0
 */
uint8_t PRBS_getEoS(PRBS_Channel *ch);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__PRBS_H_ */
