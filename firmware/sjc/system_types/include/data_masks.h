/*
 * data_structures.h
 *
 *  Created on: Sep 17, 2023
 *      Author: tiago
 */

#ifndef SJC__SYSTEM_TYPES__DATA_MASKS_H_
#define SJC__SYSTEM_TYPES__DATA_MASKS_H_

// System includes --------------------------------------------------- //
// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

// Definitions includes ---------------------------------------------- //
// ------------------------------------------------------------------- //

typedef enum {
    switchA_msk = (1UL << 0),
    switchB_msk = (1UL << 1),
    iso_inputA_msk = (1UL << 8),
    iso_inputB_msk = (1UL << 9),
} eSjcDigitalInputMasks;

typedef enum {
    outputA_msk = (1UL << 0),
    outputB_msk = (1UL << 1)
} eSjcDigitalOutputMasks;

#endif /* SJC__SYSTEM_TYPES__DATA_MASKS_H_ */
