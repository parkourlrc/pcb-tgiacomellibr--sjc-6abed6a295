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

typedef enum
{
    SWITCH_A_MASK = 0UL,
    SWITCH_B_MASK,
    ISO_INPUT_A_MASK,
    ISO_INPUT_B_MASK,
    SJC_DIGITAL_INPUT_MASK_SIZE
} eSjcDigitalInputMasks;

typedef enum
{
    OUTPUT_A_MASK = 0UL,
    OUTPUT_B_MASK,
    SJC_DIGITAL_OUTPUT_MASK_SIZE
} eSjcDigitalOutputMasks;

#endif /* SJC__SYSTEM_TYPES__DATA_MASKS_H_ */