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
    switchA_msk = 0UL,
    switchB_msk,
    iso_inputA_msk,
    iso_inputB_msk
} eSjcDigitalInputMasks;

typedef enum
{
    outputA_msk = 0UL,
    outputB_msk
} eSjcDigitalOutputMasks;

#endif /* SJC__SYSTEM_TYPES__DATA_MASKS_H_ */