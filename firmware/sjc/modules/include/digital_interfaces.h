/*
 * motor.h
 *
 *  Created on: Sep 9, 2023
 *      Author: tiago
 */

#ifndef SJC__DIGITAL_INTERFACES_H_
#define SJC__DIGITAL_INTERFACES_H_

typedef enum {
	SJC_JOINT_SWITCH_L = 0,
	SJC_JOINT_SWITCH_R,
	SJC_ISOLATED_INPUT_A,
	SJC_ISOLATED_INPUT_B,
	SJC_INPUTS_SIZE
} eSjcDigitalInputs_t;

typedef enum {
	SJC_DIGITAL_OUTPUT_A = 0,
	SJC_DIGITAL_OUTPUT_B,
	SJC_HEARTBEAT_OUTPUT,
	SJC_OUTPUTS_SIZE
} eSjcDigitalOutputs_t;

void SJC_DigitalInterfaces_init(void);
bool SJC_DigitalInterfaces_GetInputState(eSjcDigitalInputs_t Port);
void SJC_DigitalInterfaces_EnableOutput(eSjcDigitalOutputs_t Port);
void SJC_DigitalInterfaces_DisableOutput(eSjcDigitalOutputs_t Port);
void SJC_DigitalInterfaces_ToggleOutputState(eSjcDigitalOutputs_t Port);
bool SJC_DigitalInterfaces_GetOutputState(eSjcDigitalOutputs_t Port);





#endif /* SJC__DIGITAL_INTERFACES_H_ */
