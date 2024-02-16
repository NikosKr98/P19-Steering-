/*
 * output.h
 *
 *  Created on: Jan 19, 2024
 *      Author: nkre
 */

#ifndef INC_OUTPUT_H_
#define INC_OUTPUT_H_

#include <Input.h>
#include <Utils.h>

//CAN
#define SHIFTER_TX_ID 0x310

#define CAN_TX_TIMEOUT 200		// us of CAN timeout when sending a frame

extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim2;



void CAN_TX(uint32_t ID, uint8_t dlc, uint8_t* data);
void Output(InputStruct* inputs);
void InitOutput(void);

#endif /* INC_OUTPUT_H_ */
