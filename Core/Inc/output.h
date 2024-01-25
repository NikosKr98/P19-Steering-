/*
 * output.h
 *
 *  Created on: Jan 19, 2024
 *      Author: nkre
 */

#ifndef INC_OUTPUT_H_
#define INC_OUTPUT_H_

#include "stm32f1xx_hal.h"
#include "struct.h"
#include "gpio_init.h"



#define TO_SHIFTER_ID 0x310



extern CAN_HandleTypeDef hcan;
extern CAN_TxHeaderTypeDef TxHeader;
extern uint8_t TxData[8];

extern uint32_t current,msg_previous,msg_interval;



void CAN_Tx(uint32_t ID, uint8_t dlc, uint8_t* data);
void Output(ControlData* controlData);

#endif /* INC_OUTPUT_H_ */
