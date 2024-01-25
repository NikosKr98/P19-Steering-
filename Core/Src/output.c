/*
 * output.c
 *
 *  Created on: Jan 19, 2024
 *      Author: nkre
 */

#include "output.h"

void CAN_Tx(uint32_t ID, uint8_t dlc, uint8_t* data) {

	uint32_t TxMailbox;

	TxHeader.DLC = dlc;
	TxHeader.StdId = ID;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;

	if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) != HAL_OK) {
		//Error_Handler();

	}
}

void Output(ControlData* controlData) {

	if(msg_previous<current) {	// Message Send Control

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		TxData[0] = controlData->up_button;
		TxData[1] = controlData->down_button;
		TxData[2] = (controlData->clutch_position >> 8) & 0xFF;
		TxData[3] = controlData->clutch_position & 0xFF;
		msg_previous=current;
		msg_previous+=msg_interval;
		CAN_Tx(TO_SHIFTER_ID, 4, TxData);

	  }


}
