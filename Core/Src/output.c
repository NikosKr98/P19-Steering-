/*
 * output.c
 *
 *  Created on: Jan 19, 2024
 *      Author: nkre
 */

#include "output.h"

//CAN
uint8_t TxBuffer[8] = {0};

uint32_t nCanTxErrorCount=0;
uint32_t nCanOldestMailbox=4, nCanSecondOldestMailbox=2, nCanYoungestMailbox=1;


void InitOutput() {

}
void Output(InputStruct* inputs) {


	TxBuffer[0] = 0;
	TxBuffer[0] |= (inputs->BUpShiftButtonInError 	& 0x01) << 0;
	TxBuffer[0] |= (inputs->BDnShiftButtonInError 	& 0x01) << 1;
	TxBuffer[0] |= (inputs->BLaunchButtonInError  	& 0x01) << 2;
	TxBuffer[0] |= (inputs->BEmergencyButtonInError & 0x01) << 3;
	TxBuffer[0] |= (inputs->BAuxLeftButtonInError   & 0x01) << 4;
	TxBuffer[0] |= (inputs->BAuxRightButtonInError	& 0x01) << 5;
	TxBuffer[0] |= (inputs->BrClutchPaddleInError	& 0x01) << 6;


	TxBuffer[1] = 0;
	TxBuffer[1] |= (inputs->BUpShiftButtonPressed 	& 0x01) << 0;
	TxBuffer[1] |= (inputs->BDnShiftButtonPressed 	& 0x01) << 1;
	TxBuffer[1] |= (inputs->BLaunchButtonPressed  	& 0x01) << 2;
	TxBuffer[1] |= (inputs->BEmergencyButtonPressed & 0x01) << 3;
	TxBuffer[1] |= (inputs->BAuxLeftButtonPressed   & 0x01) << 4;
	TxBuffer[1] |= (inputs->BAuxRightButtonPressed	& 0x01) << 5;

	TxBuffer[2] = inputs->rClutchPaddle;

	uint16_t VSupplyCAN = (uint16_t)(inputs->VSupply * 1000);

	TxBuffer[3] = (VSupplyCAN & 0xFF00) >> 8;
	TxBuffer[4] = (VSupplyCAN & 0x00FF) >> 0;

	CAN_TX(SHIFTER_TX_ID,8,TxBuffer);
}


void CAN_TX(uint32_t ID, uint8_t dlc, uint8_t* data) {

	CAN_TxHeaderTypeDef CanTxHeader;
	uint32_t nCanTxMailbox;

	CanTxHeader.DLC = dlc;
	CanTxHeader.StdId = ID;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.RTR = CAN_RTR_DATA;

	uint32_t wait = __HAL_TIM_GET_COUNTER(&htim2) + CAN_TX_TIMEOUT;
	while((HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) && (__HAL_TIM_GET_COUNTER(&htim2) < wait));

	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {	// all mailboxes are still filled
		HAL_CAN_AbortTxRequest(&hcan, nCanOldestMailbox);
	}

	if(HAL_CAN_AddTxMessage(&hcan, &CanTxHeader, data, &nCanTxMailbox) != HAL_OK) {
		print("Failed to Add Message can 1\n");
		nCanTxErrorCount++;
		return;
	}

	// Mailbox aging adjustment
	if(nCanTxMailbox != nCanYoungestMailbox) {

		if(nCanTxMailbox != nCanSecondOldestMailbox) {
			nCanOldestMailbox = nCanSecondOldestMailbox;
			nCanSecondOldestMailbox = nCanYoungestMailbox;
			nCanYoungestMailbox = nCanTxMailbox;
		}
		else {
			nCanSecondOldestMailbox = nCanYoungestMailbox;
			nCanYoungestMailbox = nCanTxMailbox;
		}
	}

}

