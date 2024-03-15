/*
 * output.c
 *
 *  Created on: Jan 19, 2024
 *      Author: nkre
 */

#include <Output.h>

//CAN
uint8_t CANTxBuffer[8] = {0};

uint32_t nCanTxErrorCount=0;
uint32_t nCanOldestMailbox=4, nCanSecondOldestMailbox=2, nCanYoungestMailbox=1;


void InitOutput() {

}
void Output(InputStruct* inputs) {

	// Digital Outputs
	HAL_GPIO_WritePin(DO01_GPIO_Port, DO01_Pin, inputs->BSIUDO01Demand);
	HAL_GPIO_WritePin(DO02_GPIO_Port, DO02_Pin, inputs->BSIUDO02Demand);
	HAL_GPIO_WritePin(DO03_GPIO_Port, DO03_Pin, inputs->BSIUDO03Demand);
	HAL_GPIO_WritePin(DO04_GPIO_Port, DO04_Pin, inputs->BSIUDO04Demand);


	// CAN
	CANTxBuffer[0] = 0;
	CANTxBuffer[0] |= (inputs->BUpShiftButtonInError 	& 0x01) << 0;
	CANTxBuffer[0] |= (inputs->BDnShiftButtonInError 	& 0x01) << 1;
	CANTxBuffer[0] |= (0							  	& 0x01) << 2;
	CANTxBuffer[0] |= (0								& 0x01) << 3;
	CANTxBuffer[0] |= (0							    & 0x01) << 4;
	CANTxBuffer[0] |= (0								& 0x01) << 5;
	CANTxBuffer[0] |= (inputs->BrClutchPaddleInError	& 0x01) << 6;
	CANTxBuffer[0] |= (0								& 0x01) << 7;

	CANTxBuffer[1] = 0;
	CANTxBuffer[1] |= (inputs->BUpShiftButtonPressed 	& 0x01) << 0;
	CANTxBuffer[1] |= (inputs->BDnShiftButtonPressed 	& 0x01) << 1;
	CANTxBuffer[1] |= (inputs->BButtonAPressed  		& 0x01) << 2;
	CANTxBuffer[1] |= (inputs->BButtonBPressed 			& 0x01) << 3;
	CANTxBuffer[1] |= (inputs->BButtonCPressed   		& 0x01) << 4;
	CANTxBuffer[1] |= (inputs->BButtonDPressed			& 0x01) << 5;
	CANTxBuffer[1] |= (inputs->BButtonEPressed			& 0x01) << 6;
	CANTxBuffer[1] |= (inputs->BButtonFPressed			& 0x01) << 7;

	CANTxBuffer[2] = inputs->rClutchPaddle;

	uint16_t VSupplyCAN = (uint16_t)(inputs->VSupply * 1000);

	CANTxBuffer[3] = (VSupplyCAN & 0xFF00) >> 8;
	CANTxBuffer[4] = (VSupplyCAN & 0x00FF) >> 0;

	CAN_TX(SHIFTER_TX_ID, 8, CANTxBuffer);
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

