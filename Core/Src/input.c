/*
 * input.c
 *
 *  Created on: Jan 19, 2024
 *      Author: nkre
 */

#include <Input.h>
#include <Maps.h>


//timing Variables!
uint32_t tInputsTimer=0, tUpButtonTimer=0, tUpButtonStuckTimmer=0, tDnButtonTimer=0, tDnButtonStuckTimmer=0, tButtonATimer=0, tButtonBTimer=0, tButtonCTimer=0, tButtonDTimer=0, tButtonETimer=0, tButtonFTimer=0;

// CAN
volatile uint8_t BDO01Demand, BDO03Demand, BDO02Demand, BDO04Demand;
volatile uint8_t NCANErrorCount;
volatile uint16_t NCanGetRxErrorCount=0;

volatile uint8_t NAdcBufferSide;

//Private Functions Declaration
uint16_t MyHalfBufferAverage(uint16_t *buffer, uint16_t halfsize, uint8_t side, uint8_t offset);


void ReadInputs(InputStruct *inputs){

	tInputsTimer = HAL_GetTick();

	// ---------------------------------------------------------------------------------------------------
	//Analog & Digital Inputs

	//ADC Averaging
	inputs->NADCChannel01Raw = MyHalfBufferAverage(adcRawValue, ADC_BUFFER_HALF_SIZE, NAdcBufferSide, 0);
	inputs->NADCChannel02Raw = MyHalfBufferAverage(adcRawValue, ADC_BUFFER_HALF_SIZE, NAdcBufferSide, 1);
	inputs->NADCChannel03Raw = MyHalfBufferAverage(adcRawValue, ADC_BUFFER_HALF_SIZE, NAdcBufferSide, 2);
	inputs->NADCChannel04Raw = MyHalfBufferAverage(adcRawValue, ADC_BUFFER_HALF_SIZE, NAdcBufferSide, 3);

	//Voltage Conversion
	inputs->VSIUAnalog01 = (float)(inputs->NADCChannel01Raw * 3.3 / 4095.0);
	inputs->VSIUAnalog02 = (float)(inputs->NADCChannel02Raw * 3.3 / 4095.0);
	inputs->VSIUAnalog03 = (float)(inputs->NADCChannel03Raw * 3.3 / 4095.0);
	inputs->VSIUAnalog04 = (float)(inputs->NADCChannel04Raw * 3.3 / 4095.0);

	//Digital Read (we invert the logic to read 1 when it goes to GND, because of the pull ups)
	inputs->BSIUDIN01 = !HAL_GPIO_ReadPin(DIN01_GPIO_Port, DIN01_Pin);
	inputs->BSIUDIN02 = !HAL_GPIO_ReadPin(DIN02_GPIO_Port, DIN02_Pin);
	inputs->BSIUDIN03 = !HAL_GPIO_ReadPin(DIN03_GPIO_Port, DIN03_Pin);
	inputs->BSIUDIN04 = !HAL_GPIO_ReadPin(DIN04_GPIO_Port, DIN04_Pin);
	inputs->BSIUDIN05 = !HAL_GPIO_ReadPin(DIN05_GPIO_Port, DIN05_Pin);
	inputs->BSIUDIN06 = !HAL_GPIO_ReadPin(DIN06_GPIO_Port, DIN06_Pin);
	inputs->BSIUDIN07 = !HAL_GPIO_ReadPin(DIN07_GPIO_Port, DIN07_Pin);
	inputs->BSIUDIN08 = !HAL_GPIO_ReadPin(DIN08_GPIO_Port, DIN08_Pin);

	// Digital Outputs
	inputs->BSIUDO01Demand = BDO01Demand;
	inputs->BSIUDO02Demand = BDO02Demand;
	inputs->BSIUDO03Demand = BDO03Demand;
	inputs->BSIUDO04Demand = BDO04Demand;

	// ---------------------------------------------------------------------------------------------------
	//Clutch Paddle

	inputs->VrClutchPaddle = inputs->VSIUAnalog01;

	//Mapping
	inputs->BrClutchPaddleInError= My2DMapInterpolate(CLUTCH_PADDLE_MAP_SIZE, rClutchMap, inputs->VrClutchPaddle, &(inputs->rClutchPaddleRaw), VrCLUTCH_MARGIN_MIN, VrCLUTCH_MARGIN_MAX);

	// Conversion from Float to int8_t
	inputs->rClutchPaddle = (int8_t)round(inputs->rClutchPaddleRaw);

	//Clamping
	inputs->rClutchPaddle = CLAMP(inputs->rClutchPaddle, rCLUTCH_PADDLE_MIN, rCLUTCH_PADDLE_MAX);

	// ---------------------------------------------------------------------------------------------------
	// PCB Supply Voltage Conditioning

	inputs->VSupply = inputs->VSIUAnalog04 * VSUPPLY_DIVIDER_GAIN;

	// ---------------------------------------------------------------------------------------------------
	// SIU Inputs

	// Up Shift Button
	if(inputs->BSIUDIN01 && tUpButtonTimer < tInputsTimer && !inputs->BUpShiftButtonState) {
		inputs->BUpShiftButtonState = 1;
		inputs->BUpShiftButtonPressed = 1;
		tUpButtonTimer = tInputsTimer;
		tUpButtonTimer += UP_BUTTON_DEBOUNCE;
	}
	else if(!inputs->BSIUDIN01 && inputs->BUpShiftButtonState) {
		inputs->BUpShiftButtonState = 0;
		inputs->BUpShiftButtonPressed = 0;
		tUpButtonStuckTimmer = tInputsTimer;
	}
	// Auto reset Strategy
	if(inputs->BUpShiftButtonPressed && (tUpButtonTimer + UP_BUTTON_RESET_TIMEOUT) < tInputsTimer) {
		inputs->BUpShiftButtonPressed = 0;
	}
	// Stuck Detection
	if(inputs->BUpShiftButtonState && (tUpButtonTimer + UP_BUTTON_STUCK_TIMEOUT) < tInputsTimer) {
		inputs->BUpShiftButtonInError = 1;
	}
	else if(!inputs->BUpShiftButtonState && (tUpButtonStuckTimmer + UP_BUTTON_STUCK_TIMEOUT) < tInputsTimer) {
		inputs->BUpShiftButtonInError = 0;
	}

	// Down Shift Button
	if(inputs->BSIUDIN02 && tDnButtonTimer < tInputsTimer && !inputs->BDnShiftButtonState) {
		inputs->BDnShiftButtonState = 1;
		inputs->BDnShiftButtonPressed = 1;
		tDnButtonTimer = tInputsTimer;
		tDnButtonTimer += DN_BUTTON_DEBOUNCE;
	}
	else if(!inputs->BSIUDIN02 && inputs->BDnShiftButtonState) {
		inputs->BDnShiftButtonState = 0;
		inputs->BDnShiftButtonPressed = 0;
	}
	// Auto reset Strategy
	if(inputs->BDnShiftButtonPressed && (tDnButtonTimer + DN_BUTTON_RESET_TIMEOUT) < tInputsTimer) {
		inputs->BDnShiftButtonPressed = 0;
	}
	// Stuck Detection
	if(inputs->BDnShiftButtonState && (tDnButtonTimer + DN_BUTTON_STUCK_TIMEOUT) < tInputsTimer) {
		inputs->BDnShiftButtonInError = 1;
	}
	else if(!inputs->BDnShiftButtonState && (tDnButtonStuckTimmer + DN_BUTTON_STUCK_TIMEOUT) < tInputsTimer) {
		inputs->BDnShiftButtonInError = 0;
	}

	// Button A
	if(inputs->BSIUDIN03 && tButtonATimer < tInputsTimer && !inputs->BButtonAState) {
		inputs->BButtonAState = 1;
		inputs->BButtonAPressed = 1;
		tButtonATimer = tInputsTimer;
		tButtonATimer += BUTTON_A_DEBOUNCE;
	}
	else if(!inputs->BSIUDIN03 && inputs->BButtonAState) {
		inputs->BButtonAState = 0;
		inputs->BButtonAPressed = 0;
	}

	// Button B
	if(inputs->BSIUDIN04 && tButtonBTimer < tInputsTimer && !inputs->BButtonBState) {
		inputs->BButtonBState = 1;
		inputs->BButtonBPressed = 1;
		tButtonBTimer = tInputsTimer;
		tButtonBTimer += BUTTON_B_DEBOUNCE;
	}
	else if(!inputs->BSIUDIN04 && inputs->BButtonBState) {
		inputs->BButtonBState = 0;
		inputs->BButtonBPressed = 0;
	}

	// Button C
	if(inputs->BSIUDIN05 && tButtonCTimer < tInputsTimer && !inputs->BButtonCState) {
		inputs->BButtonCState = 1;
		inputs->BButtonCPressed = 1;
		tButtonCTimer = tInputsTimer;
		tButtonCTimer += BUTTON_C_DEBOUNCE;
	}
	else if(!inputs->BSIUDIN05 && inputs->BButtonCState) {
		inputs->BButtonCState = 0;
		inputs->BButtonCPressed = 0;
	}

	// Button D
	if(inputs->BSIUDIN06 && tButtonDTimer < tInputsTimer && !inputs->BButtonDState) {
		inputs->BButtonDState = 1;
		inputs->BButtonDPressed = 1;
		tButtonDTimer = tInputsTimer;
		tButtonDTimer += BUTTON_D_DEBOUNCE;
	}
	else if(!inputs->BSIUDIN06 && inputs->BButtonDState) {
		inputs->BButtonDState = 0;
		inputs->BButtonDPressed = 0;
	}

	// Button E
	if(inputs->BSIUDIN07 && tButtonETimer < tInputsTimer && !inputs->BButtonEState) {
		inputs->BButtonEState = 1;
		inputs->BButtonEPressed = 1;
		tButtonETimer = tInputsTimer;
		tButtonETimer += BUTTON_E_DEBOUNCE;
	}
	else if(!inputs->BSIUDIN07 && inputs->BButtonEState) {
		inputs->BButtonEState = 0;
		inputs->BButtonEPressed = 0;
	}

	// Button F
	if(inputs->BSIUDIN08 && tButtonFTimer < tInputsTimer && !inputs->BButtonFState) {
		inputs->BButtonFState = 1;
		inputs->BButtonFPressed = 1;
		tButtonFTimer = tInputsTimer;
		tButtonFTimer += BUTTON_F_DEBOUNCE;
	}
	else if(!inputs->BSIUDIN08 && inputs->BButtonFState) {
		inputs->BButtonFState = 0;
		inputs->BButtonFPressed = 0;
	}

	// ---------------------------------------------------------------------------------------------------
}


void InitInputs(void) {
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcRawValue, ADC_BUFFER_SIZE);
}


uint16_t MyHalfBufferAverage(uint16_t *buffer, uint16_t halfsize, uint8_t side, uint8_t offset) {

	uint32_t Accumulator = 0;
	uint16_t SideOffset = (side == 1 ? halfsize : 0);
	uint16_t maxArrayIndex = halfsize / ADC_NUMBER_OF_CHANNELS;

 	for(uint16_t i=0; i< maxArrayIndex; i++) {
		Accumulator += buffer[(i * ADC_NUMBER_OF_CHANNELS) + offset + SideOffset];
	}

	Accumulator /= maxArrayIndex;
	return (uint16_t)Accumulator;

}

void CAN_RX(CAN_HandleTypeDef *hcan, uint32_t RxFifo) {

	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxBuffer[8];

	if(HAL_CAN_GetRxMessage(hcan, RxFifo, &RxHeader, RxBuffer) != HAL_OK) {
		NCanGetRxErrorCount++;
		return;
	}

	 //Don't forget to add and enable filters for each message
	switch(RxHeader.StdId) {

	 case SIU_RX_ID :

		 BDO01Demand = (RxBuffer[0] >> 0) & 0x01;
		 BDO02Demand = (RxBuffer[0] >> 1) & 0x01;
		 BDO03Demand = (RxBuffer[0] >> 2) & 0x01;
		 BDO04Demand = (RxBuffer[0] >> 3) & 0x01;

		 break;

	 default:
		 break;
	 }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RX(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RX(hcan, CAN_RX_FIFO1);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	NCANErrorCount++;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	// we enter here every time ADC_BUFFER_SIZE/2 samples have been moved to the adcRawValue buffer by the DMA

	if(hadc == &hadc1) {
		NAdcBufferSide ^= 1;	// changes from 0 to 1
	}
}
