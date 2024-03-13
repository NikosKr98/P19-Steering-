/*
 * input.c
 *
 *  Created on: Jan 19, 2024
 *      Author: nkre
 */

#include <Input.h>
#include <Maps.h>


//timing Variables!
uint32_t tInputsTimer=0, tUpButtonTimer=0, tDnButtonTimer=0, tButtonATimer=0, tButtonBTimer=0, tButtonCTimer=0, tButtonDTimer=0, tButtonETimer=0, tButtonFTimer=0;

volatile uint8_t NCANErrorCount;
volatile uint16_t NCanGetRxErrorCount=0;

volatile uint8_t NAdcBufferSide;

//Private Functions Declaration
uint16_t MyHalfBufferAverage(uint16_t *buffer, uint16_t halfsize, uint8_t side, uint8_t offset);


void ReadInputs(InputStruct *inputs){

	tInputsTimer = HAL_GetTick();

	// ---------------------------------------------------------------------------------------------------
	//Analog Inputs

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


	// ---------------------------------------------------------------------------------------------------
	//Clutch Paddle

	inputs->VrClutchPaddle = inputs->VSIUAnalog01;

	//Mapping
	inputs->BrClutchPaddleInError= My2DMapInterpolate(CLUTCH_PADDLE_MAP_SIZE, rClutchMap, inputs->VrClutchPaddle, &(inputs->rClutchPaddleRaw), VrCLUTCH_MARGIN_MIN, VrCLUTCH_MARGIN_MAX);

	// Conversion from Float to int8_t
	inputs->rClutchPaddle = (int8_t)round(inputs->rClutchPaddleRaw);

	//Clamping
	inputs->rClutchPaddle = CLAMP(inputs->rClutchPaddle, CLUTCH_PADDLE_MIN, CLUTCH_PADDLE_MAX);

	// ---------------------------------------------------------------------------------------------------
	// PCB Supply Voltage Conditioning

	inputs->VSupply = inputs->VSIUAnalog04 * VSUPPLY_DIVIDER_GAIN;


	// ---------------------------------------------------------------------------------------------------
	// Shifting Inputs

	//Up Button
	if(HAL_GPIO_ReadPin(DIN01_GPIO_Port, DIN01_Pin) == GPIO_PIN_RESET && tUpButtonTimer < tInputsTimer && !inputs->BUpShiftButtonDebounce) {
		inputs->BUpShiftButtonDebounce = 1;
		inputs->BUpShiftButtonPressed = 1;
		tUpButtonTimer = tInputsTimer;
		tUpButtonTimer += UP_BUTTON_DEBOUNCE;
	}
	else if(HAL_GPIO_ReadPin(DIN01_GPIO_Port, DIN01_Pin) == GPIO_PIN_SET && inputs->BUpShiftButtonDebounce) {
		inputs->BUpShiftButtonDebounce = 0;
		inputs->BUpShiftButtonPressed = 0;
	}
	else if(inputs->BUpShiftButtonPressed && (tUpButtonTimer + UP_BUTTON_TIMEOUT) < tInputsTimer) {
		inputs->BUpShiftButtonPressed = 0;
	}

	//Down Button
	if(HAL_GPIO_ReadPin(DIN02_GPIO_Port, DIN02_Pin) == GPIO_PIN_RESET && tDnButtonTimer < tInputsTimer && !inputs->BDnShiftButtonDebounce) {
		inputs->BDnShiftButtonDebounce = 1;
		inputs->BDnShiftButtonPressed = 1;
		tDnButtonTimer = tInputsTimer;
		tDnButtonTimer += DN_BUTTON_DEBOUNCE;
	}
	else if(HAL_GPIO_ReadPin(DIN02_GPIO_Port, DIN02_Pin) == GPIO_PIN_SET && inputs->BDnShiftButtonDebounce) {
		inputs->BDnShiftButtonDebounce = 0;
		inputs->BDnShiftButtonPressed = 0;
	}
	else if(inputs->BDnShiftButtonPressed && (tDnButtonTimer + DN_BUTTON_TIMEOUT) < tInputsTimer) {
		inputs->BDnShiftButtonPressed = 0;
	}

	// ---------------------------------------------------------------------------------------------------
	// Buttons

	// Button A
	if(HAL_GPIO_ReadPin(DIN03_GPIO_Port, DIN03_Pin) == GPIO_PIN_RESET && tButtonATimer < tInputsTimer && !inputs->BButtonADebounce) {
		inputs->BButtonADebounce = 1;
		inputs->BButtonAPressed = 1;
		tButtonATimer = tInputsTimer;
		tButtonATimer += BUTTON_A_DEBOUNCE;
	}
	else if(HAL_GPIO_ReadPin(DIN03_GPIO_Port, DIN03_Pin) == GPIO_PIN_SET && inputs->BButtonADebounce) {
		inputs->BButtonADebounce = 0;
		inputs->BButtonAPressed = 0;
	}

	// Button B
	if(HAL_GPIO_ReadPin(DIN04_GPIO_Port, DIN04_Pin) == GPIO_PIN_RESET && tButtonBTimer < tInputsTimer && !inputs->BButtonBDebounce) {
		inputs->BButtonBDebounce = 1;
		inputs->BButtonBPressed = 1;
		tButtonBTimer = tInputsTimer;
		tButtonBTimer += BUTTON_B_DEBOUNCE;
	}
	else if(HAL_GPIO_ReadPin(DIN04_GPIO_Port, DIN04_Pin) == GPIO_PIN_SET && inputs->BButtonBDebounce) {
		inputs->BButtonBDebounce = 0;
		inputs->BButtonBPressed = 0;
	}

	// Button C
	if(HAL_GPIO_ReadPin(DIN05_GPIO_Port, DIN05_Pin) == GPIO_PIN_RESET && tButtonCTimer < tInputsTimer && !inputs->BButtonCDebounce) {
		inputs->BButtonCDebounce = 1;
		inputs->BButtonCPressed = 1;
		tButtonCTimer = tInputsTimer;
		tButtonCTimer += BUTTON_C_DEBOUNCE;
	}
	else if(HAL_GPIO_ReadPin(DIN05_GPIO_Port, DIN05_Pin) == GPIO_PIN_SET && inputs->BButtonCDebounce) {
		inputs->BButtonCDebounce = 0;
		inputs->BButtonCPressed = 0;
	}

	// Button D
	if(HAL_GPIO_ReadPin(DIN06_GPIO_Port, DIN06_Pin) == GPIO_PIN_RESET && tButtonDTimer < tInputsTimer && !inputs->BButtonDDebounce) {
		inputs->BButtonDDebounce = 1;
		inputs->BButtonDPressed = 1;
		tButtonDTimer = tInputsTimer;
		tButtonDTimer += BUTTON_D_DEBOUNCE;
	}
	else if(HAL_GPIO_ReadPin(DIN06_GPIO_Port, DIN06_Pin) == GPIO_PIN_SET && inputs->BButtonDDebounce) {
		inputs->BButtonDDebounce = 0;
		inputs->BButtonDPressed = 0;
	}

	// Button E
	if(HAL_GPIO_ReadPin(DIN07_GPIO_Port, DIN07_Pin) == GPIO_PIN_RESET && tButtonETimer < tInputsTimer && !inputs->BButtonEDebounce) {
		inputs->BButtonEDebounce = 1;
		inputs->BButtonEPressed = 1;
		tButtonETimer = tInputsTimer;
		tButtonETimer += BUTTON_E_DEBOUNCE;
	}
	else if(HAL_GPIO_ReadPin(DIN07_GPIO_Port, DIN07_Pin) == GPIO_PIN_SET && inputs->BButtonEDebounce) {
		inputs->BButtonEDebounce = 0;
		inputs->BButtonEPressed = 0;
	}

	// Button F
	if(HAL_GPIO_ReadPin(DIN08_GPIO_Port, DIN08_Pin) == GPIO_PIN_RESET && tButtonFTimer < tInputsTimer && !inputs->BButtonFDebounce) {
		inputs->BButtonFDebounce = 1;
		inputs->BButtonFPressed = 1;
		tButtonFTimer = tInputsTimer;
		tButtonFTimer += BUTTON_F_DEBOUNCE;
	}
	else if(HAL_GPIO_ReadPin(DIN08_GPIO_Port, DIN08_Pin) == GPIO_PIN_SET && inputs->BButtonFDebounce) {
		inputs->BButtonFDebounce = 0;
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
		 // TODO: TBC...
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
