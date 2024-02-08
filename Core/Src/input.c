/*
 * input.c
 *
 *  Created on: Jan 19, 2024
 *      Author: nkre
 */

#include <Input.h>
#include <Maps.h>


//timing Variables!
uint32_t tInputsTimer=0, tUpButtonTimer=0, tDnButtonTimer=0, tLaunchButtonTimer=0, tEmergercyButtonTimer=0, tAuxLeftButtonTime=0, tAuxRightButtonTime=0 ;

volatile uint8_t NCANErrorCount;
volatile uint16_t NCanGetRxErrorCount=0;


uint16_t rClutchRawADCFiltered;
volatile uint8_t NAdcBufferSide;

//Private Functions Declaration
uint16_t MyHalfBufferAverage(uint16_t *buffer, uint16_t halfsize, uint8_t side);


void ReadInputs(InputStruct *inputs){

	tInputsTimer = HAL_GetTick();

	// ---------------------------------------------------------------------------------------------------
	//Clutch Paddle

	//ADC Averaging
	rClutchRawADCFiltered = MyHalfBufferAverage(adcRawValue, ADC_BUFFER_HALF_SIZE, NAdcBufferSide);

	//Voltage Conversion
	inputs-> VrClutchPaddleRaw = (float)(rClutchRawADCFiltered * 3.3 / 4095.0);

	//Mapping
	inputs-> BrClutchPaddleInError= My2DMapInterpolate(CLUTCH_PADDLE_MAP_SIZE, rClutchMap, inputs->VrClutchPaddleRaw, &(inputs->rClucthPaddleRaw), VrCLUTCH_MARGIN_MIN, VrCLUTCH_MARGIN_MAX);

	// Conversion from Float to int8_t
	inputs->rClutchPaddle = (int8_t)round(inputs->rClucthPaddleRaw);

	//Clamping
	inputs->rClutchPaddle = CLAMP(inputs->rClutchPaddle, CLUTCH_PADDLE_MIN, CLUTCH_PADDLE_MAX);



	// ---------------------------------------------------------------------------------------------------
	//Buttons

	//Up Button
	if(HAL_GPIO_ReadPin(UP_BUTTON_GPIO_Port, UP_BUTTON_Pin) == GPIO_PIN_RESET && tUpButtonTimer < tInputsTimer && !inputs->BUpShiftButtonDebounce){
		inputs->BUpShiftButtonDebounce = 1;
		inputs->BUpShiftButtonPressed = 1;
		tUpButtonTimer = tInputsTimer;
		tUpButtonTimer += UP_BUTTON_INTERVAL;
	}
	else if(HAL_GPIO_ReadPin(UP_BUTTON_GPIO_Port, UP_BUTTON_Pin) == GPIO_PIN_SET && inputs->BUpShiftButtonDebounce){
		inputs->BUpShiftButtonDebounce = 0;
		inputs->BUpShiftButtonPressed = 0;
	}

	//Down Button
	if(HAL_GPIO_ReadPin(DOWN_BUTTON_GPIO_Port, DOWN_BUTTON_Pin) == GPIO_PIN_RESET && tDnButtonTimer < tInputsTimer && !inputs->BDnShiftButtonDebounce){
		inputs->BDnShiftButtonDebounce = 1;
		inputs->BDnShiftButtonPressed = 1;
		tDnButtonTimer = tInputsTimer;
		tDnButtonTimer += DN_BUTTON_INTERVAL;
	}
	else if(HAL_GPIO_ReadPin(DOWN_BUTTON_GPIO_Port, DOWN_BUTTON_Pin) == GPIO_PIN_SET && inputs->BDnShiftButtonDebounce){
		inputs->BDnShiftButtonDebounce = 0;
		inputs->BDnShiftButtonPressed = 0;
	}

	//TODO: Fix the Buttons(New PCB rev)
	//Launch Button
	if(HAL_GPIO_ReadPin(DOWN_BUTTON_GPIO_Port, DOWN_BUTTON_Pin) == GPIO_PIN_RESET && tLaunchButtonTimer < tInputsTimer && !inputs->BLaunchButtonDebounce){
		inputs->BLaunchButtonDebounce = 1;
		inputs->BLaunchButtonPressed = 1;
		tLaunchButtonTimer = tInputsTimer;
		tLaunchButtonTimer += LAUNCH_BUTTON_INTERVAL;
	}
	else if(HAL_GPIO_ReadPin(DOWN_BUTTON_GPIO_Port, DOWN_BUTTON_Pin) == GPIO_PIN_SET && inputs->BLaunchButtonDebounce){
		inputs->BLaunchButtonDebounce = 0;
		inputs->BLaunchButtonPressed = 0;
	}

	//Emergency Button
	if(HAL_GPIO_ReadPin(DOWN_BUTTON_GPIO_Port, DOWN_BUTTON_Pin) == GPIO_PIN_RESET && tEmergercyButtonTimer < tInputsTimer && !inputs->BEmergencyButtonDebounce){
		inputs->BEmergencyButtonDebounce = 1;
		inputs->BEmergencyButtonPressed = 1;
		tEmergercyButtonTimer = tInputsTimer;
		tEmergercyButtonTimer += EMERGENCY_BUTTON_INTERVAL;
	}
	else if(HAL_GPIO_ReadPin(DOWN_BUTTON_GPIO_Port, DOWN_BUTTON_Pin) == GPIO_PIN_SET && inputs->BEmergencyButtonDebounce){
		inputs->BEmergencyButtonDebounce = 0;
		inputs->BEmergencyButtonPressed = 0;
	}

	//Aux Left Button
	if(HAL_GPIO_ReadPin(DOWN_BUTTON_GPIO_Port, DOWN_BUTTON_Pin) == GPIO_PIN_RESET && tAuxLeftButtonTime < tInputsTimer && !inputs->BAuxLeftButtonDebounce){
		inputs->BAuxLeftButtonDebounce = 1;
		inputs->BAuxLeftButtonPressed = 1;
		tAuxLeftButtonTime = tInputsTimer;
		tAuxLeftButtonTime += AUXL_BUTTON_INTERVAL;
	}
	else if(HAL_GPIO_ReadPin(DOWN_BUTTON_GPIO_Port, DOWN_BUTTON_Pin) == GPIO_PIN_SET && inputs->BAuxLeftButtonDebounce){
		inputs->BAuxLeftButtonDebounce = 0;
		inputs->BAuxLeftButtonPressed = 0;
	}

	//Aux Right Button
	if(HAL_GPIO_ReadPin(DOWN_BUTTON_GPIO_Port, DOWN_BUTTON_Pin) == GPIO_PIN_RESET && tAuxRightButtonTime < tInputsTimer && !inputs->BAuxRightButtonDebounce){
		inputs->BAuxRightButtonDebounce = 1;
		inputs->BAuxRightButtonPressed = 1;
		tAuxRightButtonTime = tInputsTimer;
		tAuxRightButtonTime += AUXR_BUTTON_INTERVAL;
	}
	else if(HAL_GPIO_ReadPin(DOWN_BUTTON_GPIO_Port, DOWN_BUTTON_Pin) == GPIO_PIN_SET && inputs->BAuxRightButtonDebounce){
		inputs->BAuxRightButtonDebounce = 0;
		inputs->BAuxRightButtonPressed = 0;
	}

	// ---------------------------------------------------------------------------------------------------
}


void InitInputs(void){
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcRawValue, ADC_BUFFER_SIZE);
}


uint16_t MyHalfBufferAverage(uint16_t *buffer, uint16_t halfsize, uint8_t side) {

	uint32_t Accumulator=0;
	uint16_t Offset = (side == 1 ? halfsize : 0);

	for(uint16_t i=0; i<halfsize; i++) {
		Accumulator += buffer[i + Offset];
	}

	Accumulator /= halfsize;
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
		 // TODO: BrClutchPaddleRawInErrorCAN = ... 	// TODO: TBC...
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
