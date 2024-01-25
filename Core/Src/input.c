/*
 * input.c
 *
 *  Created on: Jan 19, 2024
 *      Author: nkre
 */

#include "input.h"


void Input(ControlData* controlData) {

	controlData-> action = RxData[0];
	controlData-> gear = RxData[1];


    HAL_ADC_PollForConversion(&hadc1, 100);
    uint16_t clutch_value = HAL_ADC_GetValue(&hadc1);

    // Normal Averaging
    if (adc_counter < adc_counter_const) {
        controlData->clutch_value1 += clutch_value;
        adc_counter++;
    } else {
        controlData->clutch_position = controlData->clutch_value1 /adc_counter;
        controlData->clutch_value1=0;
        clutch_value = 0;
        adc_counter = 0;

    }

    controlData->clutch_detection = (controlData->clutch_position > clutch_detection_threshold) ? 1 : 0;

    if(HAL_GPIO_ReadPin(UP_BUTTON_GPIO_Port, UP_BUTTON_Pin) == GPIO_PIN_RESET && button_previous < current && !controlData->up_button && !controlData->action){
    	controlData->up_button = 1;
    	button_previous=current;
    	button_previous+=button_interval;
    }
    else if(controlData->up_button && HAL_GPIO_ReadPin(UP_BUTTON_GPIO_Port, UP_BUTTON_Pin) == GPIO_PIN_SET && button_previous < current) {
    	controlData->up_button = 0;
    }

    if(HAL_GPIO_ReadPin(DOWN_BUTTON_GPIO_Port, DOWN_BUTTON_Pin) == GPIO_PIN_RESET && button_previous < current && !controlData->down_button && !controlData->action){
        controlData->down_button = 1;
        button_previous=current;
        button_previous+=button_interval;
    }
    else if(controlData->down_button && HAL_GPIO_ReadPin(DOWN_BUTTON_GPIO_Port, DOWN_BUTTON_Pin) == GPIO_PIN_SET && button_previous < current) {
        controlData->down_button = 0;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

	switch(RxHeader.StdId) {

	 case FROM_SHIFTER_ID :



	 break;

	 }
}

