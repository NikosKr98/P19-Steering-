/*
 * input.h
 *
 *  Created on: Jan 19, 2024
 *      Author: nkre
 */

#ifndef INC_INPUT_H_
#define INC_INPUT_H_

#include "stm32f1xx_hal.h"
#include "gpio_init.h"
#include "struct.h"


#define adc_counter_const 500
#define clutch_detection_threshold 1000
#define FROM_SHIFTER_ID 0x320



extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan;

extern CAN_FilterTypeDef FilterConfig0;
extern CAN_RxHeaderTypeDef RxHeader;


extern uint8_t RxData[8];
extern uint32_t current, button_previous, button_interval;
extern volatile uint16_t adc_counter;


/* Function prototype */
void Input(ControlData* controlData);

#endif /* INC_INPUT_H_ */


