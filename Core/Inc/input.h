/*
 * input.h
 *
 *  Created on: Jan 19, 2024
 *      Author: nkre
 */

#ifndef INC_INPUT_H_
#define INC_INPUT_H_

#include <Utils.h>

// ANALOGS
#define ADC_BUFFER_SIZE						1072*2
#define ADC_BUFFER_HALF_SIZE				ADC_BUFFER_SIZE/2
#define ADC_NUMBER_OF_CHANNELS				4

#define VSUPPLY_DIVIDER_GAIN					0.23077f

// CLUTCH
#define VrCLUTCH_MARGIN_MIN 				0.1f	// the voltage below the min map voltage we accept to arrive before declaring out of bounds
#define VrCLUTCH_MARGIN_MAX 				0.1f	// the voltage above the max map voltage we accept to arrive before declaring out of bounds

#define CLUTCH_PADDLE_MAX					104
#define CLUTCH_PADDLE_MIN					-4

// TIMING
#define UP_BUTTON_INTERVAL					50
#define DN_BUTTON_INTERVAL					50
#define LAUNCH_BUTTON_INTERVAL				100
#define EMERGENCY_BUTTON_INTERVAL			100
#define AUXL_BUTTON_INTERVAL				100
#define AUXR_BUTTON_INTERVAL				100

#define UP_BUTTON_TIMEOUT					1000 - UP_BUTTON_INTERVAL
#define DN_BUTTON_TIMEOUT					1000 - DN_BUTTON_INTERVAL

// CAN
#define SIU_TX_ID							0x310
#define SIU_RX_ID							0x320


extern uint16_t adcRawValue[ADC_BUFFER_SIZE];

extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan;

typedef struct _InputStruct{

	//Analog Inputs
	float VSIUAnalog01;
	float VSIUAnalog02;
	float VSIUAnalog03;
	float VSIUAnalog04;

	uint16_t NADCChannel01Raw;
	uint16_t NADCChannel02Raw;
	uint16_t NADCChannel03Raw;
	uint16_t NADCChannel04Raw;

	//CLUTCH
	uint8_t BrClutchPaddleInError;
	float VrClutchPaddle;
	float rClutchPaddleRaw;
	int8_t rClutchPaddle;

	// Buttons
	uint8_t BUpShiftButtonPressed;
	uint8_t BDnShiftButtonPressed;
	uint8_t BLaunchButtonPressed;
	uint8_t BEmergencyButtonPressed;
	uint8_t BAuxLeftButtonPressed;
	uint8_t BAuxRightButtonPressed;

	// Debounce
	uint8_t BUpShiftButtonDebounce;
	uint8_t BDnShiftButtonDebounce;
	uint8_t BLaunchButtonDebounce;
	uint8_t BEmergencyButtonDebounce;
	uint8_t BAuxLeftButtonDebounce;
	uint8_t BAuxRightButtonDebounce;

	// Error Flags
	uint8_t BUpShiftButtonInError;
	uint8_t BDnShiftButtonInError;
	uint8_t BLaunchButtonInError;
	uint8_t BEmergencyButtonInError;
	uint8_t BAuxLeftButtonInError;
	uint8_t BAuxRightButtonInError;

	// CAN
	uint8_t NCANErrors;				// CAN Bus error count
	uint8_t NCANRxErrors;			// CAN message receive error count

	//PCB Power Supply
	float VSupply;					// PCB Voltage Input Diagnostic


}InputStruct;


/* Function prototype */

void InitInputs(void);
void ReadInputs(InputStruct *input);

#endif /* INC_INPUT_H_ */


