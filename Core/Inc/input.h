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

#define VSUPPLY_DIVIDER_GAIN				0.23077f

// CLUTCH
#define VrCLUTCH_MARGIN_MIN 				0.1f	// the voltage below the min map voltage we accept to arrive before declaring out of bounds
#define VrCLUTCH_MARGIN_MAX 				0.1f	// the voltage above the max map voltage we accept to arrive before declaring out of bounds

#define CLUTCH_PADDLE_MAX					104
#define CLUTCH_PADDLE_MIN					-4

// TIMING (ms)
#define UP_BUTTON_DEBOUNCE					50
#define DN_BUTTON_DEBOUNCE					50
#define UP_BUTTON_TIMEOUT					1000 - UP_BUTTON_DEBOUNCE
#define DN_BUTTON_TIMEOUT					1000 - DN_BUTTON_DEBOUNCE

#define BUTTON_A_DEBOUNCE					100
#define BUTTON_B_DEBOUNCE					100
#define BUTTON_C_DEBOUNCE					100
#define BUTTON_D_DEBOUNCE					100
#define BUTTON_E_DEBOUNCE					100
#define BUTTON_F_DEBOUNCE					100


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

	// Digital Inputs
	uint8_t BSIUDIN01;
	uint8_t BSIUDIN02;
	uint8_t BSIUDIN03;
	uint8_t BSIUDIN04;
	uint8_t BSIUDIN05;
	uint8_t BSIUDIN06;
	uint8_t BSIUDIN07;
	uint8_t BSIUDIN08;

	// Digital Outputs
	uint8_t BSIUDO01Demand;
	uint8_t BSIUDO02Demand;
	uint8_t BSIUDO03Demand;
	uint8_t BSIUDO04Demand;

	//CLUTCH
	uint8_t BrClutchPaddleInError;
	float VrClutchPaddle;
	float rClutchPaddleRaw;
	int8_t rClutchPaddle;

	// Buttons
	uint8_t BUpShiftButtonPressed;
	uint8_t BDnShiftButtonPressed;

	uint8_t BButtonAPressed;
	uint8_t BButtonBPressed;
	uint8_t BButtonCPressed;
	uint8_t BButtonDPressed;
	uint8_t BButtonEPressed;
	uint8_t BButtonFPressed;

	// Debounce
	uint8_t BUpShiftButtonDebounce;
	uint8_t BDnShiftButtonDebounce;

	uint8_t BButtonADebounce;
	uint8_t BButtonBDebounce;
	uint8_t BButtonCDebounce;
	uint8_t BButtonDDebounce;
	uint8_t BButtonEDebounce;
	uint8_t BButtonFDebounce;

	// Error Flags
	uint8_t BUpShiftButtonInError;
	uint8_t BDnShiftButtonInError;


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


