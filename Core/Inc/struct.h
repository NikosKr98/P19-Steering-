/*
 * struct.h
 *
 *  Created on: Jan 19, 2024
 *      Author: nkre
 */

#ifndef INC_STRUCT_H_
#define INC_STRUCT_H_


#include <stdint.h>

typedef struct {
    uint16_t clutch_position;
    uint8_t clutch_detection;
    uint32_t clutch_value1;
    uint8_t up_button;
    uint8_t down_button;
    uint8_t action;
    uint8_t gear;
} ControlData;


#endif /* INC_STRUCT_H_ */
