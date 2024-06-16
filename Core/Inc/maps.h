/*
 * maps.h
 *
 *  Created on: Jan 29, 2024
 *      Author: orestis
 */

#ifndef INC_MAPS_H_
#define INC_MAPS_H_

#include <Utils.h>

#define CLUTCH_PADDLE_MAP_SIZE	2

#define CLUTCH_PADDLE_MAP_MAX		104
#define CLUTCH_PADDLE_MAP_MIN		-4

static const float rClutchMap[2][CLUTCH_PADDLE_MAP_SIZE] = {

		/* In:  VrClutchPaddleRaw */	{		1.08			,  			2.16	 	},
		/* Out: rClutchPaddle 	  */	{ CLUTCH_PADDLE_MAP_MIN ,  CLUTCH_PADDLE_MAP_MAX}
};

#endif /* INC_MAPS_H_ */
