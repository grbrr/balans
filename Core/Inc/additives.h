/*
 * additives.h
 *
 *  Created on: 3 lis 2021
 *      Author: Damian
 *      Library contains functions useful for different subprograms.
 */

#ifndef INC_ADDITIVES_H_
#define INC_ADDITIVES_H_

#include <stdint.h>
#include "adc.h"

#define potentiometerADC &hadc1

uint32_t getCurrentMicros(void);
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
		int32_t out_max);
float mapfloat(float x, float in_min, float in_max, float out_min,
		float out_max);
float potentiometer_value(void);

#endif /* INC_ADDITIVES_H_ */
