/*
 * servo.c
 *
 *  Created on: Dec 12, 2021
 *      Author: Damian
 */

#include "servo.h"

void camera_angle(float angle) {
	int32_t value;
	if (angle < 1)
		angle = 0;
	if (angle >= 90)
		angle = 90;
	value = map(angle, 0, 90, 950, 1750);
	__HAL_TIM_SET_COMPARE(servoTIM, TIM_CHANNEL_1, value);
}
