/*
 * servo.h
 *
 *  Created on: Dec 12, 2021
 *      Author: Damian
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "tim.h"
#include "additives.h"

#define servoTIM &htim11

void camera_angle(float angle);

#endif /* INC_SERVO_H_ */
