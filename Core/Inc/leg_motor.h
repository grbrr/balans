/*
 * leg_motor.h
 *
 *  Created on: Jan 4, 2022
 *      Author: Damian
 */

#ifndef INC_LEG_MOTOR_H_
#define INC_LEG_MOTOR_H_

#include <stdint.h>
#include "tim.h"

#define motorTIM &htim3

void leg_motor(int16_t PWM_value);
void hide_leg();

#endif /* INC_LEG_MOTOR_H_ */
