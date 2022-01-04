/*
 * leg_motor.c
 *
 *  Created on: Jan 4, 2022
 *      Author: Damian
 */

#include "leg_motor.h"

uint16_t switch_vibrations_counter = 0;

void leg_motor(int16_t PWM_value) {
	// nastawienie obrotów silnika na daną wartość
	if (PWM_value >= 0) {
		__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_1, PWM_value);
		__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_2, 0);
	} else {
		__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_2, -PWM_value);
	}
}

void hide_leg() {
	//jeżeli noga nie jest schowana to silnik nogi działa dopóki kontaktron nie wykryje obecności nogi
	if (HAL_GPIO_ReadPin(LEG_GPIO_Port, LEG_Pin)) {
		switch_vibrations_counter = 0;
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1); //dioda do debugowania
		leg_motor(50);
	} else {
		switch_vibrations_counter++;
		if (switch_vibrations_counter > 20) {
			//jeżeli przez 20 iteracji programu noga jest wykrywana to wyłącz silnik (kwesta drgań styków kontaktronu)
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
			leg_motor(0);
		}
	}
}
