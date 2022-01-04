/*
 * setting_speed_rc.c
 *
 *  Created on: 3 lis 2021
 *      Author: Damian
 */

#include "setting_speed_rc.h"

int16_t Relay = 11;
int16_t Jazda = 0;

int16_t Robot_V = 0;
int16_t Robot_Fi = 0;



int16_t V = 0;
int16_t V_bok = 0;
int16_t V_max = 0;
int16_t Fi_max = 0;
int16_t VL = 0;
int16_t VR = 0;
int16_t VL_out = 0;
int16_t VR_out = 0;
int16_t V_nierownosc = 0;



void horizontal_control(int16_t V_bok_apar, int16_t V_apar, int16_t V_max_apar, int16_t Fi_max_apar, int16_t Relay_SW) {

	V_max = map(V_max_apar, 1000, 2000, 0, 400);
	//                                      / tu jest wartocm maskymalnej rotacji
	Fi_max = map(Fi_max_apar, 1000, 2000, 0, 200);

	if ((Relay_SW > 1900) && (Relay_SW < 2100)) {
		Jazda = 1;
	} else {
		Jazda = 0;
	}

	if (Jazda == 1) {
		Robot_V = -map(V_apar, 1000, 2000, -V_max, V_max);
		Robot_Fi = map(V_bok_apar, 1000, 2000, -Fi_max, Fi_max);
	} else {
		Robot_V = 0;
		Robot_Fi = 0;
	}
	if ((Robot_V < 5) && (Robot_V > -5))
		Robot_V = 0;
	if ((Robot_Fi < 5) && (Robot_Fi > -5))
		Robot_Fi = 0;

	Send(Robot_Fi, Robot_V);
	HAL_Delay(7);//need wait a little before next usage, may reduce later (should be at least 7 ms)
}

float pid_output = 0, suma_e_n = 0, steering_angle = 0, poprzedni_e_n = 0,
		auto_balance = 0;
int start;
int8_t vertical_control(int16_t V_bok_apar, int16_t V_apar, int16_t V_max_apar, int16_t Fi_max_apar, int16_t Relay_SW, int16_t balancing_mode, float angle) {

	V_max = map(V_max_apar, 1000, 2000, 0, 7); //zadawany kat
	//                                      / tu jest wartocm maskymalnej rotacji
	Fi_max = map(Fi_max_apar, 1000, 2000, 0, 200);

	steering_angle = mapfloat((float) V_apar, 1000, 2000, (float) -V_max,
			(float) V_max);
	pid_output = pid_calculations(angle, &suma_e_n, &poprzedni_e_n,
			&auto_balance, steering_angle, V_bok_apar);

	if ((Relay_SW > 1900) && (Relay_SW < 2100) && angle > (theta_ref - 45)
			&& angle < (theta_ref + 45) && (balancing_mode > 1900)
			&& (balancing_mode < 2100)) {
		Jazda = 1;
		start = 1;
	} else {
		Jazda = 0;
		suma_e_n = 0;
		pid_output = 0;
		start = 0;
		poprzedni_e_n = 0;
		auto_balance = 0;
	}

	if (Jazda == 1) {
		Robot_V = -pid_output;
		Robot_Fi = map(V_bok_apar, 1000, 2000, -Fi_max, Fi_max);
	} else {
		Robot_V = 0;
		Robot_Fi = 0;
	}
	if ((Robot_V < 1) && (Robot_V > -1))
		Robot_V = 0;
	if ((Robot_Fi < 5) && (Robot_Fi > -5))
		Robot_Fi = 0;

//	Robot_V = pid_output;
//	Robot_Fi = 0;
	Send(Robot_Fi, Robot_V);
	if (start == 1)
		return 2;
	else
		return 3;
}
