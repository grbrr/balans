/*
 * setting_speed_rc.c
 *
 *  Created on: 3 lis 2021
 *      Author: Damian
 */

#include "setting_speed_rc.h"

_Bool start;

int16_t Jazda = 0;
int16_t Robot_V = 0;
int16_t Robot_Fi = 0;
int16_t V_max = 0;
int16_t Fi_max = 0;

float pid_output = 0;
float suma_e_n = 0;
float poprzedni_e_n = 0;
float auto_balance = 0;
float steering_angle = 0;

void horizontal_control(int16_t V_bok_apar, int16_t V_apar, int16_t V_max_apar,
		int16_t Fi_max_apar, int16_t Relay_SW) {

	V_max = map(V_max_apar, 1000, 2000, 0, 400);
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
	HAL_Delay(7);
}

int8_t vertical_control(int16_t V_bok_apar, int16_t V_apar, int16_t V_max_apar,
		int16_t Fi_max_apar, int16_t Relay_SW, int16_t balancing_mode,
		float angle) {

	V_max = map(V_max_apar, 1000, 2000, 0, 7);
	Fi_max = map(Fi_max_apar, 1000, 2000, 0, 200);

	steering_angle = mapfloat((float) V_apar, 1000, 2000, (float) -V_max,
			(float) V_max);
	//przekazywane są wskaźniki do zmiennych, które muszą być wyzerowane jeżeli zakończy się tryb balansowania
	pid_output = pid_calculations(angle, &suma_e_n, &poprzedni_e_n,
			&auto_balance, steering_angle, V_bok_apar);

	// balansuj jeśli przełączniiki silników i balansowania ustawione, kąt nie odbiega o +-35 stopni
	if ((Relay_SW > 1900) && (Relay_SW < 2100) && angle > (theta_ref - 35)
			&& angle < (theta_ref + 35) && (balancing_mode > 1900)
			&& (balancing_mode < 2100)) {
		Jazda = 1;
		start = 1;
	} else {//w przeciwnym przypadku wyzeruj wszystko i wyłącz tryb balansowania
		Jazda = 0;
		start = 0;
		suma_e_n = 0;
		pid_output = 0;
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

	Send(Robot_Fi, Robot_V);
	if (start == 1)
		return 2;	// kontynuuj balansowanie
	else
		return 3;	// przejdź do opadania
}
