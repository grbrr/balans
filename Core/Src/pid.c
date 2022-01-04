/*
 * pid.c
 *
 *  Created on: 3 lis 2021
 *      Author: Damian
 */

#include "pid.h"
#include "math.h"

float e_n = 0;
float output = 0;

float pid_calculations(float angle, float *suma_e_n, float *poprzedni_e_n,
		float *auto_balance, float steering_angle, int16_t V_bok_apar) {

	float potentiometer = potentiometer_value();
	//definicja uchybu - kat referencyjny (być może zmieniony aparaturą) odjąć kąt aktualny z uwzględnioną autokalibracją i potencjometrem
	e_n = (theta_ref - steering_angle) - angle - potentiometer - *auto_balance;
	//Obliczenie i ograniczenie sumy wszystkich błędów
	*suma_e_n += e_n;
	if (*suma_e_n > ograniczenie_regulatora)
		*suma_e_n = ograniczenie_regulatora;
	else if (*suma_e_n < -ograniczenie_regulatora)
		*suma_e_n = -ograniczenie_regulatora;
	//PID
	output = k_p * e_n + k_i * *suma_e_n + k_d * (e_n - *poprzedni_e_n);

	//ograniczenie wyjścia PID
	if (output > ograniczenie_regulatora)
		output = ograniczenie_regulatora;
	else if (output < -ograniczenie_regulatora)
		output = -ograniczenie_regulatora;

	//Zapamiętanie ostatniego błędu
	*poprzedni_e_n = e_n;

	//Robot poszukuje punktu równowagi jeżeli się przemieszcza
	if (steering_angle == 0) {                 //jeżeli nie podano ką
		if (output < 0)
			*auto_balance += 0.003; //zwiększ autokalibrację jeśli porusza się do przodu
		else if (output > 0)
			*auto_balance -= 0.003; //zmniejsz autkalibrację jeśli porusza się do tyłu
	}
	// przesunięcie wyjścia PID o 40 jeżeli przemieszcza się tylko do przodu (przy skręcaniu powoduje to duże drgania, więc wtedy nieaktywne)
	// silniki BLDC zaczynają reagować dopiero powyżej wartości 40 podanej do sterownika
	if (V_bok_apar < 1520 && V_bok_apar > 1480) {
		if (output > 0)
			output = output + 40;
		else if (output < 0)
			output = output - 40;
	}
	return output;
}
