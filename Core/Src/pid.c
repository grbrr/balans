/*
 * pid.c
 *
 *  Created on: 3 lis 2021
 *      Author: Damian
 */

#include "pid.h"

float e_n, poprzedni_e_n = 0;

float pid_calculations(float angle, float *suma_e_n) {
	//definicja uchybu - aktualny kat odjac kat zadany
	//theta_ref += x;
	float potentiometer = potentiometer_value();

	e_n = theta_ref - angle - potentiometer;

	//Obliczenie i ograniczenie sumy wszystkich błędów
	*suma_e_n += e_n;
	if (*suma_e_n > ograniczenie_regulatora)
		*suma_e_n = ograniczenie_regulatora;
	else if (*suma_e_n < -ograniczenie_regulatora)
		*suma_e_n = -ograniczenie_regulatora;
	//PID
	float output = k_p * e_n + k_i * *suma_e_n + k_d * (e_n - poprzedni_e_n);

	//ograniczenie wyjścia PID
	if (output > ograniczenie_regulatora)
		output = ograniczenie_regulatora;
	else if (output < -ograniczenie_regulatora)
		output = -ograniczenie_regulatora;

	//Zapamiętanie ostatniego błędu
	poprzedni_e_n = e_n;

	//przełącznik histerezowy (zapobiega ciągłym próbom regulacji w pobliżu theta_ref)
//			if (pid_output < theta_ref+0.5 && pid_output > theta_ref-0.5)
//				pid_output = 0;
	return output;
}
