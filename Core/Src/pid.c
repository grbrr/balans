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
	//definicja uchybu - aktualny kat odjac kat zadany
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

	//przełącznik histerezowy (zapobiega ciągłym próbom regulacji w pobliżu theta_ref)
//	if (output < theta_ref + 0.1 && output > theta_ref - 0.1)
//		output = 0;
	//The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
	if (steering_angle == 0) {                 //If the setpoint is zero degrees
		if (output < 0)
			*auto_balance += 0.003; //Increase the self_balance_pid_setpoint if the robot is still moving forewards
		else if (output > 0)
			*auto_balance -= 0.003; //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
	}
	if (V_bok_apar < 1520 && V_bok_apar > 1480) {
		if (output > 0)
			output = output + 40;
		else if (output < 0)
			output = output - 40;
	}
	return output;
}
