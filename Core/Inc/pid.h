/*
 * pid.h
 *
 *  Created on: 3 lis 2021
 *      Author: Damian
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "additives.h"

#define ograniczenie_regulatora 400
#define theta_ref 81
#define k_p 25
#define k_i 0.3
#define k_d 1200

float pid_calculations(float angle, float *suma_e_n, float steering_angle);

#endif /* INC_PID_H_ */
