/*
 * setting_speed_rc.h
 *
 *  Created on: 3 lis 2021
 *      Author: Damian
 */

#ifndef INC_SETTING_SPEED_RC_H_
#define INC_SETTING_SPEED_RC_H_

#include <stdint.h>
#include "additives.h"
#include "hoverserial.h"
#include "pid.h"
#include "main.h"

#define RC_CH1_NEUTRAL   1500.0
#define RC_CH1_MAX       2000.0
#define RC_CH1_MIN       1000.0

#define RC_CH2_NEUTRAL   1500.0
#define RC_CH2_MAX       2000.0
#define RC_CH2_MIN       1000.0

void horizontal_control(int16_t V_bok_apar, int16_t V_apar, int16_t V_max_apar, int16_t Fi_max_apar, int16_t Relay_SW);
int8_t vertical_control(int16_t V_bok_apar, int16_t V_apar, int16_t V_max_apar, int16_t Fi_max_apar, int16_t Relay_SW, int16_t balancing_mode, float angle);
#endif /* INC_SETTING_SPEED_RC_H_ */
