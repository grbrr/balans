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

#define RC_CH1_NEUTRAL   1500.0
#define RC_CH1_MAX       2000.0
#define RC_CH1_MIN       1000.0

#define RC_CH2_NEUTRAL   1500.0
#define RC_CH2_MAX       2000.0
#define RC_CH2_MIN       1000.0

void horizontal_control(uint16_t *control_data);
int vertical_control(uint16_t *control_data, float angle);
#endif /* INC_SETTING_SPEED_RC_H_ */
