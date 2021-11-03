/*
 * hoverserial.h
 *
 *  Created on: Oct 17, 2021
 *      Author: Damian
 */

#ifndef INC_HOVERSERIAL_H_
#define INC_HOVERSERIAL_H_

#include "usart.h"
#include <stdio.h>
#include <string.h>

#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)




_Bool data_available;

void Send(int16_t uSteer, int16_t uSpeed);
void Receive(uint8_t* byte);

#endif /* INC_HOVERSERIAL_H_ */
