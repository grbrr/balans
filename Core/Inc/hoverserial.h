/*
 * hoverserial.h
 *
 *  Biblioteka jest konwersją biblioteki EFERU na potrzeby STM32
 */

#ifndef INC_HOVERSERIAL_H_
#define INC_HOVERSERIAL_H_

#include "usart.h"
#include <stdio.h>
#include <string.h>

#define commUART      		&huart2		// Tutaj wybierz interfejs UART do komunikacji między jednostką główną a sterownikiem BLDC
#define debugUART			&huart1		// Opcjonalny UART na potrzeby debugowania (odkomentuj DEBUG_RX)

#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
// #define DEBUG_RX                     // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

_Bool data_available;					// Jest używane zarówno w main.c jak i hoverserial.c

void Send(int16_t uSteer, int16_t uSpeed);
void Receive(uint8_t *byte);

#endif /* INC_HOVERSERIAL_H_ */
