/*
 * hoverserial.c
 *
 *  Created on: Oct 17, 2021
 *      Author: Damian
 */

#include "hoverserial.h"
#include "stdio.h"
#include "string.h"


// ########################## DEFINES ##########################



// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
uint8_t *p;                                // Pointer declaration for the new received data
uint8_t incomingByte = 0x00;
uint8_t incomingBytePrev = 0x00;




typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;





// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  //HoverSerial.write((uint8_t *) &Command, sizeof(Command));
  HAL_UART_Transmit(&huart2, (uint8_t*) &Command, sizeof(Command), 100);
}

// ########################## RECEIVE ##########################
void Receive(uint8_t* byte)
{
	char buffer[30];

	if(data_available)
	{
		incomingByte = *byte;             // Read the incoming byte
		bufStartFrame = ((uint16_t) (incomingByte) << 8) | incomingBytePrev; // Construct the start frame
		incomingBytePrev = incomingByte;
		data_available = 0;
		HAL_UART_Receive_IT(&huart2, byte, sizeof(*byte)); //ponowne oczekiwanie na przerwanie
	}

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
	HAL_UART_Transmit(&huart1, (uint8_t*) &bufStartFrame, sizeof(bufStartFrame),100);
        return;
    #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
        p       = (uint8_t *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte;
        idx++;
    }

    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            // Print data to built-in Serial
            sprintf(buffer, "1: ");
            HAL_UART_Transmit(&huart2, (uint8_t*) &buffer, strlen(buffer), 100);
            HAL_UART_Transmit(&huart2, (uint8_t*) &Feedback.cmd1, sizeof(Feedback.cmd1), 100);
            sprintf(buffer, "2: ");
            HAL_UART_Transmit(&huart2, (uint8_t*) &buffer, strlen(buffer), 100);
            HAL_UART_Transmit(&huart2, (uint8_t*) &Feedback.cmd2, sizeof(Feedback.cmd2), 100);
            sprintf(buffer, "3: ");
            HAL_UART_Transmit(&huart2, (uint8_t*) &buffer, strlen(buffer), 100);
            HAL_UART_Transmit(&huart2, (uint8_t*) &Feedback.speedR_meas, sizeof(Feedback.speedR_meas), 100);
            sprintf(buffer, "4: ");
            HAL_UART_Transmit(&huart2, (uint8_t*) &buffer, strlen(buffer), 100);
            HAL_UART_Transmit(&huart2, (uint8_t*) &Feedback.speedL_meas, sizeof(Feedback.speedL_meas), 100);
            sprintf(buffer, "5: ");
            HAL_UART_Transmit(&huart2, (uint8_t*) &buffer, strlen(buffer), 100);
            HAL_UART_Transmit(&huart2, (uint8_t*) &Feedback.batVoltage, sizeof(Feedback.batVoltage), 100);
            sprintf(buffer, "6: ");
            HAL_UART_Transmit(&huart2, (uint8_t*) &buffer, strlen(buffer), 100);
            HAL_UART_Transmit(&huart2, (uint8_t*) &Feedback.boardTemp, sizeof(Feedback.boardTemp), 100);
            sprintf(buffer, "7: ");
            HAL_UART_Transmit(&huart2, (uint8_t*) &buffer, strlen(buffer), 100);
            HAL_UART_Transmit(&huart2, (uint8_t*) &Feedback.cmdLed, sizeof(Feedback.cmdLed), 100);
            sprintf(buffer, "\n\r");
            HAL_UART_Transmit(&huart2, (uint8_t*) &buffer, strlen(buffer), 100);

            /*Serial.print("1: ");   Serial.print(Feedback.cmd1);
            Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
            Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
            Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
            Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
            Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
            Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);*/
        } else {
        	sprintf(buffer, "Non-valid data skipped");
        	HAL_UART_Transmit(&huart2, (uint8_t*) &buffer, strlen(buffer), 100);
        	sprintf(buffer, "\n\r");
        	HAL_UART_Transmit(&huart2, (uint8_t*) &buffer, strlen(buffer), 100);
          //Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}
