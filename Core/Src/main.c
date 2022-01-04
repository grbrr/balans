/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "hoverserial.h"
#include "mpu6050.h"
#include "ibus.h"
#include "additives.h"
#include "setting_speed_rc.h"
#include "pid.h"
#include "servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define motorTIM &htim3
int16_t silnik;
int16_t silnikbest;
float theta;
uint8_t balance_state_machine = 0;
uint16_t balancing_switch;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	// char buffer[100]; //bufor na potrzeby debugowania w uart1
	// przyklad do wysylania danych po serialu
	// sprintf(buffer, "cos tam: %3.2f\n\r", jakis_float);
	// HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
	uint8_t byte;

	_Bool mpu6050_ready = 0;

	uint16_t ibus_data[IBUS_USER_CHANNELS];
	uint32_t loop_timer = 0;
	float time = 0;

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_DMA_Init();
	MX_USART6_UART_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_TIM11_Init();
	/* USER CODE BEGIN 2 */
	HAL_Delay(2000);
	mpu6050_ready = MPU6050_Init();
	HAL_Delay(10);

	ibus_init();
	ibus_read(ibus_data);

	HAL_UART_Receive_IT(commUART, &byte, sizeof(byte)); //oczekiwanie na przerwanie

	HAL_TIM_PWM_Start(motorTIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(motorTIM, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
//	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 1100);
	uint16_t switch_vibrations_counter = 0;
	uint32_t fall_counter = 0;
	loop_timer = getCurrentMicros();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		if (mpu6050_ready) { //jeśli mpu6050 nie jest gotowe to cały program nie wystartuje i należy zrestartować kontroler/rozwiązać problemy z magistralą I2C

			// pochodzi z hoverserial.ino
			Receive(&byte);

			ibus_read(ibus_data);
			ibus_soft_failsafe(ibus_data, 10); // if ibus is not updated, clear ibus data - pochodzi z biblioteki ibus

			balancing_switch = ibus_data[8 - 1];
			uint16_t Relay_SW = ibus_data[6 - 1];	//zalacz silniki

			silnik = ibus_data[4 - 1];

			float acctheta = MPU6050_Read_Accel();
			float gyrovelo = MPU6050_Read_Gyro(time);
			theta = 0.9995 * (theta + gyrovelo * time)
					+ (1 - 0.9995) * acctheta;

			camera_angle(theta);


			//if (balancing_switch > 1900 && balancing_switch < 2100) {
			//	for (int i = 0; i <= 1000; i++) {
			//		__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_1, 0);
			//		__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_2, i);
			//	}
			//	while (theta < 48) {
			//	}
			//	__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_1, 0);
			//	__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_2, 0);
			//}
			//bezpieczenstwo

			if (balance_state_machine == 0) {			// 4wheel mode
				horizontal_control(ibus_data);
				if (HAL_GPIO_ReadPin(LEG_GPIO_Port, LEG_Pin)) {
					switch_vibrations_counter = 0;

					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
					__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_1, 50);
					__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_2, 0);

				} else {
					switch_vibrations_counter++;
					if (switch_vibrations_counter > 50) {
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
						__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_1, 0);
						__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_2, 0);
					}
				}
				//if (theta > 1) {
				//	__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_1, 50);
				//} else
				//	__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_1, 0);
			}
			if (Relay_SW > 1900 && Relay_SW < 2100) {
				if (balance_state_machine == 0 && balancing_switch > 1900
						&& balancing_switch < 2100) {
					balance_state_machine = 1;
				}
				if (balance_state_machine == 1) {			// getting up
					Send(0, 0);
					__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_1, 0);
					__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_2, 100);
				}
				if (balance_state_machine == 1 && theta > 70) {
					balance_state_machine = 2;
					__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_1, 0);
					__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_2, 0);
				}
				if (balance_state_machine == 2) {			// balancing
					balance_state_machine = vertical_control(ibus_data, theta);
					fall_counter = 0;
					if (HAL_GPIO_ReadPin(LEG_GPIO_Port, LEG_Pin)) {
						switch_vibrations_counter = 0;

						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
						__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_1, 50);
						__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_2, 0);

					} else {
						switch_vibrations_counter++;
						if (switch_vibrations_counter > 50) {
							HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
							__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_1, 0);
							__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_2, 0);
						}
					}

				}
				if (balance_state_machine == 3) {			// going down
					__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_1, 0);
					__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_2, 75);
					Send(0, 0);
					fall_counter++;
					if (fall_counter > 700) {
						balance_state_machine = 0;
						__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_1, 0);
						__HAL_TIM_SET_COMPARE(motorTIM, TIM_CHANNEL_2, 0);
					}
				}

			}

			//	sprintf(buffer, "%d %d\n\r", Robot_Fi, Robot_V);
			//	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);

			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */

			time = (getCurrentMicros() - loop_timer) * 1e-6;
			loop_timer = getCurrentMicros();
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {//przerwanie na potrzeby obsługi komunikacji ze sterownikiem BLDC
	if (huart == commUART) { 	//sprawdzenie czy przyszło z właściwego uarta
		data_available = 1;
		//ponowne wywołanie oczekiwania na przerwania dzieje się po przetworzeniu danych w hoverserial.c (linijka 74 w hoverserial.c)
	}
	if (huart == IBUS_UART)
		ibus_reset_failsafe();
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
