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
//#include <stdio.h> not needed if no debugging
//#include <string.h>
#include "hoverserial.h"
#include "mpu6050.h"
#include "ibus.h"
#include "additives.h"
#include "setting_speed_rc.h"
#include "pid.h"
#include "servo.h"
#include "leg_motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
	_Bool mpu6050_ready = 0; //zwracane przez MPU6050_Init()

	uint8_t balance_state_machine = 0; //maszyna stanów (stany 0 - poziomo, 1 - wstawanie, 2 - pionowo, 3 - opadanie)

	uint8_t byte;	//wymagane dla ibus
	uint16_t ibus_data[IBUS_USER_CHANNELS];

	uint32_t fall_counter = 0; // zmienna do eksperymentalnego wyznaczania czasu opadania robota

	uint32_t loop_timer = 0; //zmienne do obliczania czasu trwania pętli
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

	HAL_UART_Receive_IT(commUART, &byte, sizeof(byte)); //oczekiwanie na przerwanie

	HAL_TIM_PWM_Start(motorTIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(motorTIM, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(servoTIM, TIM_CHANNEL_1);

	loop_timer = getCurrentMicros();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (mpu6050_ready) {
			//jeśli mpu6050 nie jest gotowe to cały program nie wystartuje i należy zrestartować kontroler/rozwiązać problemy z magistralą I2C

			// pochodzi z hoverserial.ino
			Receive(&byte);

			ibus_read(ibus_data);
			ibus_soft_failsafe(ibus_data, 10); // if ibus is not updated, clear ibus data - pochodzi z biblioteki ibus

			int16_t V_bok_apar = ibus_data[1 - 1]; 		// predkosc boki
			int16_t V_apar = ibus_data[2 - 1]; 			// predkosc przód
			int16_t V_max_apar = ibus_data[5 - 1]; // regulacja maksymalnej predkosci silnikow w przód
			int16_t Relay_SW = ibus_data[6 - 1]; 		// zalacz silniki
			int16_t Fi_max_apar = ibus_data[7 - 1];	// regulacja maksymalnej predkosci katowej przy skrecaniu
			int16_t balancing_mode = ibus_data[8 - 1];	// przelacz tryb jazdy

			float acctheta = MPU6050_Read_Accel();
			float gyrovelo = MPU6050_Read_Gyro();

			float theta = 0.9995 * (theta + gyrovelo * time)//filtr komplementarny
			+ (1 - 0.9995) * acctheta;

			camera_angle(theta);	//ustawienie serwa kamery

			// Poniżej maszyna stanów do jazdy i zmiany trybów
			if (balance_state_machine == 0 || Relay_SW < 1900
					|| Relay_SW > 2100) {
				//pierwotny stan - 4wheel, jazda horyzontalna, także stan po wyłączeniu silników
				horizontal_control(V_bok_apar, V_apar, V_max_apar, Fi_max_apar,
						Relay_SW);
				hide_leg();
			}
			if (Relay_SW > 1900 && Relay_SW < 2100) {
				//kolejne tryby pod warunkiem załączenia silników
				if (balance_state_machine == 0 && balancing_mode > 1900
						&& balancing_mode < 2100) {
					balance_state_machine = 1;// przejście do stanu 1 - wstawanie jeżeli przełączono switcha
				}

				if (balance_state_machine == 1) {			// wstawanie
					Send(0, 0);
					leg_motor(-100);
				}

				if (balance_state_machine == 1 && theta > 70) {
					balance_state_machine = 2;// jeżeli osiągnięto 70 stopni - przejdź do trybu balansowania - stan 2
					leg_motor(0);
				}

				if (balance_state_machine == 2) {// balansowanie, kolejny stan zależy od wartości zwróconej przez vertical_control
					balance_state_machine = vertical_control(V_bok_apar, V_apar,
							V_max_apar, Fi_max_apar, Relay_SW, balancing_mode,
							theta);

					fall_counter = 0;
					hide_leg();
				}
				if (balance_state_machine == 3) {			// osiadanie
					//noga jest wysuwana do pewnego stopnia zależnego od liczby iteracji - teraz 550
					leg_motor(-90);
					Send(0, 0);
					fall_counter++;
					//gdy czas upłynie noga zostanie wyłączona i nastąpi przejście do stanu zero, w którym to noga znów zostanie schowana
					if (fall_counter > 550) {
						balance_state_machine = 0;
						leg_motor(0);
					}
				}

			}
			// loop_timer przepełni się po 70 minutach, jeśli robot ma działać dłużej należy zmienić metodę pomiaru czasu
			time = (getCurrentMicros() - loop_timer) * 1e-6;
			loop_timer = getCurrentMicros();
			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */
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
