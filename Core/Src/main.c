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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "hoverserial.h"
#include "mpu6050.h"
#include "ibus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RC_CH1_NEUTRAL   1500.0
#define RC_CH1_MAX       2000.0
#define RC_CH1_MIN       1000.0

#define RC_CH2_NEUTRAL   1500.0
#define RC_CH2_MAX       2000.0
#define RC_CH2_MIN       1000.0

#define mpu6050
//#define aparatura

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char buffer[100];
uint8_t byte;
float acctheta = 0, gyrotheta = 0, theta = 0, gyrotesttheta = 0;
unsigned long loop_timer;
float time = 0;
_Bool mpu6050_ready = 0, start = 0;
float potentiometer;

int ograniczenie_regulatora = 200;
float k_p = 25;
float k_i = 0.3;
float k_d = 1200;
float e_n, suma_e_n = 0, theta_ref = 80, pid_output = 0, poprzedni_e_n = 0;

int16_t Relay = 11;
int16_t Jazda = 0;

int16_t Robot_V = 0;
int16_t Robot_Fi = 0;
int16_t V_apar = 0;
int16_t V_bok_apar = 0;
int16_t V_max_apar = 0;
int16_t Fi_max_apar = 0;
int16_t V = 0;
int16_t V_bok = 0;
int16_t V_max = 0;
int16_t Fi_max = 0;
int16_t VL = 0;
int16_t VR = 0;
int16_t VL_out = 0;
int16_t VR_out = 0;
int16_t V_nierownosc = 0;
int16_t Relay_SW = 0;
int16_t Funkcja_SW = 0;

uint16_t ibus_data[IBUS_USER_CHANNELS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline uint32_t LL_SYSTICK_IsActiveCounterFlag(void) {
	return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
			== (SysTick_CTRL_COUNTFLAG_Msk));
}

uint32_t getCurrentMicros(void) {
	/* Ensure COUNTFLAG is reset by reading SysTick control and status register */
	LL_SYSTICK_IsActiveCounterFlag();
	uint32_t m = HAL_GetTick();
	const uint32_t tms = SysTick->LOAD + 1;
	__IO uint32_t u = tms - SysTick->VAL;
	if (LL_SYSTICK_IsActiveCounterFlag()) {
		m = HAL_GetTick();
		u = tms - SysTick->VAL;
	}
	return (m * 1000 + (u * 1000) / tms);
}

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
		int32_t out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
float mapfloat(float x, float in_min, float in_max, float out_min,
		float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

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
	/* USER CODE BEGIN 2 */
	mpu6050_ready = MPU6050_Init();	//może podmienić rezystory na I2C bo musiałem dać pullup software'owy
	ibus_init();
	HAL_UART_Receive_IT(&huart2, &byte, sizeof(byte)); //oczekiwanie na przerwanie
	loop_timer = getCurrentMicros();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (mpu6050_ready) {

			// Check for new received data
			Receive(&byte);

			ibus_read(ibus_data);
			ibus_soft_failsafe(ibus_data, 10); // if ibus is not updated, clear ibus data.

			V_bok_apar = ibus_data[1 - 1];	//predkosc boki
			V_apar = ibus_data[2 - 1];   //predkosc
//			Funkcja_SW = ibus_data[7-1];
			Relay_SW = ibus_data[6 - 1];	//zalacz silniki
			V_max_apar = ibus_data[5 - 1];	//regulacja predkosci silnikow
//			Fi_max_apar = ibus_data[8-1];

#ifdef aparatura

			V_max = map(V_max_apar, 1000, 2000, 0, 400);
			//                                      / tu jest wartocm maskymalnej rotacji
			Fi_max = map(Fi_max_apar, 1000, 2000, 0, 200);

			if ((Relay_SW > 1900) && (Relay_SW < 2100)) {
				Jazda = 1;
			} else {
				Jazda = 0;
			}

			if (Jazda == 1) {
				Robot_V = map(V_apar, 1000, 2000, -V_max, V_max);
				Robot_Fi = map(V_bok_apar, 1000, 2000, -Fi_max, Fi_max);
			} else {
				Robot_V = 0;
				Robot_Fi = 0;
			}
			if ((Robot_V < 5) && (Robot_V > -5))
				Robot_V = 0;
			if ((Robot_Fi < 5) && (Robot_Fi > -5))
				Robot_Fi = 0;

			Send(Robot_Fi, Robot_V);
			HAL_Delay(7);
#endif

			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */
#ifdef mpu6050
			potentiometer = 0.0;
			for (int i = 0; i < 10; i++) {
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, 100);
				potentiometer += mapfloat(HAL_ADC_GetValue(&hadc1), 0, 1023, -5,
						5);
			}
			potentiometer /= 10;

//			sprintf(buffer, "%3.2f %3.2f\n\r", potentiometer, theta);
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);

			acctheta = MPU6050_Read_Accel();
			gyrotheta = MPU6050_Read_Gyro(time);
			gyrotesttheta += (gyrotheta * time);
			theta = 0.9995 * (theta + gyrotheta * time)
					+ (1 - 0.9995) * acctheta;
//			sprintf(buffer, "%3.2f, %3.2f, %3.2f\n\r", acctheta, gyrotesttheta,
//					theta);
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);

			//bezpieczenstwo
			if (start == 0 && (acctheta > (theta_ref - 0.5))
					&& (acctheta < (theta_ref + 0.5))) {
				start = 1;
			}

	//		Relay_SW = getValuePPM(6);	//zalacz silniki
			if ((Relay_SW > 1900) && (Relay_SW < 2100)
					&& (theta > (theta_ref - 30)) && (theta < (theta_ref + 30))
					&& start == 1) {
				//digitalWrite(Relay,HIGH);
				Jazda = 1;
				//gyrotheta=theta;
			} else {
				//digitalWrite(Relay,LOW);
				Jazda = 0;
				suma_e_n = 0;
				pid_output = 0;
				start = 0;
			}
//			sprintf(buffer, "%d, %d, %3.2f, %d\n\r", start, Relay_SW, theta, Jazda);
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);

			//definicja uchybu - aktualny kat odjac kat zadany
			//theta_ref += x;
			e_n = theta_ref - theta - potentiometer;

			//Obliczenie i ograniczenie sumy wszystkich błędów
			suma_e_n += e_n;
			if (suma_e_n > ograniczenie_regulatora)
				suma_e_n = ograniczenie_regulatora;
			else if (suma_e_n < -ograniczenie_regulatora)
				suma_e_n = -ograniczenie_regulatora;
			//PID
			pid_output = k_p * e_n + k_i * suma_e_n
					+ k_d * (e_n - poprzedni_e_n);

			//ograniczenie wyjścia PID
			if (pid_output > ograniczenie_regulatora)
				pid_output = ograniczenie_regulatora;
			else if (pid_output < -ograniczenie_regulatora)
				pid_output = -ograniczenie_regulatora;

			//Zapamiętanie ostatniego błędu
			poprzedni_e_n = e_n;

			//przełącznik histerezowy (zapobiega ciągłym próbom regulacji w pobliżu theta_ref)
//			if (pid_output < theta_ref+0.5 && pid_output > theta_ref-0.5)
//				pid_output = 0;

			if (Jazda == 1) {
				Robot_V = pid_output + 0;			//+x
				Robot_Fi = 0;
			} else {
				Robot_V = 0;
				Robot_Fi = 0;
			}

			Send(Robot_Fi, Robot_V);
			//sprintf(buffer, "Robot_Fi: %d Robot_V: %d\n\r", Robot_Fi, Robot_V);
			sprintf(buffer, "%d %d\n\r", Robot_Fi, Robot_V);
			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
#endif
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) { //sprawdzenie czy przyszło z właściwego usarta
		data_available = 1;
		//ponowne wywołanie oczekiwania na przerwania dzieje się po przetworzeniu danych w hoverserial.c
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
