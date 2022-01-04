/*
 * mpu6050.c
 *
 *  Created on: 13 paź 2021
 *      Author: Damian
 */

#include "mpu6050.h"

int16_t acc_rawX = 0;
int16_t acc_rawY = 0;
int16_t acc_rawZ = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

uint32_t loop_timer;
float gyro_calibr = 0, acc_calibr = 0;

float angle_acc, gyro_velocity = 0;

_Bool MPU6050_Init(void) {

	uint8_t check;
	uint8_t Data;
	_Bool mpu6050_status = 0;

	// sprawdzenie czy to MPU6050
	HAL_I2C_Mem_Read(mpuI2C, MPU6050_ADDR, WHO_AM_I, 1, &check, 1, 1000);

	if (check == 0x72) { // powinno być 0x68 ale w tym przypadku to jakiś klon
		// power management register: brak resetu, wyłączone tryby sleep i cycle, wyłączony czujnik temp. (bit wysoki), wewnętrzny zegar 8MHz
		Data = 0b00001000;
		HAL_I2C_Mem_Write(mpuI2C, MPU6050_ADDR, PWR_MGMT_1, 1, &Data, 1, 1000);

		// konfiguracja akcelerometru
		// bity 3 = 0; 4 = 0 - zakres akcelerometru na +-0g, pozostałe bity to selftest bądź zastrzeżone (oba nieistotne)
		Data = 0x00;
		HAL_I2C_Mem_Write(mpuI2C, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1,
				1000);

		// konfiguracja żyroskopu
		// bity 3 = 0; 4 = 0 - zakres żyroskopu na +-250 deg/s, pozostałe bity to selftest bądź zastrzeżone (oba nieistotne)
		Data = 0x00;
		HAL_I2C_Mem_Write(mpuI2C, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, 1000);

		Data = 0b00000110;////ostatnie 3 bity to filtr LPF, teraz jest najostrzejszy
		HAL_I2C_Mem_Write(mpuI2C, MPU6050_ADDR, CONFIG, 1, &Data, 1, 1000);

		gyro_calibr = kalibracja_gyro();
		acc_calibr = kalibracja_acc();
		mpu6050_status = 1;
	}
	loop_timer = HAL_GetTick();
	return mpu6050_status;
}

float MPU6050_Read_Accel(void) {
	uint8_t i2c_data[6];

	// odczytanie 6 bajtów począwszy od rejestru ACCEL_XOUT_H, wymagane dwa do obliczeń
	HAL_I2C_Mem_Read(mpuI2C, MPU6050_ADDR, ACCEL_XOUT_H, 1, i2c_data, 6, 1000);

	acc_rawX = (int16_t) (i2c_data[0] << 8 | i2c_data[1]);
	acc_rawY = (int16_t) (i2c_data[2] << 8 | i2c_data[3]);
	acc_rawZ = (int16_t) (i2c_data[4] << 8 | i2c_data[5]);

	//kąt jest atanem ilorazu przyspieszeń zatem nie trzeba konwertować do g, bo się skraca,
	//teraz dane są wyliczane dla osi Y, istotne jest ułożenie MPU6050
	angle_acc = atan2((float) acc_rawX, (float) acc_rawZ) * -180 / M_PI;

	return angle_acc - acc_calibr;
}

float MPU6050_Read_Gyro() {

	uint8_t i2c_data[6];	//odczytanie danych począwszy rejestru GYRO_XOUT_H
	HAL_I2C_Mem_Read(mpuI2C, MPU6050_ADDR, GYRO_XOUT_H, 1, i2c_data, 6, 1000);
	Gyro_X_RAW = (int16_t) (i2c_data[0] << 8 | i2c_data[1]);
	Gyro_Y_RAW = (int16_t) (i2c_data[2] << 8 | i2c_data[3]);
	Gyro_Z_RAW = (int16_t) (i2c_data[4] << 8 | i2c_data[5]);

	Gyro_Y_RAW -= gyro_calibr;          //uwzględnienie odczytu kalibracyjnego
	gyro_velocity = Gyro_Y_RAW / 131;

	return gyro_velocity;
}

float kalibracja_acc(void) {
	for (int i = 0; i < 5000; i++) {
		if (i % 400 == 0)
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); //migająca dioda - informacja o procesie kalibracji

		uint8_t i2c_data[6];
		HAL_I2C_Mem_Read(mpuI2C, MPU6050_ADDR, ACCEL_XOUT_H, 1, i2c_data, 6,
				1000);
		acc_rawX = (int16_t) (i2c_data[0] << 8 | i2c_data[1]);
		acc_rawY = (int16_t) (i2c_data[2] << 8 | i2c_data[3]);
		acc_rawZ = (int16_t) (i2c_data[4] << 8 | i2c_data[5]);

		acc_calibr += atan2((float) acc_rawX, (float) acc_rawZ) * -180 / M_PI;
	}
	acc_calibr /= 5000; //wartosc srednia

	return acc_calibr;
}

float kalibracja_gyro(void) {
	for (int i = 0; i < 5000; i++) {
		if (i % 400 == 0)
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); //migająca dioda - informacja o procesie kalibracji

		uint8_t i2c_data[6];
		HAL_I2C_Mem_Read(mpuI2C, MPU6050_ADDR, GYRO_XOUT_H, 1, i2c_data, 6,
				1000);

		Gyro_X_RAW = (int16_t) (i2c_data[0] << 8 | i2c_data[1]);
		Gyro_Y_RAW = (int16_t) (i2c_data[2] << 8 | i2c_data[3]);
		Gyro_Z_RAW = (int16_t) (i2c_data[4] << 8 | i2c_data[5]);
		gyro_calibr += Gyro_Y_RAW;

	}
	gyro_calibr /= 5000; //wartosc srednia

	return gyro_calibr;
}

