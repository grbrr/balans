/*
 * mpu6050.h
 *
 *  Created on: 13 paź 2021
 *      Author: Damian
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "i2c.h"
#include <math.h>

#define mpuI2C &hi2c1

#define MPU6050_ADDR 0x68<<1 //adres I2C wymaga przesunięcia bitowego

#define WHO_AM_I 0x75		//powinno być 0x68 ale to chiński klon, w przypadku problemów z innymi chipami zastosować jakiś I2C scanner
#define PWR_MGMT_1 0x6B		//info o rejestrach w dokumentacji mpu-6050

#define SMPLRT_DIV 0x19
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H 0x41
#define GYRO_XOUT_H 0x43
#define CONFIG 0x1A

_Bool MPU6050_Init(void);
float MPU6050_Read_Accel(void);
float MPU6050_Read_Gyro(void);
float kalibracja_acc(void);
float kalibracja_gyro(void);

#endif /* INC_MPU6050_H_ */
