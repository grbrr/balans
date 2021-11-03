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

#define MPU6050_ADDR 0xD0

#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B

#define SMPLRT_DIV 0x19
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H 0x41
#define GYRO_XOUT_H 0x43

_Bool MPU6050_Init(void);
float MPU6050_Read_Accel(void);
float MPU6050_Read_Gyro(float time);
float kalibracja_acc(void);
float kalibracja_gyro(void);

#endif /* INC_MPU6050_H_ */
