/*
 * mpu6050.h
 *
 *  Created on: Nov 9, 2023
 *      Author: inou
 */
#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"

#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19 	//sample rate divider register
#define GYRO_CONFIG_REG 0x1B	//Gyroscope config register
#define ACCEL_CONFIG_REG 0x1C	//Accelerometer config register
#define ACCEL_XOUT_H_REG 0x3B	//Accelerometer X_measurement
#define GYRO_XOUT_H_REG 0x43	//Gyroscope X Measurement
#define PWR_MGMT_1_REG 0x6B		//Power Management 1 Register
#define WHO_AM_I_REG 0x75		//WHO am I register

void MPU6050_Init();
void MPU6050_Read_Accel();
void MPU6050_Read_Gyro();

#endif /* INC_MPU6050_H_ */
