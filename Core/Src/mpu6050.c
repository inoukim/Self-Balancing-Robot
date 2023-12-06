#include "mpu6050.h"

extern 	I2C_HandleTypeDef hi2c1;

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float AX, AY, AZ, GX, GY, GZ;

int8_t MPU6050_Init(void)
{
	uint8_t check, Data;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104) //if register value is 0x68 the device is there
	{
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000); //Wake up all the sensors
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000); //Set the Sampling rate to 1KHz
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG,1, &Data, 1, 1000); //Range is +/- 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG,1, &Data, 1, 1000); //Range is +/- 250 degree
		return MPU6050_OK;
	}
	return MPU6050_I2C_ERR;
}

int8_t MPU6050_Read_Accel(void)
{
	uint8_t R_data[6];
	HAL_StatusTypeDef returnValue;
	returnValue = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG,1, R_data, 6, 1000);
	Accel_X_RAW = (int16_t)(R_data[0] << 8 | R_data[1]);
	Accel_Y_RAW = (int16_t)(R_data[2] << 8 | R_data[3]);
	Accel_Z_RAW = (int16_t)(R_data[4] << 8 | R_data[5]);
	AX = Accel_X_RAW / 16384.0; //16,384 is pulled from the datasheet sensivity scale
	AY = Accel_Y_RAW / 16384.0;
	AZ = Accel_Z_RAW / 16384.0;
	if (returnValue != HAL_OK) {
		return MPU6050_I2C_ERR;
	}
	return MPU6050_OK;

}

int8_t MPU6050_Read_Gyro(void)
{
	uint8_t R_data[6];
	HAL_StatusTypeDef returnValue;
	returnValue = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG,1, R_data, 6, 1000);
	Gyro_X_RAW = (int16_t)(R_data[0] << 8 | R_data[1]);
	Gyro_Y_RAW = (int16_t)(R_data[2] << 8 | R_data[3]);
	Gyro_Z_RAW = (int16_t)(R_data[4] << 8 | R_data[5]);

	GX = Gyro_X_RAW /131.0; //131.0 is pulled from the datasheet sensivity scale
	GY = Gyro_Y_RAW /131.0;
	GZ = Gyro_Z_RAW /131.0;
	if (returnValue != HAL_OK) {
		return MPU6050_I2C_ERR;
	}
	return MPU6050_OK;
}
int8_t MPU6050_Read_All(void)
{
	uint8_t R_data[14];
	HAL_StatusTypeDef returnValue;
	returnValue = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG,1, R_data, 14, 1000);
	Accel_X_RAW = (int16_t)((R_data[0] << 8) | R_data[1]);
	Accel_Y_RAW = (int16_t)((R_data[2] << 8) | R_data[3]);
	Accel_Z_RAW = (int16_t)((R_data[4] << 8) | R_data[5]);
	Gyro_X_RAW = (int16_t)((R_data[8] << 8) | R_data[9]);
	Gyro_Y_RAW = (int16_t)((R_data[10] << 8) | R_data[11]);
	Gyro_Z_RAW = (int16_t)((R_data[12] << 8) | R_data[13]);

	AX = Accel_X_RAW / 16384.0; //16,384 is pulled from the datasheet sensivity scale
	AY = Accel_Y_RAW / 16384.0;
	AZ = Accel_Z_RAW / 16384.0;
	GX = Gyro_X_RAW /131.0; //131.0 is pulled from the datasheet sensivity scale
	GY = Gyro_Y_RAW /131.0;
	GZ = Gyro_Z_RAW /131.0;

	if (returnValue != HAL_OK) {
		return MPU6050_I2C_ERR;
	}
	return MPU6050_OK;
}


