#include "control.h"
#include "pid.h"
#include "mpu6050.h"
#include "hbridge.h"

extern float AX, AY, AZ, GX , GY, GZ;


float pitch = 0;
float ref = 0;
int8_t p_dir = STOP;
static float pitch_gyro = 0, pitch_accel = 0;
int count = 0;
int pwm;


void angle_to_pwm(){


	pwm = PID(ref,pitch);
	Set_PWM(pwm);

}

void loop(){

	int8_t loop_flag = MPU6050_Read_All();
	if (loop_flag == MPU6050_OK && cal_flag == 1){
		//pitch_accel = atan2(AX, sqrt(AY*AY + AZ*AZ) * (180/M_PI));
	    pitch_accel = atan2(AY, sqrt(pow(AX, 2) + pow(AZ, 2))) * (180/M_PI);
		pitch_gyro = pitch + GX * delay;
		pitch = CF_OFFSET_GYRO * pitch_gyro + CF_OFFSET_ACCEL * pitch_accel;
		angle_to_pwm();

	}
	else{
		pitch_accel = atan2(AX, sqrt(AY*AY +AZ*AZ) * (180/M_PI));
		pitch_gyro = pitch + GX * delay;
		pitch = CF_OFFSET_GYRO * pitch_gyro + CF_OFFSET_ACCEL * pitch_accel;
	}

}
