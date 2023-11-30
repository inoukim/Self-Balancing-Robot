#include "control.h"
#include "pid.h"
#include "mpu6050.h"
#include "hbridge.h"

extern float AX, AY, AZ, GY;


float pitch = 0;
int ref;
float angle = 0;
int8_t p_dir = STOP;

void calibrate_mpu(){
	loop();
	ref = pitch;
}

void angle_to_pwm(){
	int pwm;

	pwm = PID(ref,angle);
	Set_PWM(pwm);

}

void loop(){
	float pitch_gyro = 0, pitch_accel = 0;
	int8_t loop_flag = MPU6050_Read_All();
	if (loop_flag == MPU6050_OK){
		pitch_accel = atan2(AX, sqrt(AY*AY +AZ*AZ) * (180/M_PI));
		pitch_gyro = pitch + GY * delay;
		pitch = CF_OFFSET_GYRO * pitch_gyro + CF_OFFSET_ACCEL * pitch_accel;
		angle_to_pwm();

	}

}
