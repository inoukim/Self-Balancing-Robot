#include "control.h"
#include "PID.h"
#include "mpu6050.h"
#include "hbridge.h"

extern float AX, AY, AZ, GY;


float pitch = 0;
int ref;
float angle = 0;
int8_t p_dir = STOP;

void calibrate_mpu(){
	ref = pitch;
}

void angle_to_pwm(){
	int pwm;

	pwm = PID(ref,angle,delay);
	Set_PWM(pwm);

}

void execute_control(){
	float pitch_gyro = 0, pitch_accel = 0;

	if (MPU6050_Read_All() == HAL_OK){
		pitch_accel = atan2(AX, sqrt(AY*AY +AZ*AZ) * (180/M_PI));
		pitch_gyro = pitch + GY * delay;
		pitch = CF_OFFSET_GYRO * pitch_gyro + CF_OFFSET_ACCEL * pitch_accel;
		angle_to_pwm();

	}

}
