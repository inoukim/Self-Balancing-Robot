/*
 * pid.c
 *
 *  Created on: Nov 13, 2023
 *      Author: vinhc
 */


#include "pid.h"
#include <stdlib.h>
#define Kp //value?
#define Kd //value?
#define Ki //value?
#define Max_PWM 70 //for now
#define Min_PWM -Max_PWM


int PID(float ref, float pitch, PID_flag) {

	float P, D, PID_PWM;
	float error = ref - pitch;

	//set point = 0

	// P term
	P = Kp * error;


	// I term
	if (PID_flag )


	// D term


	// PID
	PID_PWM = P + I + D;

}
