#include "pid.h"
#include <stdlib.h>


#define Kp 11
#define Kd 0.4
#define Ki 2

#define MAX_PWM 65
#define MIN_PWM -65




int PID(float ref, float pitch) {

	static float lastError;
	float P, I, D, pid_pwm;

	//calculate error
	float error = ref - pitch;

	//calculate Proportional term
	P = Kp * error;

	//calculate Integral term. Account for wind-up
	I += Ki* error ;

	if (I > MAX_PWM)
		I = MAX_PWM;
	else if (I<MIN_PWM){
		I=MIN_PWM;
	}

	//calculate Derivative term
	D = -Kd * ((error - lastError)/0.03);

	//total PID value
	pid_pwm = P + I + D;

	//max sure pwm is bound between allowed min/max thresholds

	int out_pwm = (int) pid_pwm;
	if (pid_pwm > MAX_PWM)
		out_pwm = MAX_PWM;
	else if (pid_pwm < MIN_PWM)
		out_pwm = MIN_PWM;

	lastError = error;

	return out_pwm;
}
