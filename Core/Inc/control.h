/*
 * control.h
 *
 *  Created on: Nov 28, 2023
 *      Author: Inou
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#define delay (0.001)
#define M_PI (3.141592)
#define CF_OFFSET_GYRO (0.98)
#define CF_OFFSET_ACCEL (1-CF_OFFSET_GYRO)

void calibrate_mpu();
void angle_to_pwm();
void execute_control();
#endif /* INC_CONTROL_H_ */
