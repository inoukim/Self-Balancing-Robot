/*
 * hbridge.h
 *
 *  Created on: Nov 11, 2023
 *      Author: Inou
 */

#ifndef INC_HBRIDGE_H_
#define INC_HBRIDGE_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define FORWARD (1)
#define BACKWARD (-1)
#define STOP (0)

void Set_PWM(int PWM);

#endif /* INC_HBRIDGE_H_ */
