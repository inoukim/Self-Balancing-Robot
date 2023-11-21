#ifndef INC_PID_H_
#define INC_PID_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

int PID(float ref, float pitch, float dt);

#endif /* INC_PID_H_ */
