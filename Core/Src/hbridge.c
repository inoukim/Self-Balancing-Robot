	#include "hbridge.h"

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	uint16_t readValue;
	int speed;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
