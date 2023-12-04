#include "hbridge.h"
#include "stdlib.h"

extern TIM_HandleTypeDef htim2;

void Set_PWM(int PWM) {
	int8_t direction;
	uint16_t speed = abs(PWM);

	// Direction is based on angle read from MPU6050
	if (PWM > 0){
		direction = BACKWARD;
	} else if (PWM < 0){
		direction = FORWARD;
	}
	else
		direction = STOP;


	if (direction == FORWARD) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed);

	} else if (direction == BACKWARD) {

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed);

	} else if (direction == STOP) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);

	}

}
