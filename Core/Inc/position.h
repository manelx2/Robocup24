/*
 * position.h
 *
 *  Created on: Sep 20, 2024
 *      Author: User
 */

#ifndef INC_POSITION_H_
#define INC_POSITION_H_
#define PI (float) 3.14159265358979323846
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"

void set_right(TIM_HandleTypeDef* htim, TIM_TypeDef* TIM, int resolution, int precision, int sens);
void set_left(TIM_HandleTypeDef* htim, TIM_TypeDef* TIM, int resolution, int precision, int sens);
void set_dimentions(float right_wheel_radius, float left_wheel_radius, float encoder_spacing, float wheels_spacing);


void read_right(void);
void read_left(void);



float distance(int x, float r, int resolution, int precision);
float angle(double x);


#endif /* INC_POSITION_H_ */
