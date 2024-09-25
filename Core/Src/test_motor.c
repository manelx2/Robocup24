/*
 * test_motor.c
 *
 *  Created on: Sep 21, 2024
 *      Author: User
 */
#include "test_motor.h"
#define PWM_CHANNEL TIM_CHANNEL_1      // Use the correct channel for your motor control timer
#define MOTOR_PWM_SPEED 800            // Adjust this for the desired motor speed (range: 0 to max PWM value)
#define MOTOR_GPIO_PIN GPIO_PIN_X      // Replace X with the actual motor control GPIO pin
#define MOTOR_GPIO_PORT GPIOA          // Replace with the actual motor control GPIO port

extern TIM_HandleTypeDef htimX;        // Define the timer handle (replace htimX with your actual timer handle)

// Function to start motor rotation
void motor_rotate(void) {
    // Set motor direction (if using an H-Bridge or motor driver, set forward direction)
    HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_GPIO_PIN, GPIO_PIN_SET);

    // Set the PWM speed for motor rotation
    __HAL_TIM_SET_COMPARE(&htimX, PWM_CHANNEL, MOTOR_PWM_SPEED);  // Set PWM for motor rotation
}

// Function to stop the motor
void motor_stop(void) {
    // Set the PWM speed to 0 to stop the motor
    __HAL_TIM_SET_COMPARE(&htimX, PWM_CHANNEL, 0);

    // Optionally, reset the motor direction pin if needed
    HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_GPIO_PIN, GPIO_PIN_RESET);
}
