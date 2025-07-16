#include "motor.h"

// Initializes the motor PWM timers for all four channels
// Uses HAL library to start PWM with interrupt capability
void MOTOR_Init()
{
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1); // Back Left motor
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2); // Back Right motor
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3); // Front Right motor
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4); // Front Left motor
}

// Sets the speed for each motor based on the provided MotorSpeed structure
// Uses PWM compare values to control motor speeds
void MOTOR_SetSpeed(MotorSpeed speed)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed.bl); // Set Back Left motor speed
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed.br); // Set Back Right motor speed
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed.fr); // Set Front Right motor speed
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed.fl); // Set Front Left motor speed
}
