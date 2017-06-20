#ifndef PINS_H
#define PINS_H
#include "stm32f4xx.h"

#define LEFT_MOTOR_PIN GPIO_PIN_6
#define RIGHT_MOTOR_PIN GPIO_PIN_7
#define MOTOR_PORT GPIOC

#define LEFT_MOTOR_PWM 10
#define RIGHT_MOTOR_PWM 6

#define GREEN_LED GPIO_PIN_12

#define LED_PORT GPIOD


// I2C pins

#define I2C_SCL_GPIO_PIN GPIO_PIN_6
#define I2C_SDA_GPIO_PIN GPIO_PIN_7
#define I2C_GPIO_PORT GPIOB


//Encoder
#define ENC_1_A_PIN 1
#define ENC_2_A_PIN 2
#define ENC_1_B_PIN 4
#define ENC_2_B_PIN 5

//encoder counts
extern volatile long LeftEncoderCounts;
extern volatile long RightEncoderCounts;
#define ENCODER_MAX 262143 //2^18-1

//LED lights for debugging
#define GREEN_LED GPIO_PIN_12
#define ORANGE_LED GPIO_PIN_13


#endif


