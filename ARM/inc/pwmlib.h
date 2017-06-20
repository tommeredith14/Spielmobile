
#ifndef PWMLIB_H_
#define PWMLIB_H_
#include "stm32f4xx.h"
#include "pins.h"
#include "stdint.h"

// PWM GPIO configurations
#define PWM_GPIO_MODE               GPIO_MODE_AF_PP
#define PWM_GPIO_PULL               GPIO_PULLUP
#define PWM_GPIO_SPEED              GPIO_SPEED_FREQ_HIGH
//TODO: made more of the configurations constants
// PWM timer configuration
#define PWM_PRESCALER               47  
#define PWM_PERIOD                  127 

// Public Functions
int PWMLIB_Init(uint32_t pwm_id);
void PWMLIB_set_value ( uint32_t pwm_id, uint32_t value);


#endif
