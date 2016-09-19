#include <stdint.h>
#include "stm32f411xe.h"




#define ENC_1_A_PIN 1
#define ENC_2_A_PIN 2
#define ENC_1_B_PIN 11
#define ENC_2_B_PIN 12

//encoder counts
volatile long LeftEncoderCounts = 0;
volatile long RightEncoderCounts = 0;

void encoder_init(void)
{

  int pins[4] = {ENC_1_A_PIN,ENC_1_B_PIN,ENC_2_A_PIN,ENC_2_B_PIN};
	int i;
	/* 1. enable GPIOA , encoders are plugged here (1 and 2 for A1 and A2, B in 11 and 12)*/
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //0x01
	//RCC_AHB1PeriphClockCmd(RCC_AHB1PeriphGPIOA,ENABLE);
	/* 2. set mode of gpioA pins set by encoder defines to input*/
	GPIOA->MODER &= ~ 0x300;
	
	for ( i = 0; i < 4; i++)
	{
		GPIOA->MODER |= ~ (0x03 << (pins[i] * 2));
	}
	
	/* 3. set interrupt triggering level for encoder 1A (EXTI1) and Encoder 2A (EXTI2) pins*/
	EXTI->FTSR |= (0x01 << ENC_1_A_PIN);
	EXTI->FTSR |= (0x01 << ENC_2_A_PIN);
	
	/* 4. enable the interupt over EXTI0*/
	EXTI->IMR |= (0x01 << ENC_1_A_PIN);
	EXTI->IMR |= (0x01 << ENC_2_A_PIN);
	
	/* 5. the interrupt on NVIC for IRQ#'s of exti1 and 2*/ //TODO: avoid hardcoding exti line and make it based on defines
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);
}


int main(void)
{
	encoder_init();
	while(1)
	{
		
	}
	
	
}


void EXTI1_IRQHandler(void)
{
		NVIC_ClearPendingIRQ(EXTI1_IRQn);
	//Left Encoder (Encoder 1)
	//Read from left encoder B to determine motor direction.
	//Refer to https://www.pololu.com/product/2827 for encoder documentation
	//This is triggered on a Falling edge trigger
	//If A falls and B is high, motor is turning forward
	//If A falls and B is low, motor is turning backwards.
	GPIOA->IDR;
	if (GPIOA->IDR & (0x01 << ENC_1_B_PIN))
	{//going forward
		LeftEncoderCounts++;
	}
	else
	{//going backwards
		LeftEncoderCounts--;
	}
	
}
void EXTI2_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
	//Right Encoder (Encoder 1)
	//Read from right encoder B to determine motor direction.
	//Refer to https://www.pololu.com/product/2827 for encoder documentation
	//This is triggered on a Falling edge trigger
	//If A falls and B is high, motor is turning forward
	//If A falls and B is low, motor is turning backwards.
	GPIOA->IDR;
	if (GPIOA->IDR & (0x01 << ENC_2_B_PIN))
	{//going forward
		RightEncoderCounts++;
	}
	else
	{//going backwards
		RightEncoderCounts--;
	}
}
