/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

//#define STM32F411xE
#define USE_HAL_DRIVER
#include "stdint.h"

#include "stm32f4xx.h"
#include "pwmlib.h"
#include "i2clib.h"
//#include "stm32f072b_discovery.h"
void SystemClock_Config(void);	
void Error_Handler(void);

//encoder counts
volatile long LeftEncoderCounts = 0;
volatile long RightEncoderCounts = 0;

void encoder_init(void)
{
//  int pins[4] = {ENC_1_A_PIN,ENC_1_B_PIN,ENC_2_A_PIN,ENC_2_B_PIN};
//	int i;
	/* 1. enable GPIOA , encoders are plugged here (1 and 2 for A1 and A2, B in 11 and 12)*/
	__GPIOA_CLK_ENABLE();
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //0x01
	//RCC_AHB1PeriphClockCmd(RCC_AHB1PeriphGPIOA,ENABLE);
	/* 2. set mode of gpioA pins set by encoder defines to input*/
	GPIO_InitTypeDef GPIO_InitStruct = {
					.Pin = ENC_1_A_PIN | ENC_2_A_PIN,
					.Mode = GPIO_MODE_IT_FALLING,
					.Pull = GPIO_NOPULL,
					//.Speed = GPIO_SPEED_FREQ_HIGH, //?
	};
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = ENC_1_B_PIN | ENC_2_B_PIN,
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT,
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* 3. set interrupt triggering level for encoder 1A (EXTI1) and Encoder 2A (EXTI2) pins*/
	EXTI->FTSR |= (0x01 << ENC_1_A_PIN);
	EXTI->FTSR |= (0x01 << ENC_2_A_PIN);
	
	/* 4. enable the interupt over EXTI0*/
	EXTI->IMR |= (0x01 << ENC_1_A_PIN);
	EXTI->IMR |= (0x01 << ENC_2_A_PIN);
	
	HAL_NVIC_SetPriority(EXTI1_IRQn,0,0);
	HAL_NVIC_SetPriority(EXTI2_IRQn,0,0);

		//HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	/* 5. the interrupt on NVIC for IRQ#'s of exti1 and 2*/ //TODO: avoid hardcoding exti line and make it based on defines
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);
		
}

void debug_GPIO_init(void) {
	__GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {
			.Pin = GREEN_LED | ORANGE_LED,
			.Mode = GPIO_MODE_OUTPUT_PP,
			.Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_FREQ_HIGH
	};
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	//TODO: add user button (A0)
}

int main(void)
{
	HAL_Init();

	  SystemClock_Config();
	PWMLIB_Init(LEFT_MOTOR_PWM);
	encoder_init();
	debug_GPIO_init();
	I2C_init(I2C1);
	I2C_Device_t sensor;
	I2C_Device_t arduino;
	I2C_slave_init(&sensor,I2C1,0x30,1000);
	I2C_slave_init(&arduino,I2C1,0x08,0xFFFFFFFF);



	while(1) {
/*
		Communication strategy:
		We are getting desired motor speeds from the Raspberry Pi communication node
		For now, this will consist of the desired speed for both wheels, as a byte
		Form of each byte: SXXX_XXXX
			S: sign - 1 = negative, 0 = positive
				-	This determined whether we set the reverse pin on each motor or not
			XXX_XXXX: value from 0-127
				- This is the pwm speed we will set for each motor. We may desire to bump this to 2 bytes for better resolution
		
		We want to tell the Pi how far we have gone in the return message.
		The raspberry pi likes to stick in 1 as the most significant bit of the bytes it receives so we need a workaround
		Each wheel's message will be as following:
		
		00SF_XXXX 0XXX_XXXX 0XXX_XXXX
		0 - bits we leave alone because the pi will likely mess them up
		S - sign of the number sent (2's complement would be better but I don't trust the Pi to get conversions the way we want them...)
		F - Flag. We will set this bit if we couldn't fit all the encoder ticks into the message
				This means the pi isnt messaging often enough
		XX...XXXX - 18 bit number of the number of encoder counts for the wheel since the last message sequence
								This allows 262,144 encoder ticks per message signal, more than enough (this is many tens of rotations)
		
		
		
		We will have an infinite timeout on waiting for the first message -- it can come whenever
		We will have a shorter timeout on the second message -- we don't want to get stuck trying to send 
				when we are supposed to be receiving.
		
*/
		uint8_t data_out[6] = {0};
		uint8_t data_in[2] = {0,0};	
		I2C_receive_slave(&arduino,data_in,2);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); //Toggle the state of Green LED

		PWMLIB_set_value(LEFT_MOTOR_PWM, data_in[0] & 0x7F);
		PWMLIB_set_value(RIGHT_MOTOR_PWM, data_in[1] & 0x7F);

		
		int32_t temp_left = LeftEncoderCounts;
		int32_t temp_right = RightEncoderCounts;
		LeftEncoderCounts = 0;
		RightEncoderCounts = 0;
		
		uint8_t left_neg = 0;
		uint8_t right_neg = 0;
		if (temp_left < 0) {
			left_neg = 1;
			temp_left = -temp_left;
		}
		if (temp_right < 0) {
			right_neg = 1;
			temp_right = -temp_right;
		}
		
		data_out[0] = (temp_left &0x000FC000) >> 14;		//00000000_00000011_11000000_00000000
		data_out[1] = (temp_left &0x00003F80) >> 7;		//00000000_00000000_00111111_10000000
		data_out[2] = (temp_left &0x0000007F);				//00000000_00000000_00000000_01111111
		data_out[0] |= (left_neg << 5);
		if (temp_left >= ENCODER_MAX) data_out[0] |= 0x10; //0b0001_0000
		
		data_out[3] = (temp_right &0x000FC000) >> 14;		//00000000_00000011_11000000_00000000
		data_out[4] = (temp_right &0x00003F80) >> 7;		//00000000_00000000_00111111_10000000
		data_out[5] = (temp_right &0x0000007F);				//00000000_00000000_00000000_01111111
		data_out[3] |= (right_neg << 5);
		if (temp_right >= ENCODER_MAX) data_out[3] |= 0x10; //0b0001_0000

		I2C_send_slave(&arduino,data_out,6);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); //Toggle the state of green LED

		
	}
}



void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	
		/* Enable appropriate peripheral clocks */
	__SYSCFG_CLK_ENABLE();
}


void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

