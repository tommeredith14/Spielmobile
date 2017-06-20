/*********************************************************

**********************************************************/

#include "pwmlib.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal.h"


// Initializes the PWM GPIO pins and clocks. Automatically called by HAL on initialization.
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;

    // General GPIO settings (same for all pins)
    GPIO_InitStruct.Mode        = PWM_GPIO_MODE;
    GPIO_InitStruct.Pull        = PWM_GPIO_PULL;
    GPIO_InitStruct.Speed       = PWM_GPIO_SPEED;


    GPIO_InitStruct.Pin         = LEFT_MOTOR_PIN | RIGHT_MOTOR_PIN;
    GPIO_InitStruct.Alternate   = GPIO_AF2_TIM3;

    HAL_GPIO_Init(MOTOR_PORT, &GPIO_InitStruct);

}

TIM_HandleTypeDef htim3;

int PWMLIB_Init(uint32_t pwm_id)
{
  int error = 0;

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PWM_PERIOD;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    error = 1;
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    error = 2;
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    error = 3;
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    error = 4;
  }

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,0);
	
    return error;
}

void PWMLIB_set_value ( uint32_t pwm_id, uint32_t value) {
	  uint32_t channel;
		TIM_HandleTypeDef *htim;
		switch (pwm_id) {
			case LEFT_MOTOR_PWM:
				htim = &htim3;
				channel = TIM_CHANNEL_1;
				break;
			case RIGHT_MOTOR_PWM:
				htim = &htim3;
				channel = TIM_CHANNEL_2;
				break;
			default:
				return;
		}
		
		__HAL_TIM_SET_COMPARE(htim,channel, value);
}
