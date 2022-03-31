#include "stm32f4xx_hal.h"
#include "gpio.h"

void gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
	
	/*Configure GPIO pins : PA1 */
  GPIO_InitStruct.Pin 	= GPIO_PIN_BUTTON;
  GPIO_InitStruct.Mode 	= GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_PORT_BUTTON, &GPIO_InitStruct);
	
  /*Configure GPIO pins : PC13 */
  GPIO_InitStruct.Pin 	= GPIO_PIN_LED;
  GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_PORT_LED, &GPIO_InitStruct);
	
	/*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_PORT_LED, GPIO_PIN_LED, GPIO_PIN_RESET);

}

void gpio_led_high(void)		{ HAL_GPIO_WritePin(GPIO_PORT_LED, GPIO_PIN_LED, GPIO_PIN_SET); 	}
void gpio_led_low(void)   	{ HAL_GPIO_WritePin(GPIO_PORT_LED, GPIO_PIN_LED, GPIO_PIN_RESET);	}
void gpio_led_toggle(void)	{ HAL_GPIO_TogglePin(GPIO_PORT_LED, GPIO_PIN_LED); }


