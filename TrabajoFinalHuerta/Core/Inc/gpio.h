#ifndef __GPIO__
#define __GPIO__

#include "stm32f4xx_hal_def.h"

//#define 	VERDE				GPIO_PIN_13

#define 	GPIO_PIN_LED		GPIO_PIN_13
#define 	GPIO_PORT_LED		GPIOC

#define 	GPIO_PIN_BUTTON		GPIO_PIN_1
#define 	GPIO_PORT_BUTTON	GPIOA

void gpio_init(void);
void gpio_led_high(void);
void gpio_led_low(void);
void gpio_led_toggle(void);

#endif
