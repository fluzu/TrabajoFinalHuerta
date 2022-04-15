
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f4xx_hal.h"


void APP_Timer10ms();
void APP_Timer100ms();
void APP_Timer1000ms();
void APP_Timer10s();

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


void Error_Handler(void);


#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define Audio_SDA_Pin GPIO_PIN_9
#define Audio_SDA_GPIO_Port GPIOB


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

