//
// Created by Facu on 13/04/2022.
//

#ifndef TESTLCDI2C_BSP_H
#define TESTLCDI2C_BSP_H

#include "stm32f4xx_hal.h"

#define LD4_Pin GPIO_PIN_12



void BSP_Init(void);

void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#endif //TESTLCDI2C_BSP_H










