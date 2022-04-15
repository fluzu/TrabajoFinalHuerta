//
// Created by Facu on 13/04/2022.
//

#ifndef TESTLCDI2C_BSP_H
#define TESTLCDI2C_BSP_H

#include "stm32f4xx_hal.h"

#define LD4_Pin GPIO_PIN_12

void BSP_Init(void);
void BSP_LCD_Temperature(float);
void BSP_LCD_Humidity(float);
uint32_t BSP_Get_percentageHS(uint32_t);

void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#endif //TESTLCDI2C_BSP_H










