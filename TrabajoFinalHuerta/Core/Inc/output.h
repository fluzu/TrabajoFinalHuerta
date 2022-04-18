
#include "stm32f4xx_hal.h"

#ifndef TESTLCDI2C_OUTPUT_H
#define TESTLCDI2C_OUTPUT_H

#define CoveringDriverMotor_ENA_Pin GPIO_PIN_4
#define CoveringDriverMotor_ENA_GPIO_Port GPIOD

#define CoveringDriverMotor_IN1_Pin GPIO_PIN_3
#define CoveringDriverMotor_IN1_GPIO_Port GPIOD

#define CoveringDriverMotor_IN2_Pin GPIO_PIN_2
#define CoveringDriverMotor_IN2_GPIO_Port GPIOD

#define ValveDriver_ENA_Pin GPIO_PIN_12
#define ValveDriver_ENA_GPIO_Port GPIOC

#define ValveDriver_IN1_Pin GPIO_PIN_11
#define ValveDriver_IN1_GPIO_Port GPIOC

#define ValveDriver_IN2_Pin GPIO_PIN_10
#define ValveDriver_IN2_GPIO_Port GPIOC

void BSP_Output_Init(void);
void BSP_Output_Toggle(void *output);
void BSP_Output_On(void *output);
void BSP_Output_Off(void *output);



#endif //TESTLCDI2C_OUTPUT_H
