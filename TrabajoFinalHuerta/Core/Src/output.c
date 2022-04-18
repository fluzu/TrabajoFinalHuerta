#include "output.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"

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

typedef struct {
    GPIO_TypeDef *Port;
    uint16_t Pin;
    //agregar estado?
} Output_Typedef;

Output_Typedef CoveringDriverMotor_ENA = {CoveringDriverMotor_ENA_GPIO_Port, CoveringDriverMotor_ENA_Pin};
Output_Typedef CoveringDriverMotor_IN1 = {CoveringDriverMotor_IN1_GPIO_Port, CoveringDriverMotor_IN1_Pin};
Output_Typedef CoveringDriverMotor_IN2 = {CoveringDriverMotor_IN2_GPIO_Port, CoveringDriverMotor_IN2_Pin};

Output_Typedef ValveDriver_ENA = {ValveDriver_ENA_GPIO_Port, ValveDriver_ENA_Pin};
Output_Typedef ValveDriver_IN1 = {ValveDriver_IN1_GPIO_Port, ValveDriver_IN1_Pin};
Output_Typedef ValveDriver_IN2 = {ValveDriver_IN2_GPIO_Port, ValveDriver_IN2_Pin};

extern void *DriverMotor_ENA;
extern void *DriverMotor_IN1;
extern void *DriverMotor_IN2;

extern void *DriverValve_ENA;
extern void *DriverValve_IN1;
extern void *DriverValve_IN2;

void BSP_Output_Init(){
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin 	= CoveringDriverMotor_ENA_Pin|CoveringDriverMotor_IN1_Pin|CoveringDriverMotor_IN2_Pin|ValveDriver_ENA_Pin|ValveDriver_IN1_Pin|ValveDriver_IN2_Pin;
    GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull 	= GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CoveringDriverMotor_ENA_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_Init(CoveringDriverMotor_IN1_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_Init(CoveringDriverMotor_IN2_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_Init(ValveDriver_ENA_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_Init(ValveDriver_IN1_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_Init(ValveDriver_IN2_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(CoveringDriverMotor_ENA_GPIO_Port, CoveringDriverMotor_ENA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CoveringDriverMotor_IN1_GPIO_Port, CoveringDriverMotor_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CoveringDriverMotor_IN2_GPIO_Port, CoveringDriverMotor_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ValveDriver_ENA_GPIO_Port, ValveDriver_ENA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ValveDriver_IN1_GPIO_Port, ValveDriver_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ValveDriver_IN2_GPIO_Port, ValveDriver_IN2_Pin, GPIO_PIN_RESET);

    DriverMotor_ENA = &CoveringDriverMotor_ENA;
    DriverMotor_IN1 = &CoveringDriverMotor_IN1;
    DriverMotor_IN2 = &CoveringDriverMotor_IN2;

    DriverValve_ENA = &ValveDriver_ENA;
    DriverValve_IN1 = &ValveDriver_IN1;
    DriverValve_IN2 = &ValveDriver_IN2;
}

void BSP_Output_Toggle(void *output){
    Output_Typedef *out = (Output_Typedef*) output;
    HAL_GPIO_TogglePin(out->Port, out->Pin);
}

void BSP_Output_On(void *output){
    Output_Typedef *out = (Output_Typedef*) output;
    HAL_GPIO_WritePin(out->Port, out->Pin, GPIO_PIN_SET);
}

void BSP_Output_Off(void *output){
    Output_Typedef *out = (Output_Typedef*) output;
    HAL_GPIO_WritePin(out->Port, out->Pin, GPIO_PIN_RESET);
}
