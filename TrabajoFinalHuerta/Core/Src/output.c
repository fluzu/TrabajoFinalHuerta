#include "output.h"
#include "stm32f4xx_hal.h"

#define CoveringDriverMotor_ENA_Pin GPIO_PIN_4
#define CoveringDriverMotor_ENA_GPIO_Port GPIO_PIN_D

#define CoveringDriverMotor_IN1_Pin GPIO_PIN_3
#define CoveringDriverMotor_IN1_GPIO_Port GPIO_PIN_D

#define CoveringDriverMotor_IN2_Pin GPIO_PIN_2
#define CoveringDriverMotor_IN2_GPIO_Port GPIO_PIN_D

#define ValveDriver_ENA_Pin GPIO_PIN_12
#define ValveDriver_ENA_GPIO_Port GPIO_PIN_C

#define ValveDriver_IN1_Pin GPIO_PIN_11
#define ValveDriver_IN1_GPIO_Port GPIO_PIN_C

#define ValveDriver_IN2_Pin GPIO_PIN_10
#define ValveDriver_IN2_GPIO_Port GPIO_PIN_C

typedef struct {
    GPIO_TypeDef *Port;
    uint16_t Pin;

} Output_Typedef;

//Output_Typedef CoveringDriverMotor_ENA = {CoveringDriverMotor_ENA_GPIO_Port, CoveringDriverMotor_ENA_Pin};
