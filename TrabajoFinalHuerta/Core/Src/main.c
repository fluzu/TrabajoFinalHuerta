/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bsp.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_i2cModule.h"
#include "Timer_Delay.h"
#include "DHT.h"
#include "keypad.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

static ADC_HandleTypeDef hadc1;
//DMA_HandleTypeDef hdma_adc1;

//I2C_HandleTypeDef hi2c1;

//RTC_HandleTypeDef hrtc;

static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

static DHT_DataTypeDef DHT22;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_DMA_Init(void);
//static void MX_I2C1_Init(void);
//static void MX_TIM3_Init(void);
//static void MX_ADC1_Init(void);
//static void MX_TIM2_Init(void);
//static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t value_adc[3]; // almacenar datos adc

void LCD_Temperatura(float temperatura) {
    LCD_SetCursor(1, 4);
    LCD_Print("Grados:%0.0fC", temperatura);
}

void LCD_Humedad(float humedad) {
    LCD_SetCursor(2, 1);
    LCD_Print("HA:%0.0f%%", humedad);
}

uint32_t Get_percentageHS(uint32_t value){
    int hummin = 4095;                      //REVISAR TIPO DE DATO
    int hummax = 2300;                      //REVISAR MÃ?NIMO
//#define humminp = 0
    int hummaxp = 100;
    if (value > hummin)
        value = hummin;
    if (value <= hummax)
        value = hummax + 1;
    value = value - hummax;
    return 100 - ((value * hummaxp) / (hummin - hummax));
}


//void buzzer_on(void) {                   //RECORDAR QUE ESTA MISMO TIM QUE SERVO(CAMBIAR)
//    htim2.Instance->CCR1 = 80; //duty cycle de 80
//}

//void buzzer_off(void){
//    htim2.Instance->CCR1 = 0; //duty cycle de 0
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
 // HAL_Init();
    BSP_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
 // SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
 // MX_GPIO_Init();
 // MX_DMA_Init();
//  MX_I2C1_Init();
 // MX_TIM3_Init();
 // MX_ADC1_Init();
//  MX_TIM2_Init();
 // MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  ///1)set time
  /// // sTime.Hours = 21;
//  sTime.Minutes = 56;
//  sTime.Seconds = 10;
//  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  ///2)set date
//  sDate.Date = 28;
//  sDate.Month = RTC_MONTH_OCTOBER;
//  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
//  sDate.Year = 20;
//  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);




  //TimerDelay_Init();
  HAL_Delay(200);   //REVISAR

//empezar el pwm

//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);


    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //  ENA1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET); //  ENA2

    LCD_i2cDeviceCheck();
    LCD_Init();
    LCD_BackLight(LCD_BL_ON);
    LCD_SetCursor(1,1);
    LCD_Clear();
    LCD_Print("Cargando Datos",1);
    HAL_Delay(4000);
    LCD_Clear();



    char tecla;
 //   keypad_init();

    int AMoPM = 0;
    int hora_de_riego = 0;
    int estado_cortina = 0;
    int cortina_manual = 0;        //bandera si se presiona de manera manual la cortina
    int rangohmin = 50;
    int rangohmax = 60;            //REVISAR RANGO INICIAL DE HUMEDAD

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      htim2.Instance->CCR1 = 50; //ANGULO 45 GRADOS REVISAR SI NO ROME SERVO ESTANDO EN BUCLE
///Teclado
    tecla = keypad_read();
    LCD_Clear();
    if (tecla != 0){
        switch (tecla) {
            case 65:
                LCD_Clear();
                LCD_SetCursor(1, 5);
                LCD_Print("MINIMO:", 1);
                HAL_Delay(2000);
                do {                                         //REVISAR
                    tecla = keypad_read();                  //DUDA NECESARIO PRESIONADO EN EL MOMENTO JUSTO?
                    switch (tecla) {
                        case 49: rangohmin = 10; break;
                        case 50: rangohmin = 20; break;
                        case 51: rangohmin = 30; break;
                        case 52: rangohmin = 40; break;
                        case 53: rangohmin = 50; break;
                        case 54: rangohmin = 60; break;
                        case 55: rangohmin = 70; break;
                        case 56: rangohmin = 80; break;
                        case 57: rangohmin = 90; break;
                        case 48: rangohmin =  0; break;
                        default: rangohmin = 100;              //FALTA CASO 100
                    }
                } while (tecla == 0 || rangohmin == 100);             //VER MAS CASOS // oscioso dos veces 100 porciento
                LCD_SetCursor(1, 5);
                LCD_Print("MINIMO:%0.0f", rangohmin);
                LCD_SetCursor(2, 5);
                LCD_Print("MAXIMO:", 1);
                do {
                    tecla = keypad_read();                  //DUDA NECESARIO PRESIONADO EN EL MOMENTO JUSTO?
                    switch (tecla) {                         //REVISAR
                        case 49: rangohmax = 10; break;
                        case 50: rangohmax = 20; break;
                        case 51: rangohmax = 30; break;
                        case 52: rangohmax = 40; break;
                        case 53: rangohmax = 50; break;         //FALTA DEFAULT??
                        case 54: rangohmax = 60; break;
                        case 55: rangohmax = 70; break;
                        case 56: rangohmax = 80; break;
                        case 57: rangohmax = 90; break;
                        case 48: rangohmax =  0; break;
                        default: rangohmax = 100;               //FALTA CASO 100
                    }                                           //FALTA CASO ERROR QUE SEA MENOR AL MÍNIMO
                } while (tecla == 0 || rangohmax <= rangohmin);   //REVISAR NO HACE EFECTO
                 LCD_SetCursor(2, 5);
                 LCD_Print("MAXIMO:%0.0f", rangohmax);
                 HAL_Delay(4000);
                break;
            case 49:
                LCD_SetCursor(2, 1);
                LCD_Print("Ingreso 1", 1);
                HAL_Delay(2000);
                break;
            case 50:
                LCD_SetCursor(2, 1);
                LCD_Print("Ingreso 2", 1);
                HAL_Delay(2000);
                break;
            case 51:
                LCD_SetCursor(2, 1);
                LCD_Print("Ingreso 3", 1);
                HAL_Delay(2000);
                break;
            case 52:
                LCD_SetCursor(2, 1);
                LCD_Print("Ingreso 4", 1);
                HAL_Delay(2000);
                break;
            case 53:
                LCD_SetCursor(2, 1);
                LCD_Print("Ingreso 5", 1);
                HAL_Delay(2000);
                break;
            case 54:
                LCD_SetCursor(2, 1);
                LCD_Print("Ingreso 6", 1);
                HAL_Delay(2000);
                break;
            case 66:                                //TECLA 'B'
//                LCD_SetCursor(1, 1);
//                LCD_Send_String("Ingrese 1 por AM/2 por PM", STR_NOSLIDE);
//                do {
//                    tecla = keypad_read();
//                    switch (tecla) {
//                        case 0:             break;   //buen metodo?
//                        case 49: AMoPM = 1; break;
//                        case 50: AMoPM = 2; break;
//                        default:
//                            LCD_Clear();
//                            LCD_SetCursor(1, 1);
//                           HAL_Delay(3000);
//                            LCD_Clear();
//                            LCD_SetCursor(1, 1);
//                            LCD_Send_String("Ingrese 1 por AM/2 por PM", STR_NOSLIDE);
//                    }
//                } while (AMoPM != 1 && AMoPM != 2);
//                if (AMoPM == 1){
//                    LCD_SetCursor(1, 1);
//                    LCD_Send_String("Ingrese hora de riego", STR_NOSLIDE);
//                    do {
//                        tecla = keypad_read();
//                        switch (tecla) {
//                            case 49:
//                                                                  //PROBLEMA
//                                hora_de_riego = 1;
//                                break;
//                            case 50: hora_de_riego = 2; break;
//                            case 51: hora_de_riego = 3; break;
//                            case 52: hora_de_riego = 4; break;
//                            case 53: hora_de_riego = 5; break;
//                            case 54: hora_de_riego = 6; break;
//                            case 55: hora_de_riego = 7; break;
//                            case 56: hora_de_riego = 8; break;
//                            case 57: hora_de_riego = 9; break;
                            //case 48: hora_de_riego =  0; break;
//                        }
//                    } while (tecla == 0);
//                }
//                if (AMoPM == 2){

//                }
                break;
            case 55:
                LCD_SetCursor(2, 1);
                LCD_Print("Ingreso 7", 1);
                HAL_Delay(2000);
                break;
            case 56:
                LCD_SetCursor(2, 1);
                LCD_Print("Ingreso 8", 1);
                HAL_Delay(2000);
                break;
            case 57:
                LCD_SetCursor(2, 1);
                LCD_Print("Ingreso 9", 1);
                HAL_Delay(2000);
                break;
            case 67:                                             //TECLA 'C'
                LCD_SetCursor(2, 5);
                LCD_Print("PESTICIDA", 1);
                htim2.Instance->CCR1 = 75; //ANGULO 90 GRADOS
                HAL_Delay(4000);
                break;
            case 68:                                             //TECLA 'D'
                if(estado_cortina == 0) {       //flag para ver si la cortina esta abierta o cerrada
                    LCD_SetCursor(2, 1);
                    LCD_Print("CERRANDO CORTINA", 1);
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); //  ENA
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); //  IN1
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); //  IN2
                    while (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5));   //espera hasta que la cortina toque fin de carrera con pull up
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //  ENA
                    estado_cortina = 1;                                                  //cambio de estado
                    if (cortina_manual == 0)   //revisar
                    cortina_manual = 1;        //bandera para saber si se quiere de manera manual la cortina abierta
                    else
                    cortina_manual = 0;
                    }
                else {
                    LCD_SetCursor(2, 1);
                    LCD_Print("ABRIENDO CORTINA", 1);
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); //  ENA
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); //  IN1
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //  IN2
                    while (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3));   //espera hasta que la cortina toque fin de carrera con pull up
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //  ENA
                    estado_cortina = 0;                                           //cambio de estado
                    if (cortina_manual == 0)   //revisar
                        cortina_manual = 1;
                    else
                        cortina_manual = 0;    //bandera para saber si se quiere de manera manual la cortina abierta
                }
                break;
            case 48:
                LCD_SetCursor(2, 1);
                LCD_Print("Ingreso 0", 1);
                HAL_Delay(2000);
                break;
        }
        }
///DHT22
    LCD_Clear();
    DHT_GetData(&DHT22);
    LCD_Temperatura(DHT22.Temperature);
    LCD_Humedad(DHT22.Humidity);

///Sensor humedad de suelo
    HAL_ADC_Start(&hadc1);
     if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){     //incilur esta parte en el solenoide para hecr while?
         value_adc[0] = HAL_ADC_GetValue(&hadc1);
         value_adc[0] = Get_percentageHS(value_adc[0]);
         HAL_ADC_Stop(&hadc1);
     }
      LCD_SetCursor(2, 10);
      LCD_Print("HS:%0.0f%%", value_adc[0]);  //REVISAR

      HAL_Delay(2000);


///Cerrar o abrir cortina por temperatura
      if(DHT22.Temperature < 6) {           //se puede optimizar preguntando con dos condiciones?
          if (estado_cortina == 0 && cortina_manual == 0)
          {        //flag para ver si la cortina esta abierta o cerrada  REVISAR cortina manual
              LCD_Clear();
              LCD_SetCursor(2, 1);
              LCD_Print("CERRANDO CORTINA", 1);
              HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); //  ENA
              HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); //  IN1
              HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); //  IN2
              while ( !HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5));   //espera hasta que la cortina toque fin de carrera                                              //VER CUANTO TIEMPO DEMORA EN CERRAR CORTINA
              HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //  ENA
              estado_cortina = 1;                                                  //cambio de estado
          }
      }
      else{
          if (estado_cortina == 1 && cortina_manual == 0) {
              LCD_Clear();
             LCD_SetCursor(2, 1);
              LCD_Print("ABRIENDO CORTINA", 1);
             HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);       //  ENA
              HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);     //  IN1
              HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);       //  IN2
              while (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3));   //espera hasta que la cortina toque fin de carrera                                                  //VER CUANTO TIEMPO DEMORA EN ABRIR CORTINA
             HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);     //  ENA
             estado_cortina = 0;                                                    //cambio de estado
          }
      }

///Valvula solenoide riego
      LCD_Clear();
      do {                                  //ver caso si se quiere regar durante movimiento
          HAL_ADC_Start(&hadc1);
          if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){
              value_adc[0] = HAL_ADC_GetValue(&hadc1);
              value_adc[0] = Get_percentageHS(value_adc[0]);
              HAL_ADC_Stop(&hadc1);
          }
          if (value_adc[0] < rangohmax && value_adc[0] > rangohmin) {//revisar hacer con while
              LCD_SetCursor(2, 5);
              LCD_Print("REGANDO", 1);
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);   //  ENA
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);   //  IN1
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); //  IN2

          }
      }
      while (value_adc[0] <= rangohmax && value_adc[0] >= rangohmin);
           HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET); //  ENA
///Sensor movimiento
    if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_2))) {  //si el pin esta en alto
        //buzzer_on();  //suena el buzzer
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); //Encender led verde
        LCD_Clear();
        LCD_SetCursor(2, 4);
        LCD_Print("Movimiento", 1);   // Durante toda la espera con cortina cerrada?
        HAL_Delay(1000);

//        if (estado_cortina == 0 && cortina_manual == 0) {        //flag para ver si la cortina esta abierta o cerrada  REVISAR cortina manual
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); //  ENA
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); //  IN1
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); //  IN2
//            while ( !HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5));   //espera hasta que la cortina toque fin de carrera                                              //VER CUANTO TIEMPO DEMORA EN CERRAR CORTINA
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //  ENA

//            HAL_Delay(20000);                                              //20 segundos espera cerrada para volver a abrir
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);       //  ENA
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);     //  IN1
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);       //  IN2
//            while (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3));   //espera hasta que la cortina toque fin de carrera                                                  //VER CUANTO TIEMPO DEMORA EN ABRIR CORTINA
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);     //  ENA
//        }
//        while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2));   //espera hasta que el pir se apague

        //buzzer_off(); //se apaga el buffer
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET); //se apaga el led verde
    }

  }
  /* USER CODE END 3 */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
