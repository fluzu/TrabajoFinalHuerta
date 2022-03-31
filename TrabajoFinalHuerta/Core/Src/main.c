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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

DHT_DataTypeDef DHT22;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t value_adc[3]; // almacenar datos adc

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
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
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




  TimerDelay_Init();
  HAL_Delay(200);

//empezar el pwm

HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);


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
    keypad_init();

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
      htim2.Instance->CCR1 = 50; //ANGULO 45 GRADOS
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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 23;
  sTime.Minutes = 59;
  sTime.Seconds = 45;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_OCTOBER;
  sDate.Date = 28;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE10 PE12 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin PD2 PD3 PD4 */
  GPIO_InitStruct.Pin = LD4_Pin|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
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
