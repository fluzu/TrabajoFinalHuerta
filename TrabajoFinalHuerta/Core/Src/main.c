#include "main.h"
#include "bsp.h"
#include "lcd_i2cModule.h"
#include "DHT.h"

void *DriverMotor_ENA;
void *DriverMotor_IN1;
void *DriverMotor_IN2;

void *DriverValve_ENA;
void *DriverValve_IN1;
void *DriverValve_IN2;


extern DHT_DataTypeDef DHT22;


//void buzzer_on(void) {                   //RECORDAR QUE ESTA MISMO TIM QUE SERVO(CAMBIAR)
//    htim2.Instance->CCR1 = 80; //duty cycle de 80
//}

//void buzzer_off(void){
//    htim2.Instance->CCR1 = 0; //duty cycle de 0
//}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){

    BSP_Init();
    APP_Show_SystemIntro();
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

    //int AMoPM = 0;
    //int hora_de_riego = 0;
    int estado_cortina = 0;
    int cortina_manual = 0;        //bandera si se presiona de manera manual la cortina
    int rangohmin = 50;
    int rangohmax = 60;            //REVISAR RANGO INICIAL DE HUMEDAD

  while (1)
  {
      //htim2.Instance->CCR1 = 50; //ANGULO 45 GRADOS REVISAR SI NO ROME SERVO ESTANDO EN BUCLE
///Teclado
      APP_Keypad(rangohmin, rangohmax, estado_cortina, cortina_manual);
///DHT22
      APP_Show_DHT22();
///Sensor humedad de suelo
      APP_Show_SoilHumidity();
      ///Sensor movimiento
      APP_Show_Movement();
///Cerrar o abrir cortina por temperatura
      APP_CoverFromTemperature(estado_cortina, cortina_manual);
///Valvula solenoide riego
      APP_Irrigation(rangohmin, rangohmax);
  }

}

void APP_Timer10ms(){

}
void APP_Timer100ms(){

}
void APP_Timer1000ms(){

}
void APP_Timer10s(){

}
void APP_Show_SystemIntro(){
    LCD_i2cDeviceCheck();
    LCD_BackLight(LCD_BL_ON);
    LCD_SetCursor(1,1);
    LCD_Clear();
    LCD_Print("Cargando Datos",1);
    BSP_Delay(4000);
    LCD_Clear();
}

void APP_Keypad(int rangohmin, int rangohmax, int estado_cortina, int cortina_manual){
    BSP_Keypad(rangohmin, rangohmax, estado_cortina, cortina_manual);
}

void APP_Show_DHT22(){
    LCD_Clear();  //REVISAR necesidad de esta funcion
    DHT_GetData(&DHT22);
    BSP_LCD_Temperature(DHT22.Temperature);
    BSP_LCD_Humidity(DHT22.Humidity);
}

void APP_Show_Movement(){
    BSP_Detect_Movement();
}

void APP_CoverFromTemperature(int estado_cortina, int cortina_manual){
    BSP_CoverFromTemperature(estado_cortina, cortina_manual);
}

void APP_Show_SoilHumidity(){
    BSP_Show_SoilHumidity();
}

void APP_Irrigation(int rangohmin, int rangohmax){
    BSP_Irrigation(rangohmin, rangohmax);
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

