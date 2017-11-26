/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "key_state.h"
#include "lcd_yogurteo.h"
#include "pid.h"

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
KeyState KeyEnter = KeyStateNone;
KeyState KeyTimer = KeyStateNone;

typedef enum StepControlState StepControlState;
enum StepControlState
{
    StepControlStateNone,
    StepControlStateUp,
    StepControlStateDown
};

static StepControlState stepControlState = StepControlStateNone;

#define TempSPx100 4000
#define MaxPidDuty 100
#define MinPidDuty 0
#define ONE_HOUR (60 * 60 * 1000)

static uint8_t hoursLeft = 4;
static uint8_t LcdNumber = 4;
static bool blink5ms = false;
static int32_t pidDuty = 0;
static int32_t oldPidDuty = 0;
static bool isPrintHours = false;
static uint32_t buzzerTickOff = 0;
static uint32_t heaterTickOff = 0;
static bool HeaterOn = false;
//static uint32_t segNum = 0;
//static uint32_t comNum = 0;

static PidData pid;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void DebugPrint(const char* s)
{
    //HAL_UART_Transmit(&huart3, (uint8_t*)s, strlen(s), 100);
}

/*void DebugLCD(void)
{
    char s[32];
    sprintf(s, "Com: %d; Segment: %d\r\n", comNum + 1, segNum + 1);
    DebugPrint(s);
    bool blink = !blink5ms;
    switch (comNum)
    {
        case 0:
            HAL_GPIO_WritePin(LCD_COM1_GPIO_Port, LCD_COM1_Pin, blink5ms ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(LCD_COM2_GPIO_Port, LCD_COM2_Pin, blink5ms ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(LCD_COM3_GPIO_Port, LCD_COM3_Pin, blink5ms ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
    }
    switch (segNum)
    {
        case 0:
            HAL_GPIO_WritePin(LCD_SEG1_GPIO_Port, LCD_SEG1_Pin, blink ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(LCD_SEG2_GPIO_Port, LCD_SEG2_Pin, blink ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(LCD_SEG3_GPIO_Port, LCD_SEG3_Pin, blink ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(LCD_SEG4_GPIO_Port, LCD_SEG4_Pin, blink ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case 4:
            HAL_GPIO_WritePin(LCD_SEG5_GPIO_Port, LCD_SEG5_Pin, blink ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
}*/

void OnLcdUpdate(void)
{
    PrintLcd(LcdNumber, blink5ms);
    blink5ms = !blink5ms;
}

void PWM_ConfigureChannel(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t pulse)
{
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel) != HAL_OK)
    {
        Error_Handler();
    }
}

void PWM_SetChannelPercent(TIM_HandleTypeDef* htim, uint32_t channel, int32_t percent)
{
    if (percent >100)
        percent = 100;
    else
    if (percent < 0)
        percent = 0;
    HAL_TIM_PWM_Stop(htim, channel);
    PWM_ConfigureChannel(htim, channel, (htim->Init.Period + 1) * percent / 100);
    HAL_TIM_PWM_Start(htim, channel);
}

bool LimitOutput(void)
{
    if (pidDuty > MaxPidDuty)
        pidDuty = MaxPidDuty;
    else if (pidDuty < MinPidDuty)
        pidDuty = MinPidDuty;
    bool res = pidDuty != oldPidDuty;
    oldPidDuty = pidDuty;
    return res;
}

bool StepControl(int32_t Temp)
{
    #define Hysteresis 10
    switch (stepControlState)
    {
        case StepControlStateNone:
            if (abs(Temp - TempSPx100) > Hysteresis)
                stepControlState = Temp > TempSPx100 ? StepControlStateDown : StepControlStateUp;
            break;
        case StepControlStateUp:
            if (Temp < TempSPx100 - Hysteresis)
                ++pidDuty;
            else
                stepControlState = StepControlStateNone;
            break;
        case StepControlStateDown:
            if (Temp > TempSPx100 + Hysteresis)
                --pidDuty;
            else
                stepControlState = StepControlStateNone;
            break;
    }
    return LimitOutput();
}

bool PidControl(int32_t Temp)
{
    pidDuty = PControllerExecute(TempSPx100, Temp, &pid);
    return LimitOutput();
}

void BuzzerOn(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);//BUZZER
}

void BuzzerOff(void)
{
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);//BUZZER
}

void BuzzerStartForTime(uint32_t ms)
{
    BuzzerOn();
    buzzerTickOff = HAL_GetTick() + ms;
}

void SwitchOffHeater(uint32_t buzzerMs)
{
    HeaterOn = false;
    pidDuty = 0;
    oldPidDuty = 0;
    heaterTickOff = 0;
    hoursLeft = 0;
    PidControllerReset(&pid);
    PWM_SetChannelPercent(&htim1, TIM_CHANNEL_1, pidDuty);
    DebugPrint("Regulator stops!\r\n");
    BuzzerStartForTime(buzzerMs);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t tickstart250ms = 0;
  uint32_t tickstart5ms = 0;
  uint32_t tickstart3sec = 0;
  uint32_t currenttick = 0;
  uint32_t Tmax = 0;
  uint32_t Tmin = 0xFFFF;
  uint8_t TemperatureInt = 0;  
  
  PidControllerInit(10, 30, 0, 4, &pid);
  
  tickstart250ms = HAL_GetTick();
  tickstart5ms = tickstart250ms;
  tickstart3sec = tickstart250ms;

  HAL_ADC_Start(&hadc1);
  PWM_SetChannelPercent(&htim1, TIM_CHANNEL_1, pidDuty);
  HAL_TIM_Base_Start_IT(&htim3);
  
  DebugPrint("\r\nYOGURTEO v1.0\r\n\r\n");
  
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
      currenttick = HAL_GetTick();
      if (currenttick - tickstart250ms >= 250)
      {
          HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
          tickstart250ms = currenttick;
          if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
          {
              uint32_t val = HAL_ADC_GetValue(&hadc1);
              uint32_t t = val*15000/4096;
              if (t > Tmax)
              {
                  Tmax = t;
                  Tmin = t;
              }
              else if (t < Tmin)
              {
                  Tmin = t;
              }
              TemperatureInt = t / 100;
              if (HeaterOn)
              {
                  char sOut[32];
                  //if (StepControl(t))
                  if (PidControl(t))
                  {
                      if (t < 1000 || t > 6000)
                      {
                          SwitchOffHeater(1000);
                          DebugPrint("Unreal temperature! Stopping control...\r\n");
                          continue;
                      }
                      PWM_SetChannelPercent(&htim1, TIM_CHANNEL_1, pidDuty);
                      sprintf(sOut, "%d", pidDuty);
                  }
                  else
                  {
                      strcpy(sOut, "none");
                  }
                  
                  char s[32];
                  sprintf(s, "T: %d; Tmin: %d; Tmax: %d; TempInt: %d; OUT: %s\r\n", t, Tmin, Tmax, TemperatureInt, sOut);
                  DebugPrint(s);

                  if (heaterTickOff != 0)
                  {
                      if (currenttick >= heaterTickOff)
                      {
                          DebugPrint("Time for heating ends. Yogurt is ready!\r\n");
                          SwitchOffHeater(1000);
                      }
                      else
                      {
                          uint32_t delta = heaterTickOff - currenttick;
                          hoursLeft = delta / ONE_HOUR;
                          if (delta % ONE_HOUR != 0)
                              ++hoursLeft;
                      }
                  }
              }//if HeaterOn
          }
      }
      if (currenttick - tickstart3sec >= 3000)
      {
          isPrintHours = !HeaterOn || !isPrintHours;
          if (isPrintHours)
              LcdNumber = hoursLeft;
          else
              LcdNumber = TemperatureInt;
          tickstart3sec = currenttick;
      }
      if (currenttick - tickstart5ms >= 4)
      {
          tickstart5ms = currenttick;
          if (buzzerTickOff != 0 && currenttick >= buzzerTickOff)
          {
              buzzerTickOff = 0;
              BuzzerOff();
          }
      }
      switch (KeyEnter)
      {
          case KeyStateOn:
              if (!HeaterOn)
              {
                  HeaterOn = true;
                  Tmax = 0;
                  Tmin = 0xFFFF;
                  stepControlState = StepControlStateNone;
                  pidDuty = MaxPidDuty;
                  heaterTickOff = currenttick + hoursLeft * ONE_HOUR;
                  DebugPrint("Starting regulator...\r\n");
                  BuzzerStartForTime(500);
              }
              else
              {
                  SwitchOffHeater(100);
              }
              KeyEnter = KeyStateNone;
              break;
          case KeyStateOff:
              KeyEnter = KeyStateNone;
              break;
          case KeyStateNone:
              break;
      }
      switch (KeyTimer)
      {
          case KeyStateOn:
              BuzzerStartForTime(100);
              if (!HeaterOn)
              {
                  ++hoursLeft;
                  if (hoursLeft > 9)
                      hoursLeft = 0;
                  LcdNumber = hoursLeft;
              }
              /*++segNum;
              if (segNum > 4)
              {
                  segNum = 0;
                  ++comNum;
                  if (comNum > 2)
                      comNum = 0;
              }*/
              KeyTimer = KeyStateNone;
              break;
          case KeyStateOff:
              KeyTimer = KeyStateNone;
              break;
          case KeyStateNone:
              break;
      }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  DebugPrint("Error_Handler()!!!\r\n");
  SwitchOffHeater(3000);
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
