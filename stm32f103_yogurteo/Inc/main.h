/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LED_BLUE_Pin GPIO_PIN_13
#define LED_BLUE_GPIO_Port GPIOC
#define TEMP_IN_Pin GPIO_PIN_0
#define TEMP_IN_GPIO_Port GPIOA
#define KEY_TIMER_Pin GPIO_PIN_1
#define KEY_TIMER_GPIO_Port GPIOA
#define LCD_SEG4_Pin GPIO_PIN_2
#define LCD_SEG4_GPIO_Port GPIOA
#define LCD_SEG5_Pin GPIO_PIN_3
#define LCD_SEG5_GPIO_Port GPIOA
#define LCD_SEG3_Pin GPIO_PIN_4
#define LCD_SEG3_GPIO_Port GPIOA
#define LCD_SEG2_Pin GPIO_PIN_5
#define LCD_SEG2_GPIO_Port GPIOA
#define LCD_SEG1_Pin GPIO_PIN_6
#define LCD_SEG1_GPIO_Port GPIOA
#define LCD_COM3_Pin GPIO_PIN_7
#define LCD_COM3_GPIO_Port GPIOA
#define LCD_COM2_Pin GPIO_PIN_0
#define LCD_COM2_GPIO_Port GPIOB
#define LCD_COM1_Pin GPIO_PIN_1
#define LCD_COM1_GPIO_Port GPIOB
#define HEATER_OUT_Pin GPIO_PIN_8
#define HEATER_OUT_GPIO_Port GPIOA
#define KEY_ENTER_Pin GPIO_PIN_7
#define KEY_ENTER_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void OnLcdUpdate(void);

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
