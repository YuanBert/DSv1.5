/**
  ******************************************************************************
  * File Name          : ds_led.h
  * Description        : this file is greation the LED control codes
  *
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#ifndef __ds_led_H
#define __ds_led_H
#ifdef __cplusplus
 extern "C" {
#endif
   /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
   
#define MCU_LED_OFF                     GPIO_PIN_SET
#define MCU_LED_ON                     GPIO_PIN_RESET
#define MCU_FAN_ON                      GPIO_PIN_SET
#define MCU_FAN_OFF                     GPIO_PIN_RESET
#define RunningLED_OFF                   GPIO_PIN_SET
#define RunningLED_ON                  GPIO_PIN_RESET
#define CommunicationLED_OFF             GPIO_PIN_SET
#define CommunicationLED_ON            GPIO_PIN_RESET
#define MCUAtmosphereLED1_ON            GPIO_PIN_SET
#define MCUAtmosphereLED1_OFF           GPIO_PIN_RESET
#define MCUAtmosphereLED2_ON            GPIO_PIN_SET
#define MCUAtmosphereLED2_OFF           GPIO_PIN_RESET
   
   
   
/* USER CODE BEGIN Private defines */
   void DS_LED_Init(void);
   void DS_RUNNINGLED_ON(void);
   void DS_RUNNINGLED_OFF(void);
   void DS_RUNNINGLED_TORGGLE(void);
   
   void DS_COMMUNICATIONLED_ON(void);
   void DS_COMMUNICATIONLED_OFF(void);
   void DS_COMMUNICATIONLED_TORGGLE(void);
   
   void DS_ATMOSPHERELED1_ON(void);
   void DS_ATMOSPHERELED1_OFF(void);
   void DS_ATMOSPHERELED1_TOGGLE(void);
   
   void DS_ATMOSPHERELED2_ON(void);
   void DS_ATMOSPHERELED2_OFF(void);
   void DS_ATMOSPHERELED2_TOGGLE(void);
   
 
   void DS_FAN_OUT_OFF(void);
   void DS_FAN_OUT_ON(void);   
   
   void DS_LEDS_TEST(void);

/* USER CODE END Private defines */
   
/* USER CODE BEGIN Prototypes */
/* USER CODE END Prototypes */
#ifdef __cplusplus
}
#endif
#endif /*__ds_led_H */