/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CommunicationLED_Pin GPIO_PIN_13
#define CommunicationLED_GPIO_Port GPIOC
#define CTR485B_EN_Pin GPIO_PIN_0
#define CTR485B_EN_GPIO_Port GPIOC
#define CTR485A_EN_Pin GPIO_PIN_1
#define CTR485A_EN_GPIO_Port GPIOC
#define LightSensor_Pin GPIO_PIN_2
#define LightSensor_GPIO_Port GPIOC
#define BSPA_TX_Pin GPIO_PIN_2
#define BSPA_TX_GPIO_Port GPIOA
#define BSPA_RX_Pin GPIO_PIN_3
#define BSPA_RX_GPIO_Port GPIOA
#define W25Q64_SCK_Pin GPIO_PIN_5
#define W25Q64_SCK_GPIO_Port GPIOA
#define W25Q64_MISO_Pin GPIO_PIN_6
#define W25Q64_MISO_GPIO_Port GPIOA
#define W25Q64_MOSI_Pin GPIO_PIN_7
#define W25Q64_MOSI_GPIO_Port GPIOA
#define GentleSensor_Pin GPIO_PIN_4
#define GentleSensor_GPIO_Port GPIOC
#define BSPB_TX_Pin GPIO_PIN_10
#define BSPB_TX_GPIO_Port GPIOB
#define BSPB_RX_Pin GPIO_PIN_11
#define BSPB_RX_GPIO_Port GPIOB
#define MCUAtmosphereLEDR_Pin GPIO_PIN_12
#define MCUAtmosphereLEDR_GPIO_Port GPIOB
#define MCUAtmosphereLEDG_Pin GPIO_PIN_13
#define MCUAtmosphereLEDG_GPIO_Port GPIOB
#define MCU_DS18B20_Pin GPIO_PIN_6
#define MCU_DS18B20_GPIO_Port GPIOC
#define CoreBoard_TX_Pin GPIO_PIN_9
#define CoreBoard_TX_GPIO_Port GPIOA
#define CoreBoard_RX_Pin GPIO_PIN_10
#define CoreBoard_RX_GPIO_Port GPIOA
#define MCU_LED_PWM_Pin GPIO_PIN_4
#define MCU_LED_PWM_GPIO_Port GPIOB
#define MCU_FAN_OUT_Pin GPIO_PIN_5
#define MCU_FAN_OUT_GPIO_Port GPIOB
#define MCU_LED_OUT_Pin GPIO_PIN_6
#define MCU_LED_OUT_GPIO_Port GPIOB
#define RunningLED_Pin GPIO_PIN_9
#define RunningLED_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
