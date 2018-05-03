/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "ds_DataTransmissionLayer.h"
#include "ds_ProtocolLayer.h"
#include "Common.h"
#include "ds_gentlesensor.h"
#include "ds_log.h"
#include "ds_FillLight.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/*添加版本号，方便处理程序和后期的维修*/
#define		CODEVERSION				0x0201			//版本号
#define     TEMPUTERALARMVALUE		0x04FF			//温度报警阈值
#define     LIGHTHELPVALUE			0x04FF			//常亮补光灯阈值
uint16_t gLogReportCnt;
uint8_t  gLogReportFlag;
uint8_t  gLEDsCarFlag;
uint8_t  gCarComingFlag;

uint8_t		gPWMLedFlag;
uint8_t     gTIM5PWMFlag;
uint8_t     gTIM5PWMCnt;
uint16_t    gPWMValue;

uint32_t  gADCBuffer[10];
uint32_t  gTempterValue;
uint32_t  gLightValue;
uint32_t  gADCTimCnt;
uint8_t   gADCTimCntFlag;
/*氛围灯设置*/
uint16_t  gLEDCnt;
uint8_t  gLEDFlag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void CheckLightAndTemperture(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
extern USARTRECIVETYPE     CoreBoardUsartType;
GPIOSTATUSDETECTION gGentleSensorStatusDetection;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();//0.1ms
  MX_TIM5_Init();//1ms
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();//4ms

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&gADCBuffer, 10);
	
	DS_LED_Init();
	DS_GentleSensorInit();
	DS_CoreBoardProtocolInit();
	DS_LeftDoorBoardProtocolInit();
	DS_RightDoorBoardProtocolInit();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  
	  /*处理来自核心板的数据*/
	DS_HandingUartDataFromCoreBoard();
	DS_HandingCoreBoardRequest();
	
	/*处理来自闸机A的数据*/
	DS_HandingUartDataFromLeftDoorBoard();
	DS_HandingLeftDoorBoardRequest();
	  
	  /*处理来自道闸B的数据*/
	DS_HandingUartDataFromRightDoorBoard();
	DS_HandingRightDoorBoardRequest(); 
	  
	DS_SendAckData();
	  
	  /* 检测地感信号 */
	  DS_GentleSensorCheck();
	  
	/*检测温度和光照*/
	  if (gADCTimCntFlag)
	  {
		  gADCTimCntFlag = 0;
		  CheckLightAndTemperture();
	  }
	  
	/*PWM补光灯控制*/
	  if (gCarComingFlag)
	{
		if (gPWMValue > 3800)
		{
		  gPWMValue = 3900;
		}
		
		DS_SetLedPwmValue(gPWMValue);
	}
	else
	{
		DS_SetLedPwmValue(0);
	}
	  
	  
	 /*上报时间到的时候，进行日志上报*/
	if (gLogReportFlag)
	{
		DS_ReportLogInfo(); 
		gLogReportFlag = 0;
	 }
	  
	 /* 氛围灯控制 */
	if (gLEDFlag)
	{
	  gLEDFlag = 0;
	  DS_LEDS_TEST();
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* TIM5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(htim);
	/* NOTE : This function Should not be modified, when the callback is needed,
	          the __HAL_TIM_PeriodElapsedCallback could be implemented in the user file
	       */
	/* 4ms */
	if (htim3.Instance == htim->Instance)
	{
		
	}
	
	/* 0.1ms */
	if (htim4.Instance == htim->Instance)
	{
		/* 添加地感处理 */
		gGentleSensorStatusDetection.GpioCurrentReadVal = HAL_GPIO_ReadPin(GentleSensor_GPIO_Port, GentleSensor_Pin);
		if (0 == gGentleSensorStatusDetection.GpioCurrentReadVal && 0 == gGentleSensorStatusDetection.GpioLastReadVal)
		{
			if (0 == gGentleSensorStatusDetection.GpioCheckedFlag)
			{
				gGentleSensorStatusDetection.GpioFilterCnt++;
				if (gGentleSensorStatusDetection.GpioFilterCnt > gGentleSensorStatusDetection.GpioFilterCntSum)
				{
					gGentleSensorStatusDetection.GpioStatusVal		= 1;
					gGentleSensorStatusDetection.GpioFilterCnt		= 0;
					gGentleSensorStatusDetection.GpioCheckedFlag	= 1;
					gCarComingFlag = 1;
					DS_UpgentleStatusInfoLog(1);
					gPWMLedFlag = 1;
				}
			}
		}
		else
		{
			gGentleSensorStatusDetection.GpioCheckedFlag	= 0;
			gGentleSensorStatusDetection.GpioFilterCnt		= 0;
			gGentleSensorStatusDetection.GpioStatusVal		= 0;
			gGentleSensorStatusDetection.GpioSendDataFlag	= 1;
			gCarComingFlag = 0;
			DS_UpgentleStatusInfoLog(0);
			gPWMLedFlag = 0;
		}
		gGentleSensorStatusDetection.GpioLastReadVal = gGentleSensorStatusDetection.GpioCurrentReadVal;
		
		
	}
	
	/* 1ms */
	if (htim5.Instance == htim->Instance)
	{
		gADCTimCnt++;
		if (gADCTimCnt > 60000)
		{
			gADCTimCntFlag = 1;
			gADCTimCnt = 0;
		}
		if (gGentleSensorStatusDetection.GpioValidLogicTimeCnt > 80)
		{
			gGentleSensorStatusDetection.GpioValidLogicTimeCnt--;
		}
		
		/*上报日志 6s 一次*/
		gLogReportCnt++;
		if (gLogReportCnt > 5000)
		{
			gLogReportFlag = 1;
			gLogReportCnt = 0;
		}
		
		/*来车时每秒钟闪烁五次，无车时保持常亮*/
		gLEDCnt++;
		if (gLEDCnt > 200)
		{
			gLEDFlag = 1;
			gLEDCnt = 0;
		}
		
		/*PWM控制 */
		gTIM5PWMCnt++;
		if (gTIM5PWMCnt > 3)
		{
			if (gCarComingFlag)
			{
				gPWMValue += 2;
				gTIM5PWMFlag = 1;
			}
			else
			{
				gPWMValue = 1800;
				gTIM5PWMFlag = 0;
          
			}
			gTIM5PWMCnt = 0;
		}
	}	
}

void CheckLightAndTemperture(void)
{
	uint8_t i;
	for (i = 0; i < 10;)
	{
		gLightValue += gADCBuffer[i++]; 
		gTempterValue += gADCBuffer[i++];
	}
	gLightValue /= 5;
	gTempterValue /= 5;
	DS_UpLightInfoLog(gLightValue);
	DS_UpTemInfoLog(gTempterValue);
		  
	if (gLightValue < LIGHTHELPVALUE)
	{
		DS_UpFlashLightInfoLog(1);
		/*此处添加打开灯的控制*/
	}
	else
	{
		DS_UpFlashLightInfoLog(0);
	}
		  
	if (gTempterValue > TEMPUTERALARMVALUE)
	{
		/* 温度过高开启散热 */
		DS_FAN_OUT_ON();
		DS_UpFanStatusInfoLog(1);
	}
	else
	{
		/*温度正常，关闭散热*/
		DS_FAN_OUT_OFF();
		DS_UpFanStatusInfoLog(0);
	} 
	return ;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
