/**
  ******************************************************************************
  * File Name          : ds_gentlesensor.c
  * Description        : 
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  /* Includes ------------------------------------------------------------------*/
#include "ds_gentlesensor.h"
#include "ds_DataTransmissionLayer.h"
extern GPIOSTATUSDETECTION gGentleSensorStatusDetection;
extern uint8_t gLEDsCarFlag;
extern uint8_t gCarComingFlag;
static uint8_t gSendFlag = 0;;
/*******************************************************************************
*
*       Function        :DS_GentleSensorInit()
*
*       Input           :void
*
*       Return          :DS_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/26
*       Author          :bertz
*******************************************************************************/
DS_StatusTypeDef DS_GentleSensorInit(void)
{
  DS_StatusTypeDef state = DS_OK;
  gGentleSensorStatusDetection.GpioFilterCntSum = 20;
  gGentleSensorStatusDetection.GpioValidLogicTimeCnt = 0;
  return state; 
}

/*******************************************************************************
*
*       Function        :DS_GentleSensorCheck()
*
*       Input           :void
*
*       Return          :DS_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/26
*       Author          :bertz
*******************************************************************************/
DS_StatusTypeDef DS_GentleSensorCheck(void)
{
  DS_StatusTypeDef state = DS_OK;
  uint8_t data[8];

  if(0 == gGentleSensorStatusDetection.GpioCheckedFlag || gGentleSensorStatusDetection.GpioValidLogicTimeCnt < 100)
  {
    /* Turn off the flash if the car leaves or if the flash blinks longer than the set value */
    gLEDsCarFlag = 0;
	gCarComingFlag = 0;
	gSendFlag = 1;   
  }
	if (gGentleSensorStatusDetection.GpioStatusVal && gGentleSensorStatusDetection.GpioSendDataFlag)
	{
		gGentleSensorStatusDetection.GpioValidLogicTimeCnt = 90100;
		/* Flash open*/
		gCarComingFlag = 1;
		gLEDsCarFlag = 1;
		gSendFlag = 0;
		/* Report the arrival of car */
		//    gCoreBoardProtocolCmd.CmdType    = 0xB1;
		//    gCoreBoardProtocolCmd.CmdParam   = 0x01;
		//    gCoreBoardProtocolCmd.DataLength = 0x0000;
		//    gCoreBoardProtocolCmd.DataLengthLow    = 0x00;
		//    gCoreBoardProtocolCmd.DataLengthHight  = 0x00;
		//    HAL_Delay(100);
		//    state = DS_SendRequestCmdToCoreBoard(&gCoreBoardProtocolCmd);
		gGentleSensorStatusDetection.GpioSendDataFlag = 0;
	}
	
	if (0 == gGentleSensorStatusDetection.GpioSendDataFlag && gGentleSensorStatusDetection.GpioCheckedFlag)
	{
		/* if the vehcile is still ,carry out the release operation */
    
    
		if (0 == gSendFlag && gGentleSensorStatusDetection.GpioValidLogicTimeCnt < 89500)
		{
      
			data[0] = 0x5B;
			data[1] = 0xB1;
			data[2] = 0x01;
			data[3] = 0x00;
			data[4] = 0x00;
			data[5] = 0xB0;
			data[6] = 0x5D;
			DS_SendDataToCoreBoard(data, 7, 0xFFFF); 
	    
			/* Report vehicle timeout notification */
			gSendFlag = 1;
	    
		}
		
	}  
  return state;  
}

   
  /**
  * @}
  */
  /**
  * @}
  */
  /*****************************END OF FILE**************************************/
