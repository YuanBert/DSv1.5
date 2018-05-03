/**
******************************************************************************
  * File Name          : ds_log.c
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
#include "ds_log.h"
static uint8_t ReportFlag;
static uint8_t DriverBoardInfo[10];

//static uint8_t getXORCode(uint8_t* pData,uint16_t len)
//{
//  uint8_t ret;
//  uint16_t i;
//  ret = pData[0];
//  for(i = 1; i < len; i++)
//  {
//    ret ^=pData[i];
//  }
//  return ret;
//}

/*温度信息*/
void DS_UpTemInfoLog(uint16_t tTemp)
{
    DriverBoardInfo[0] = (uint8_t)((tTemp >> 8) & 0x0F);
    DriverBoardInfo[1] = (uint8_t)tTemp;
}

/*光照度信息*/
void DS_UpLightInfoLog(uint16_t tTemp)
{
    DriverBoardInfo[2] = (uint8_t)((tTemp >> 8) & 0x0F);
    DriverBoardInfo[3] = (uint8_t)tTemp;
}

/*更新风扇信息*/
void DS_UpFanStatusInfoLog(uint8_t tStatus)
{
    DriverBoardInfo[4] = tStatus;
}

/* 补光灯信息 */
void DS_UpFlashLightInfoLog(uint8_t tStatus)
{
   DriverBoardInfo[5] = tStatus;
}

/* 更新地感状态信息 */
void DS_UpgentleStatusInfoLog(uint8_t tStatus)
{
  DriverBoardInfo[6] = tStatus;
}

/*更新氛围灯的状态信息*/
void DS_UpAtmosphereStatusInfoLog(uint8_t tStatus)
{
  DriverBoardInfo[7] = tStatus;
}

/* 获取道闸A的状态信息 */
void DS_UpDoorAStatus(uint8_t tStatus)
{
  DriverBoardInfo[8] = tStatus;
}

/*获取道闸B的状态信息*/
void DS_UpDoorBStatus(uint8_t tStatus)
{
  DriverBoardInfo[9] = tStatus;
}

/* 写日志上传标记位 */
DS_StatusTypeDef DS_WriteReportLogFlag(void)
{
  DS_StatusTypeDef status = DS_OK;
  ReportFlag = 1;
  return status;
}

/* 上传日志 */
DS_StatusTypeDef DS_ReportLogInfo()
{
  DS_StatusTypeDef status = DS_OK;
  uint8_t i;
  uint8_t temp[17];
  temp[0] = 0x5B;
  temp[1] = 0xD1;
  temp[2] = 0x01;
  temp[3] = 0x00;
  temp[4] = 0x0A;
  temp[16] = 0x5D;
  
  for(i = 0; i < 10; i ++)
  {
    temp[5 + i] = DriverBoardInfo[i];
  }
  
  temp[15] = temp[1];
  for(i = 0; i < 13; i++)
  {
    temp[15] ^=temp[2 + i];
  }
  
  status = (DS_StatusTypeDef)HAL_UART_Transmit(&huart1, temp, 17,0xFFFF);
  
  return status;
}

DS_StatusTypeDef DS_CheckReportLog(void)
{
  DS_StatusTypeDef status = DS_OK;
  
  if(ReportFlag)
  {
   status = DS_ReportLogInfo();
  }
  
  ReportFlag = 0;
  return status;
}
   
  /**
  * @}
  */
  /**
  * @}
  */
  /*****************************END OF FILE**************************************/
