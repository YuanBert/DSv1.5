/**
******************************************************************************
* File Name          : ds_ProtocolLayer.c
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
#include "ds_ProtocolLayer.h"

extern USARTRECIVETYPE     CoreBoardUsartType;
extern USARTRECIVETYPE     LeftDoorBoardUsartType;
extern USARTRECIVETYPE     RightDoorBoardUsartType;

AckedStruct    CoreBoardAckedData;
AckedStruct    LeftDoorBoardAckedData;
AckedStruct    RightDoorBoardAckedData;

RevDataStruct   CoreBoardRevDataStruct;
RevDataStruct   LeftDoorBoardRevDataStruct;
RevDataStruct   RightDoorBoardRevDataStruct;

SendDataStrct   CoreBoardSendDataStruct;
SendDataStrct   LeftDoorBoardSendDataStruct;
SendDataStrct   RightDoorBoardSendDataStruct;



static uint8_t CoreRevDataBuf[DATABUFLEN];
static uint8_t CoreSenddataBuf[DATABUFLEN];    

static uint8_t LeftDoorRevDataBuf[DATABUFLEN];
static uint8_t LeftDoorSendDataBuf[DATABUFLEN];

static uint8_t RightDoorRevDataBuf[DATABUFLEN];
static uint8_t RightDoorSendDataBuf[DATABUFLEN];


static uint8_t getXORCode(uint8_t* pData,uint16_t len)
{
  uint8_t ret;
  uint16_t i;
  ret = pData[0];
  for(i = 1; i < len; i++)
  {
    ret ^=pData[i];
  }
  return ret;
}

static DS_StatusTypeDef DS_HandingUartData(pRevDataStruct pRevDataStruct,pUSARTRECIVETYPE pUsartType ,uint8_t* pRevDataBuf)
{
  DS_StatusTypeDef state = DS_OK;
  uint8_t xorTemp;
  uint16_t i;
  if(!CoreBoardUsartType.RX_Flag)
  {
    return state;
  }
  CoreBoardUsartType.RX_Flag = 0;
  
  /*  Handling the ACK Cmd */
  if(0xA0 == *(CoreBoardUsartType.RX_pData + 1)&0xF0)
  {
      if(CoreBoardAckedData.AckCnt > 5)
      {
        return state;
      }
      if(0x5B != *(CoreBoardUsartType.RX_pData) || 0x5D != *(CoreBoardUsartType.RX_pData + ACKFIXEDCOMMANDLEN -1))
      {
          return state;
      }
      xorTemp = getXORCode(CoreBoardUsartType.RX_pData + 1,3);
      if(xorTemp != *(CoreBoardUsartType.RX_pData + 4))
      {
        return state;
      }
      
      CoreBoardAckedData.AckCmdCode[CoreBoardAckedData.AckCnt]     = *(CoreBoardUsartType.RX_pData + 1);
      CoreBoardAckedData.AckCodeH[CoreBoardAckedData.AckCnt]       = *(CoreBoardUsartType.RX_pData + 2);
      CoreBoardAckedData.AckCodeL[CoreBoardAckedData.AckCnt]       = *(CoreBoardUsartType.RX_pData + 3);
      CoreBoardAckedData.CheckedAckFlag[CoreBoardAckedData.AckCnt] = 1;
      
      CoreBoardAckedData.AckCnt++;
      
      return state;
  }
  /* 在ACK命令处理时需要对ACkCnt进行间操作，每进行一次操作进行一次减操作 */
  
  /* Handling Request Cmd Data */
  if(CoreBoardRevDataStruct.RevOKFlag)
  {
    return state;
  }
  
  if(CoreBoardRevDataStruct.NumberOfBytesReceived < CoreBoardRevDataStruct.DataLength && 0 != CoreBoardRevDataStruct.NumberOfBytesReceived)
  {
    for(i = 0; i < CoreBoardUsartType.RX_Size; i++)
    {
      CoreRevDataBuf[5 + CoreBoardRevDataStruct.NumberOfBytesReceived] = *(CoreBoardUsartType.RX_pData + i);
      CoreBoardRevDataStruct.NumberOfBytesReceived ++;
      if(CoreBoardRevDataStruct.DataLength == CoreBoardRevDataStruct.NumberOfBytesReceived)
      {
        CoreBoardRevDataStruct.XOR8BIT = *(CoreBoardUsartType.RX_pData + i + 1);
        if(0x5D != *(CoreBoardUsartType.RX_pData + i + 2))
        {
          CoreBoardRevDataStruct.NumberOfBytesReceived = 0;
          CoreBoardRevDataStruct.DataLength = 0;
          CoreBoardRevDataStruct.TotalLength = 0;
          return state;
        }
        CoreBoardRevDataStruct.TotalLength = CoreBoardRevDataStruct.DataLength + REQUESTFIXEDCOMMANDLEN;
        /* here to check XOR code */
        xorTemp = getXORCode(CoreRevDataBuf + 1, CoreBoardRevDataStruct.TotalLength - 3);
        if(CoreBoardRevDataStruct.XOR8BIT != xorTemp)
        {
          CoreBoardRevDataStruct.NumberOfBytesReceived = 0;
          CoreBoardRevDataStruct.DataLength = 0;
          CoreBoardRevDataStruct.TotalLength = 0;      
          return state;
        }
        CoreRevDataBuf[5 + CoreBoardRevDataStruct.NumberOfBytesReceived] = xorTemp;
        CoreRevDataBuf[5 + CoreBoardRevDataStruct.NumberOfBytesReceived + 1] = 0x5D;
        CoreBoardRevDataStruct.pRevDataBuf = CoreRevDataBuf;
        CoreBoardRevDataStruct.RevOKFlag = 1;
      }
    }
    return state;
  }
  
  if(CoreBoardRevDataStruct.TotalLength)
  {
    if(0x5B != *(CoreBoardUsartType.RX_pData))
    {
      return state;
    }
    CoreBoardRevDataStruct.CmdType      =*(CoreBoardUsartType.RX_pData + 1);
    CoreBoardRevDataStruct.CmdParam     =*(CoreBoardUsartType.RX_pData + 2);
    
    CoreBoardRevDataStruct.DataLength   =(*(CoreBoardUsartType.RX_pData + 3)) << 8 + *(CoreBoardUsartType.RX_pData + 4);
    
    if(0 == CoreBoardRevDataStruct.DataLength)
    {
      if(0x5D != *(CoreBoardUsartType.RX_pData + REQUESTFIXEDCOMMANDLEN - 1))
      {
        return state;
      }
      CoreBoardRevDataStruct.XOR8BIT         =*(CoreBoardUsartType.RX_pData + 5);
      CoreBoardRevDataStruct.TotalLength     = REQUESTFIXEDCOMMANDLEN;
      xorTemp = getXORCode(CoreBoardUsartType.RX_pData + 1, REQUESTFIXEDCOMMANDLEN - 3);
      if(CoreBoardRevDataStruct.XOR8BIT != xorTemp)
      {
        CoreBoardRevDataStruct.TotalLength = 0;
        return state;
      }
      CoreRevDataBuf[0]           = 0x5B;
      CoreRevDataBuf[1]           = CoreBoardRevDataStruct.CmdType;
      CoreRevDataBuf[2]           = CoreBoardRevDataStruct.CmdParam;
      CoreRevDataBuf[3]           = *(CoreBoardUsartType.RX_pData + 3);
      CoreRevDataBuf[4]           = *(CoreBoardUsartType.RX_pData + 4);
      CoreRevDataBuf[5]           = CoreBoardRevDataStruct.XOR8BIT;
      CoreRevDataBuf[6]           = 0x5D;
      
      CoreBoardRevDataStruct.RevOKFlag = 1;
      
      return state;
    }
    
    for(i = 5; i < CoreBoardUsartType.RX_Size; i++)
    {
      CoreRevDataBuf[i] = *(CoreBoardUsartType.RX_pData + i);
      CoreBoardRevDataStruct.NumberOfBytesReceived ++;
      if(CoreBoardRevDataStruct.DataLength == CoreBoardRevDataStruct.NumberOfBytesReceived)
      {
        if(0x5D != *(CoreBoardUsartType.RX_pData + REQUESTFIXEDCOMMANDLEN + CoreBoardRevDataStruct.NumberOfBytesReceived - 1))
        {
          CoreBoardRevDataStruct.DataLength = 0;
          CoreBoardRevDataStruct.NumberOfBytesReceived = 0;
          CoreBoardRevDataStruct.TotalLength = 0;
        }
        
        CoreBoardRevDataStruct.XOR8BIT = *(CoreBoardUsartType.RX_pData + i + 1 );
        CoreBoardRevDataStruct.TotalLength = CoreBoardRevDataStruct.DataLength + REQUESTFIXEDCOMMANDLEN;
        /* here to XOR check */
        xorTemp = getXORCode(CoreBoardUsartType.RX_pData + 1, CoreBoardRevDataStruct.TotalLength - 3);
        if(CoreBoardRevDataStruct.XOR8BIT != xorTemp)
        {
          CoreBoardRevDataStruct.TotalLength = 0;
          return state;
        }
        
        CoreRevDataBuf[0]           = 0x5B;
        CoreRevDataBuf[1]           = CoreBoardRevDataStruct.CmdType;
        CoreRevDataBuf[2]           = CoreBoardRevDataStruct.CmdParam;
        CoreRevDataBuf[3]           = *(CoreBoardUsartType.RX_pData + 3);
        CoreRevDataBuf[4]           = *(CoreBoardUsartType.RX_pData + 4);
        
        CoreRevDataBuf[i + 1]           = CoreBoardRevDataStruct.XOR8BIT;
        CoreRevDataBuf[i + 2]           = 0x5D;
        CoreBoardRevDataStruct.RevOKFlag = 1;
        return state;
      }
    }
    
  }
  return state;  
}

/*******************************************************************************
*
*       Function        :DS_HandingUartDataFromCoreBoard()
*
*       Input           :void
*
*       Return          :DS_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2018/1/31
*       Author          :bertz
*******************************************************************************/
DS_StatusTypeDef DS_HandingUartDataFromCoreBoard(void)
{
  DS_StatusTypeDef state = DS_OK;
  uint8_t xorTemp;
  uint16_t i;
  if(!CoreBoardUsartType.RX_Flag)
  {
    return state;
  }
  CoreBoardUsartType.RX_Flag = 0;
  
  /*  Handling the ACK Cmd */
  if(0xA0 == *(CoreBoardUsartType.RX_pData + 1)&0xF0)
  {
      if(CoreBoardAckedData.AckCnt > 5)
      {
        return state;
      }
      if(0x5B != *(CoreBoardUsartType.RX_pData) || 0x5D != *(CoreBoardUsartType.RX_pData + ACKFIXEDCOMMANDLEN -1))
      {
          return state;
      }
      xorTemp = getXORCode(CoreBoardUsartType.RX_pData + 1,3);
      if(xorTemp != *(CoreBoardUsartType.RX_pData + 4))
      {
        return state;
      }
      
      CoreBoardAckedData.AckCmdCode[CoreBoardAckedData.AckCnt]     = *(CoreBoardUsartType.RX_pData + 1);
      CoreBoardAckedData.AckCodeH[CoreBoardAckedData.AckCnt]       = *(CoreBoardUsartType.RX_pData + 2);
      CoreBoardAckedData.AckCodeL[CoreBoardAckedData.AckCnt]       = *(CoreBoardUsartType.RX_pData + 3);
      CoreBoardAckedData.CheckedAckFlag[CoreBoardAckedData.AckCnt] = 1;
      
      CoreBoardAckedData.AckCnt++;
      
      return state;
  }
  /* 在ACK命令处理时需要对ACkCnt进行间操作，每进行一次操作进行一次减操作 */
  
  /* Handling Request Cmd Data */
  if(CoreBoardRevDataStruct.RevOKFlag)
  {
    return state;
  }
  
  if(CoreBoardRevDataStruct.NumberOfBytesReceived < CoreBoardRevDataStruct.DataLength && 0 != CoreBoardRevDataStruct.NumberOfBytesReceived)
  {
    for(i = 0; i < CoreBoardUsartType.RX_Size; i++)
    {
      CoreRevDataBuf[5 + CoreBoardRevDataStruct.NumberOfBytesReceived] = *(CoreBoardUsartType.RX_pData + i);
      CoreBoardRevDataStruct.NumberOfBytesReceived ++;
      if(CoreBoardRevDataStruct.DataLength == CoreBoardRevDataStruct.NumberOfBytesReceived)
      {
        CoreBoardRevDataStruct.XOR8BIT = *(CoreBoardUsartType.RX_pData + i + 1);
        if(0x5D != *(CoreBoardUsartType.RX_pData + i + 2))
        {
          CoreBoardRevDataStruct.NumberOfBytesReceived = 0;
          CoreBoardRevDataStruct.DataLength = 0;
          CoreBoardRevDataStruct.TotalLength = 0;
          return state;
        }
        CoreBoardRevDataStruct.TotalLength = CoreBoardRevDataStruct.DataLength + REQUESTFIXEDCOMMANDLEN;
        /* here to check XOR code */
        xorTemp = getXORCode(CoreRevDataBuf + 1, CoreBoardRevDataStruct.TotalLength - 3);
        if(CoreBoardRevDataStruct.XOR8BIT != xorTemp)
        {
          CoreBoardRevDataStruct.NumberOfBytesReceived = 0;
          CoreBoardRevDataStruct.DataLength = 0;
          CoreBoardRevDataStruct.TotalLength = 0;      
          return state;
        }
        CoreRevDataBuf[5 + CoreBoardRevDataStruct.NumberOfBytesReceived] = xorTemp;
        CoreRevDataBuf[5 + CoreBoardRevDataStruct.NumberOfBytesReceived + 1] = 0x5D;
        CoreBoardRevDataStruct.pRevDataBuf = CoreRevDataBuf;
        CoreBoardRevDataStruct.RevOKFlag = 1;
      }
    }
    return state;
  }
  
  if(CoreBoardRevDataStruct.TotalLength)
  {
    if(0x5B != *(CoreBoardUsartType.RX_pData))
    {
      return state;
    }
    CoreBoardRevDataStruct.CmdType      =*(CoreBoardUsartType.RX_pData + 1);
    CoreBoardRevDataStruct.CmdParam     =*(CoreBoardUsartType.RX_pData + 2);
    
    CoreBoardRevDataStruct.DataLength   =(*(CoreBoardUsartType.RX_pData + 3)) << 8 + *(CoreBoardUsartType.RX_pData + 4);
    
    if(0 == CoreBoardRevDataStruct.DataLength)
    {
      if(0x5D != *(CoreBoardUsartType.RX_pData + REQUESTFIXEDCOMMANDLEN - 1))
      {
        return state;
      }
      CoreBoardRevDataStruct.XOR8BIT         =*(CoreBoardUsartType.RX_pData + 5);
      CoreBoardRevDataStruct.TotalLength     = REQUESTFIXEDCOMMANDLEN;
      xorTemp = getXORCode(CoreBoardUsartType.RX_pData + 1, REQUESTFIXEDCOMMANDLEN - 3);
      if(CoreBoardRevDataStruct.XOR8BIT != xorTemp)
      {
        CoreBoardRevDataStruct.TotalLength = 0;
        return state;
      }
      CoreRevDataBuf[0]           = 0x5B;
      CoreRevDataBuf[1]           = CoreBoardRevDataStruct.CmdType;
      CoreRevDataBuf[2]           = CoreBoardRevDataStruct.CmdParam;
      CoreRevDataBuf[3]           = *(CoreBoardUsartType.RX_pData + 3);
      CoreRevDataBuf[4]           = *(CoreBoardUsartType.RX_pData + 4);
      CoreRevDataBuf[5]           = CoreBoardRevDataStruct.XOR8BIT;
      CoreRevDataBuf[6]           = 0x5D;
      
      CoreBoardRevDataStruct.RevOKFlag = 1;
      
      return state;
    }
    
    for(i = 5; i < CoreBoardUsartType.RX_Size; i++)
    {
      CoreRevDataBuf[i] = *(CoreBoardUsartType.RX_pData + i);
      CoreBoardRevDataStruct.NumberOfBytesReceived ++;
      if(CoreBoardRevDataStruct.DataLength == CoreBoardRevDataStruct.NumberOfBytesReceived)
      {
        if(0x5D != *(CoreBoardUsartType.RX_pData + REQUESTFIXEDCOMMANDLEN + CoreBoardRevDataStruct.NumberOfBytesReceived - 1))
        {
          CoreBoardRevDataStruct.DataLength = 0;
          CoreBoardRevDataStruct.NumberOfBytesReceived = 0;
          CoreBoardRevDataStruct.TotalLength = 0;
        }
        
        CoreBoardRevDataStruct.XOR8BIT = *(CoreBoardUsartType.RX_pData + i + 1 );
        CoreBoardRevDataStruct.TotalLength = CoreBoardRevDataStruct.DataLength + REQUESTFIXEDCOMMANDLEN;
        /* here to XOR check */
        xorTemp = getXORCode(CoreBoardUsartType.RX_pData + 1, CoreBoardRevDataStruct.TotalLength - 3);
        if(CoreBoardRevDataStruct.XOR8BIT != xorTemp)
        {
          CoreBoardRevDataStruct.TotalLength = 0;
          return state;
        }
        
        CoreRevDataBuf[0]           = 0x5B;
        CoreRevDataBuf[1]           = CoreBoardRevDataStruct.CmdType;
        CoreRevDataBuf[2]           = CoreBoardRevDataStruct.CmdParam;
        CoreRevDataBuf[3]           = *(CoreBoardUsartType.RX_pData + 3);
        CoreRevDataBuf[4]           = *(CoreBoardUsartType.RX_pData + 4);
        
        CoreRevDataBuf[i + 1]           = CoreBoardRevDataStruct.XOR8BIT;
        CoreRevDataBuf[i + 2]           = 0x5D;
        CoreBoardRevDataStruct.RevOKFlag = 1;
        return state;
      }
    }
    
  }
  return state;
}

/*******************************************************************************
*
*       Function        :DS_HandingUartDataFromLeftDoorBoard()
*
*       Input           :void
*
*       Return          :DS_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2018/1/31
*       Author          :bertz
*******************************************************************************/
DS_StatusTypeDef DS_HandingUartDataFromLeftDoorBoard(void)
{
  DS_StatusTypeDef state = DS_OK;
  
  return state;
}

/*******************************************************************************
*
*       Function        :DS_HandingUartDataFromRightDoorBoard()
*
*       Input           :void
*
*       Return          :DS_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2018/1/31
*       Author          :bertz
*******************************************************************************/
DS_StatusTypeDef DS_HandingUartDataFromRightDoorBoard(void)
{
  DS_StatusTypeDef state = DS_OK;
  
  return state; 
}

/**
* @}
*/
/**
* @}
*/
/*****************************END OF FILE**************************************/
