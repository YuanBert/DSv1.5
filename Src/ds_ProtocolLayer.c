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

HandingFlag     SendOpenFlag;


AckedStruct    CoreBoardAckedData;
AckedStruct    LeftDoorBoardAckedData;
AckedStruct    RightDoorBoardAckedData;

RevDataStruct   CoreBoardRevDataStruct;
RevDataStruct   LeftDoorBoardRevDataStruct;
RevDataStruct   RightDoorBoardRevDataStruct;

SendDataStrct   CoreBoardSendDataStruct;
SendDataStrct   LeftDoorBoardSendDataStruct;
SendDataStrct   RightDoorBoardSendDataStruct;


NeedToAckStruct sNeedToAckStruct;


static uint8_t CoreRevDataBuf[DATABUFLEN];
static uint8_t CoreSenddataBuf[DATABUFLEN];    

static uint8_t LeftDoorRevDataBuf[DATABUFLEN];
static uint8_t LeftDoorSendDataBuf[DATABUFLEN];

static uint8_t RightDoorRevDataBuf[DATABUFLEN];
static uint8_t RightDoorSendDataBuf[DATABUFLEN];

tTable Table = {{0},0};

static uint8_t GetAvailableTableID(void)
{
  uint8_t reCode = 0xFF;
  uint8_t i = 0;
  
  for(i = 0; i < 16; i++)
  {
    if(0 == Table.tab[i])
    {
      return i;
    }
  }
  return reCode;
}

static uint8_t WriteStatusToTable(uint8_t tableID,uint8_t statusCode)//statusCode : 0-表示空，1-表示正在处理，2-表示处理完成可以回复
{
    uint8_t reCode = 0xFF;
    if(tableID > 16)
    {
      return reCode;
    }
    if(0 != statusCode && 0 != Table.tab[tableID])
    {
      return reCode;
    }
    Table.tab[tableID] = statusCode;
    reCode = tableID;
    return reCode;
}




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

static DS_StatusTypeDef DS_HandingUartData(pRevDataStruct pRevData,pAckedStruct pAckedData,pUSARTRECIVETYPE pUsartType ,uint8_t* pRevDataBuf)
{
  DS_StatusTypeDef state = DS_OK;
  uint8_t xorTemp;
  uint16_t i;
  
  /* 判断串口数据是否接收完成 */
  if(!(pUsartType->RX_Flag))
  {
    return state;
  }
  
  pUsartType->RX_Flag = 0;
  
  /*  Handling the ACK Cmd */
  if(0xA0 == *(pUsartType->RX_pData + 1)&0xF0)
  {
      if(pAckedData->AckCnt > 5)
      {
        return state;
      }
      if(0x5B != *(pUsartType->RX_pData) || 0x5D != *(pUsartType->RX_pData + ACKFIXEDCOMMANDLEN -1))
      {
          return state;
      }
      xorTemp = getXORCode(pUsartType->RX_pData + 1,3);
      if(xorTemp != *(pUsartType->RX_pData + 4))
      {
        return state;
      }
      
      pAckedData->AckCmdCode[pAckedData->AckCnt]     = *(pUsartType->RX_pData + 1);
      pAckedData->AckCodeH[pAckedData->AckCnt]       = *(pUsartType->RX_pData + 2);
      pAckedData->AckCodeL[pAckedData->AckCnt]       = *(pUsartType->RX_pData + 3);
      pAckedData->CheckedAckFlag[pAckedData->AckCnt] = 1;
      
      pAckedData->AckCnt++;
      
      return state;
  }
  /* 在ACK命令处理时需要对ACkCnt进行间操作，每进行一次操作进行一次减操作 */
  
  
  /* Handling Request Cmd Data */
  if(pRevData->RevOKFlag)
  {
    return state;
  }
  
  if((pRevData->NumberOfBytesReceived) < (pRevData->DataLength) && 0 != (pRevData->NumberOfBytesReceived))
  {
    for(i = 0; i < pUsartType->RX_Size; i++)
    {
      pRevDataBuf[5 + pRevData->NumberOfBytesReceived] = *(pUsartType->RX_pData + i);
      pRevData->NumberOfBytesReceived ++;
      if(pRevData->DataLength == pRevData->NumberOfBytesReceived)
      {
        pRevData->XOR8BIT = *(pUsartType->RX_pData + i + 1);
        if(0x5D != *(pUsartType->RX_pData + i + 2))
        {
          pRevData->NumberOfBytesReceived = 0;
          pRevData->DataLength = 0;
          pRevData->TotalLength = 0;
          return state;
        }
        pRevData->TotalLength = pRevData->DataLength + REQUESTFIXEDCOMMANDLEN;
        /* here to check XOR code */
        xorTemp = getXORCode(pRevDataBuf + 1, pRevData->TotalLength - 3);
        if(pRevData->XOR8BIT != xorTemp)
        {
          pRevData->NumberOfBytesReceived = 0;
          pRevData->DataLength = 0;
          pRevData->TotalLength = 0;      
          return state;
        }
        pRevDataBuf[5 + pRevData->NumberOfBytesReceived] = xorTemp;
        pRevDataBuf[5 + pRevData->NumberOfBytesReceived + 1] = 0x5D;
        pRevData->pRevDataBuf = pRevDataBuf;
        pRevData->RevOKFlag = 1;
      }
    }
    return state;
  }
  
  if(0 == pRevData->TotalLength)
  {
    if(0x5B != *(pUsartType->RX_pData))
    {
      return state;
    }
    pRevData->CmdType      =*(pUsartType->RX_pData + 1);
    pRevData->CmdParam     =*(pUsartType->RX_pData + 2);
    
    pRevData->DataLength   =(*(pUsartType->RX_pData + 3)) << 8 + *(pUsartType->RX_pData + 4);
    
    if(0 == pRevData->DataLength)
    {
      if(0x5D != *(pUsartType->RX_pData + REQUESTFIXEDCOMMANDLEN - 1))
      {
        return state;
      }
      pRevData->XOR8BIT         =*(pUsartType->RX_pData + 5);
      pRevData->TotalLength     = REQUESTFIXEDCOMMANDLEN;
      xorTemp = getXORCode(pUsartType->RX_pData + 1, REQUESTFIXEDCOMMANDLEN - 3);
      if(pRevData->XOR8BIT != xorTemp)
      {
        pRevData->TotalLength = 0;
        return state;
      }
      pRevDataBuf[0]           = 0x5B;
      pRevDataBuf[1]           = pRevData->CmdType;
      pRevDataBuf[2]           = pRevData->CmdParam;
      pRevDataBuf[3]           = *(pUsartType->RX_pData + 3);
      pRevDataBuf[4]           = *(pUsartType->RX_pData + 4);
      pRevDataBuf[5]           = pRevData->XOR8BIT;
      pRevDataBuf[6]           = 0x5D;
      
      pRevData->RevOKFlag = 1;
      
      return state;
    }
    
    for(i = 5; i < pUsartType->RX_Size; i++)
    {
      pRevDataBuf[i] = *(pUsartType->RX_pData + i);
      pRevData->NumberOfBytesReceived ++;
      if(pRevData->DataLength == pRevData->NumberOfBytesReceived)
      {
        if(0x5D != *(pUsartType->RX_pData + REQUESTFIXEDCOMMANDLEN + pRevData->NumberOfBytesReceived - 1))
        {
          pRevData->DataLength = 0;
          pRevData->NumberOfBytesReceived = 0;
          pRevData->TotalLength = 0;
        }
        
        pRevData->XOR8BIT = *(pUsartType->RX_pData + i + 1 );
        pRevData->TotalLength = pRevData->DataLength + REQUESTFIXEDCOMMANDLEN;
        /* here to XOR check */
        xorTemp = getXORCode(pUsartType->RX_pData + 1, pRevData->TotalLength - 3);
        /* 校验，如果校验不正确，退出 */
        if(pRevData->XOR8BIT != xorTemp)
        {
          pRevData->TotalLength = 0;
          return state;
        }
        
        pRevDataBuf[0]           = 0x5B;
        pRevDataBuf[1]           = pRevData->CmdType;
        pRevDataBuf[2]           = pRevData->CmdParam;
        pRevDataBuf[3]           = *(pUsartType->RX_pData + 3);
        pRevDataBuf[4]           = *(pUsartType->RX_pData + 4);
        
        pRevDataBuf[i + 1]           = pRevData->XOR8BIT;
        pRevDataBuf[i + 2]           = 0x5D;
        pRevData->RevOKFlag = 1;
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
  state = DS_HandingUartData(&LeftDoorBoardRevDataStruct,&LeftDoorBoardAckedData,&LeftDoorBoardUsartType,LeftDoorRevDataBuf);
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
  state = DS_HandingUartData(&RightDoorBoardRevDataStruct,&RightDoorBoardAckedData,&RightDoorBoardUsartType,RightDoorRevDataBuf);
  return state; 
}

/*******************************************************************************
*
*       Function        :DS_HandingCoreBoardRequest()
*
*       Input           :void
*
*       Return          :DS_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2018/2/2
*       Author          :bertz
*******************************************************************************/
DS_StatusTypeDef DS_HandingCoreBoardRequest(void)
{
  DS_StatusTypeDef state = DS_OK;
  uint8_t tempTableID;
  if(CoreBoardRevDataStruct.RevOKFlag)
  {
    tempTableID =  GetAvailableTableID();
    if(0xFF == tempTableID)
    {
      return state;
    }
    
    switch((CoreBoardRevDataStruct.CmdType) & 0xF0)
    {
    case 0xB0:sNeedToAckStruct.AckCmdCode[tempTableID] = 0xAB;
              if(0xB2 == CoreBoardRevDataStruct.CmdType && 0x01 == CoreBoardRevDataStruct.CmdParam)
              {
                sNeedToAckStruct.AckCodeH[tempTableID] = 0x02;
                SendOpenFlag.Flag = 1;
                SendOpenFlag.position = tempTableID;//记录位置
                Table.tab[tempTableID] = 0x01;//处理中
              }
              sNeedToAckStruct.DeviceType[tempTableID] = 0x01;
              Table.tabCnt++;
              break;
              
    case 0xC0:sNeedToAckStruct.AckCmdCode[tempTableID] = 0xAC;//处理系统设置命令
              if(0xC1 == CoreBoardRevDataStruct.CmdType)//设置本机
              {
                sNeedToAckStruct.AckCodeH[tempTableID] = 0x01;
                Table.tab[tempTableID] = 0x01;
                Table.tabCnt++;
                /* 此处需要处理相关的命令 */
              }
              if(0xC2 == CoreBoardRevDataStruct.CmdType)//设置左道闸机
              {
                sNeedToAckStruct.AckCodeH[tempTableID] = 0x02;
                Table.tab[tempTableID] = 0x01;
                Table.tabCnt++;
                /* 此处需要处理相关的命令*/
              }
              if(0xC3 == CoreBoardRevDataStruct.CmdType)//设置右道闸机
              {
                sNeedToAckStruct.AckCodeH[tempTableID] = 0x03;
                Table.tab[tempTableID] = 0x01;
                Table.tabCnt++;
                /* 此处需要处理相关的命令 */
              }
              sNeedToAckStruct.DeviceType[tempTableID] = 0x01;break;
              
    case 0xD0:sNeedToAckStruct.AckCmdCode[tempTableID] = 0xAD;
              if(0xD0 == CoreBoardRevDataStruct.CmdType)
              {
                sNeedToAckStruct.AckCodeH[tempTableID] = 0x00;
                if(0x01 == CoreBoardRevDataStruct.CmdParam)
                {
                    //
                  sNeedToAckStruct.AckCodeL[tempTableID] = 0x01;
                  Table.tab[tempTableID] = 0x02;//完成处理，可以回复
                  Table.tabCnt++;
                }
                if(0x02 == CoreBoardRevDataStruct.CmdParam)
                {
                    //
                  sNeedToAckStruct.AckCodeL[tempTableID] = 0x02;
                  Table.tab[tempTableID] = 0x02;   
                  Table.tabCnt++;
                }
                if(0x03 == CoreBoardRevDataStruct.CmdParam)
                {
                    //
                  sNeedToAckStruct.AckCodeL[tempTableID] = 0x03;
                  Table.tab[tempTableID] = 0x02;
                  Table.tabCnt++;
                }
                if(0x04 == CoreBoardRevDataStruct.CmdParam)
                {
                    //
                  sNeedToAckStruct.AckCodeL[tempTableID] = 0x04;
                  Table.tab[tempTableID] = 0x02;
                  Table.tabCnt++;
                }
              }
              sNeedToAckStruct.DeviceType[tempTableID] = 0x01;break;
              
    case 0xE0:sNeedToAckStruct.AckCmdCode[tempTableID] = 0xAE;
              sNeedToAckStruct.AckCodeH[tempTableID] = 0xFF;
              sNeedToAckStruct.AckCodeL[tempTableID] = 0x00;
              sNeedToAckStruct.DeviceType[tempTableID] = 0x01;
              Table.tab[tempTableID] = 0x02;
              Table.tabCnt++;break;
              
    case 0xF0:sNeedToAckStruct.AckCmdCode[tempTableID] = 0xAF;
              sNeedToAckStruct.AckCodeH[tempTableID] = 0xFF;
              sNeedToAckStruct.AckCodeL[tempTableID] = 0x00;
              sNeedToAckStruct.DeviceType[tempTableID] = 0x01;
              Table.tab[tempTableID] = 0x02;
              Table.tabCnt++;break;
    
    default: state = DS_NOCMD;break;
    }
    
    CoreBoardRevDataStruct.DataLength  = 0;
    CoreBoardRevDataStruct.TotalLength = 0;
    CoreBoardRevDataStruct.RevOKFlag   = 0;
  }
  return state; 
}

/*******************************************************************************
*
*       Function        :DS_HandingLeftDoorBoardRequest()
*
*       Input           :void
*
*       Return          :DS_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2018/2/2
*       Author          :bertz
*******************************************************************************/
DS_StatusTypeDef DS_HandingLeftDoorBoardRequest(void)
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
