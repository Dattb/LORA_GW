/**
  ******************************************************************************
  * @file    lora_command.c
  * @author  MCD Application Team
  * @brief   Main command driver dedicated to command AT
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "smartfram_command.h"
#include "stm32_adv_trace.h"
#include "smartfram_FlashSys.h"
#include "sys_app.h"
#include "usart_if.h"
#include "stdio.h"
#include "smartfram_Messeger.h"
volatile uint32_t UpdateTime=0;
#define string_size 500
uint8_t Flag_UART = 0;

flagCommandSys_t FlagCommandSYS;
unsigned char  testChecksum[16] = {0};
/*==============================khai bao mang===============================*/

char RX_FLAG_END_LINE = 0;
uint8_t RRX[string_size];
unsigned int RXI = 0, sizeddd = 0;
char temp_char;

uint8_t FS_Command_CreateChecksum(uint8_t Data[], uint8_t size);
uint8_t FS_Command_CheckHeader(uint16_t Data);
uint8_t FS_Command_CheckTypeMsg(uint8_t Data, uint8_t TypeMsg);
uint8_t FS_Command_CheckSum(uint8_t Data[], uint8_t size);
uint8_t FS_Command_CheckMSG_HeaderAndTyped(uint8_t TypeMsg);

void FS_Command_MsgProcessing();

/**
  * @brief  CMD_GetChar callback from ADV_TRACE
  * @param  rxChar th char received
  * @param  size
  * @param  error
  * @retval None
  */
static void CMD_GetChar(uint8_t *rxChar, uint16_t size, uint8_t error);

/**
  * @brief  CNotifies the upper layer that a charchter has been receveid
  * @param  None
  * @retval None
  */
static void (*NotifyCb)(void);


void CMD_Init(void (*CmdProcessNotify)(void))
{
  UTIL_ADV_TRACE_StartRxProcess(CMD_GetChar);
  /* register call back*/
  if (CmdProcessNotify != NULL)
  {
    NotifyCb = CmdProcessNotify;
  }
}

void CMD_Process(void)
{
	int i;
	char DATA[10];
  if(RX_FLAG_END_LINE == 1)
	{
		RX_FLAG_END_LINE = 0;
		#if DEBUG
			APP_PPRINTF("\ndata: %d \n", sizeddd);
			for(i=0; i<sizeddd; i++)
			{
				sprintf(DATA,"%x ",RRX[i]);
				APP_PPRINTF("%s ", DATA);
			}
		#endif
		#if COMMAND_DEBUG_GET_DATA_PI
		for(i=0; i<sizeddd; i++)
		{
			sprintf(DATA,"%x ",RRX[i]);
			APP_PPRINTF("%s ", DATA);
		}
		#endif
		HAL_Delay(100);
		FS_Command_MsgProcessing();
		FlagCommandSYS.Flag_Commant_MSG = 0;
	}
}

static void CMD_GetChar(uint8_t *rxChar, uint16_t size, uint8_t error)
{
	temp_char = *rxChar;

	#if SYS_MODE_BUG_NOT_SEND_TO_PI
		APP_PPRINTF("\n |%c| \n", temp_char);
		if(temp_char == 'x')
		{
			Flag_UART = 1;
		}
		else if(temp_char == 's')
		{
			Flag_UART = 2;
			FlagCommandSYS.StartScanDevice = 1;
				
		}
		else if(temp_char == 'r')
		{
			Flag_UART = 3;
		}
	#else
	
		if(temp_char != '\n')
		{	
			RRX[RXI] = temp_char;
			RXI++;
		}
		else
		{
			if(RRX[RXI-1] == '\t')
			{
				RRX[RXI-1] = 0x00;
				sizeddd = RXI-1;
				RX_FLAG_END_LINE=1;
				RXI = 0;
			}
			else
			{
				RRX[RXI] = temp_char;
				RXI++;
			}
		}
		
		if (NotifyCb != NULL)
		{
			NotifyCb();
		}
  #endif
}
#define MAC_8_BYTE	0

//RD_EDIT: ban tinc cap nhat tu lora -> pi
void Smartfram_Command_ScanACK(uint32_t MAC1, uint32_t MAC2, uint32_t Unicast, uint8_t TypeSensor,uint8_t KeyWordEnd){
	
	uint8_t i;
	uint8_t *Temp;
	uint8_t TempCheckSum = 0;
	commandACKScan_t Data ={0};
	
	Data.Header[0] = 0x55;
	Data.Header[1] = 0xaa;
	Data.lenght = 13;
	Data.TypeMsg = 0x02;
	Data.opcode[0] = 0x00;
	Data.opcode[1] = 0x01;
	Temp = (uint8_t*)&MAC1;

	for(i=0; i<4; i++)
	{
		Data.MacAdress[i] = Temp[i];
	}

	#if MAC_8_BYTE
		Temp = (uint8_t*)&MAC2;
		for(i=4; i<8; i++)
		{
			Data.MacAdress[i] = Temp[i-4];
		}
	#endif
	
	Temp = (uint8_t*)&Unicast;
	for(i=0; i<4; i++){
		Data.Unicast[i] = Temp[i];
	}
	
	Data.TypeSensor = TypeSensor;
	Data.Checksum = FS_Command_CreateChecksum(&Data.lenght, sizeof(commandACKScan_t) - 3);
	vcom_Trace((uint8_t*)&Data,sizeof(commandACKScan_t));
}

void Smartfram_Command_LIGHT(uint32_t Data, uint8_t Pin,uint8_t Page, uint8_t KeyWordEnd)
{
	uint8_t i;
	uint8_t *Temp;
	uint8_t TempCheckSum = 0;
	commandSensorLight_t LightComman ={0};
	
	LightComman.Header[0] = 0x55;
	LightComman.Header[1] = 0xaa;
	LightComman.lenght = 13;
	LightComman.TypeMsg = 0x02;
	LightComman.opcode[0] = 0;
	LightComman.opcode[1] = 0x04;

	Temp = (uint8_t*)&Sys_GetwayFlashArrData[Page].Unicast;
	for(i=0; i<4; i++)
	{
		LightComman.Unicast[i] = Temp[i];
	}
	
	Temp = (uint8_t*)&Data;
	for(i=0; i<4; i++)
	{
		LightComman.Data[i] = Temp[i];
	}
	
	LightComman.Pin = Pin; //%pin
	LightComman.Checksum = FS_Command_CreateChecksum(&LightComman.lenght, sizeof(commandSensorLight_t) - 3);
	vcom_Trace((uint8_t*)&LightComman,sizeof(commandSensorLight_t));
}


void Smartfram_Command_SoilMoisture(uint32_t Data, uint8_t Pin, uint8_t Page, uint8_t KeyWordEnd)
{
	uint8_t i;
	uint8_t *Temp;
	uint8_t TempCheckSum = 0;
	commandSensorSoilMoisture_t SoilMoistureComman ={0};
	SoilMoistureComman.Header[0] = 0x55;
	SoilMoistureComman.Header[1] = 0xaa;
	SoilMoistureComman.lenght = 13;
	SoilMoistureComman.TypeMsg = 0x02;
	SoilMoistureComman.opcode[0] = 0;
	SoilMoistureComman.opcode[1] = 0x05;
	
	Temp = (uint8_t*)&Sys_GetwayFlashArrData[Page].Unicast;
	for(i=0; i<4; i++){
		SoilMoistureComman.Unicast[i] = Temp[i];
	}
	
	Temp = (uint8_t*)&Data;
	for(i=0; i<4; i++){
		SoilMoistureComman.Data[i] = Temp[i];
	}
	SoilMoistureComman.Pin = Pin; //%pin
	SoilMoistureComman.Checksum = FS_Command_CreateChecksum(&SoilMoistureComman.lenght,sizeof(commandSensorSoilMoisture_t) - 3);
	vcom_Trace((uint8_t*)&SoilMoistureComman,sizeof(commandSensorLight_t));
}



void Smartfram_Command_TEMP_HUM(float DataTemp, float DataHum, uint8_t Pin, uint8_t Page, uint8_t KeyWordEnd)
{
	for(unsigned char i=0;i<15;i++){
		testChecksum[i] = i;
	}
	uint8_t i;
	uint8_t *Temp;
	uint8_t TempCheckSum = 0;
	commandSensorHumTemp_t TempHumComman = {0};
	floatToU8_u TempData;
	uint16_t u16Temp;
	uint16_t u16Hum;
	char buffer[30];
	TempHumComman.Header[0] = 0x55;
	TempHumComman.Header[1] = 0xaa;
	TempHumComman.lenght = 13;
	TempHumComman.TypeMsg = 0x02;
	TempHumComman.opcode[0] = 0;
	TempHumComman.opcode[1] = 0x06;
	Temp = (uint8_t*)&Sys_GetwayFlashArrData[Page].Unicast;
	for(i=0; i<4; i++)
	{
		TempHumComman.Unicast[i] = Temp[i];
	}

	u16Temp = (uint32_t)(DataTemp*10); 
	TempHumComman.DataTemp[0] = u16Temp>>8;
	TempHumComman.DataTemp[1] = u16Temp;
	
	u16Hum = (uint32_t)(DataHum*10); 
	TempHumComman.DataHum[0] = u16Hum >> 8;
	TempHumComman.DataHum[1] = u16Hum;

	TempHumComman.Pin = Pin; //%pin
	TempHumComman.Checksum = FS_Command_CreateChecksum(&TempHumComman.lenght,sizeof(commandSensorHumTemp_t) - 3);
	vcom_Trace((uint8_t*)&TempHumComman,sizeof(commandSensorHumTemp_t));	
}



void Smartfram_Command_CCS811(uint32_t DataCO2, uint32_t DataTVOC, uint8_t Pin, uint8_t Page, uint8_t KeyWordEnd)
{
	uint8_t i;
	uint8_t *Temp;
	uint8_t TempCheckSum = 0;
	commandSensorCO2_t TempCSS811Comman = {0};
	TempCSS811Comman.Header[0] = 0x55;
	TempCSS811Comman.Header[1] = 0xaa;
	TempCSS811Comman.lenght = 13;
	TempCSS811Comman.TypeMsg = 0x02;
	TempCSS811Comman.opcode[0] = 0;
	TempCSS811Comman.opcode[1] = 0x07;
	
	Temp = (uint8_t*)&Sys_GetwayFlashArrData[Page].Unicast;
	for(i=0; i<4; i++){
		TempCSS811Comman.Unicast[i] = Temp[i];
	}
	TempCSS811Comman.DataCO2[0] = DataCO2 >> 8;
	TempCSS811Comman.DataCO2[1] = DataCO2;
	TempCSS811Comman.DataTVOC[0] = DataTVOC >> 8;
	TempCSS811Comman.DataTVOC[1] = DataTVOC;
	TempCSS811Comman.Pin = Pin; //%pin
	TempCSS811Comman.Checksum = FS_Command_CreateChecksum(&TempCSS811Comman.lenght,sizeof(commandSensorCO2_t) - 3);
	vcom_Trace((uint8_t*)&TempCSS811Comman,sizeof(TempCSS811Comman));
}


void Smartfram_Command_Rain(uint32_t DataRain, uint8_t Pin, uint8_t Page, uint8_t KeyWordEnd)
{
	uint8_t i;
	uint8_t *Temp;
	uint8_t TempCheckSum = 0;
	commandSensorRain_t TempRainCommand = {0};
	TempRainCommand.Header[0] = 0x55;
	TempRainCommand.Header[1] = 0xaa;
	TempRainCommand.lenght = 13;
	TempRainCommand.TypeMsg = 0x02;
	TempRainCommand.opcode[0] = 0;
	TempRainCommand.opcode[1] = 0x08;
	
	Temp = (uint8_t*)&Sys_GetwayFlashArrData[Page].Unicast;
	for(i=0; i<4; i++){
		TempRainCommand.Unicast[i] = Temp[i];
	}
	Temp = (uint8_t*)&DataRain;
	for(i=0; i<4; i++){
		TempRainCommand.DataRain[i] = Temp[i];
	}
	TempRainCommand.Pin = Pin; //%pin
	TempRainCommand.Checksum = FS_Command_CreateChecksum(&TempRainCommand.lenght,sizeof(commandSensorCO2_t) - 3);
	vcom_Trace((uint8_t*)&TempRainCommand,sizeof(TempRainCommand));
}

uint8_t FS_Command_CheckSum(uint8_t Data[], uint8_t size)
{
	uint8_t BufferCounter;
	uint8_t TempCreateCheckSum = 0;

	for (BufferCounter = 0; BufferCounter < size; BufferCounter++)
	{
			#if 0
			if(Data[BufferCounter] <10)
				APP_PPRINTF("0%x ", Data[BufferCounter]);
			else
				APP_PPRINTF("%x ", Data[BufferCounter]);
			#endif
			TempCreateCheckSum ^= Data[BufferCounter];
	}
		#if 0
			APP_PPRINTF("\n CheckSum: %x \n", TempCreateCheckSum);
		#endif
	return TempCreateCheckSum;
}


uint8_t FS_Command_CreateChecksum(uint8_t Data[], uint8_t size)
{
  uint8_t BufferCounter;
  uint8_t TempCreateCheckSum = 0;

  for (BufferCounter = 0; BufferCounter < size; BufferCounter++)
  {
		#if 0
		if(Data[BufferCounter] <10)
			APP_PPRINTF("0%x ", Data[BufferCounter]);
		else
			APP_PPRINTF("%x ", Data[BufferCounter]);
		#endif
		TempCreateCheckSum ^= Data[BufferCounter];
  }
	#if 0
		APP_PPRINTF("\n CheckSum: %x \n", TempCreateCheckSum);
	#endif
  return TempCreateCheckSum;
}

uint8_t FS_Command_CheckHeader(uint16_t Data)
{
	 if(Data == 0xaa55)
	 {
		 return 1;
	 }
	 else
	 {
		 return 0;
	 }
}

uint8_t FS_Command_CheckTypeMsg(uint8_t Data, uint8_t TypeMsg)
{
	 if(Data == TypeMsg)
	 {
		 return 1;
	 }
	 else
	 {
		 return 0;
	 }
}


uint8_t FS_Command_CheckMSG_HeaderAndTyped(uint8_t TypeMsg)
{
	uint16_t rxHeader = RRX[0] << 8 | RRX[1];
	//RX[2] is lenght (not use here)
	uint16_t rxTypeMessage = (RRX[4] << 8) | RRX[5];
	//	APP_PPRINTF("\n Header =  %x", rxHeader);
	if((rxHeader == 0x55aa) &&  (RRX[3] == LORA_HEADER) && (rxTypeMessage == TypeMsg))
	{
		return 1;
	}
	return 0;
}


void FS_Command_MsgProcessing()
{
	commanGetScan *TempGetScan;
	commanGetACK_PiToGW_t *TempACK_PiToGW_t;
	char buffer_temp[30];
	if(FS_Command_CheckMSG_HeaderAndTyped(TypeMSG_START_SCAN) == 1)
	{
		TempGetScan = (commanGetScan*)RRX;
		
		//APP_PPRINTF("\nscan\n");
		#if DEBUG
		APP_PPRINTF("\n Header =  %x", TempGetScan->Header);
		APP_PPRINTF("\n TypeMsg = %x", TempGetScan->TypeMsg);
		APP_PPRINTF("\n Checksum = %x", TempGetScan->Checksum);
		#endif
		if(FS_Command_CheckSum(&(TempGetScan->lenght), sizeof(commanGetScan) - 3) == TempGetScan->Checksum)
		{
				vcom_Trace((uint8_t*)TempGetScan,sizeof(commanGetScan));
			//	APP_PPRINTF("\nscan1\n");
			FlagCommandSYS.StartScanDevice = 1;
		}
	}
	else if(FS_Command_CheckMSG_HeaderAndTyped(TypeMSG_STOP_SCAN) == 1)
	{
		uint32_t TempUnicast = 0;
		TempGetScan = (commanGetScan*)RRX;
		APP_PPRINTF("\nstop scan\n");
		if(FS_Command_CheckSum(&(TempGetScan->lenght), sizeof(commanGetScan) - 3) == TempGetScan->Checksum)
		{
			
			#if 0
				APP_PPRINTF("\n Header =  %x", TempGetScan->Header);
				APP_PPRINTF("\n TypeMsg = %x", TempGetScan->TypeMsg);
				APP_PPRINTF("\n Checksum = %x", TempGetScan->Checksum);
			#endif
			FlagCommandSYS.StopScanDevice = 1;
		}
	}
	else if(FS_Command_CheckMSG_HeaderAndTyped(TypeMSG_ACK_3SENSOR) == 1)
	{
		u8ToU32_u TempUnicast;
		TempACK_PiToGW_t = (commanGetACK_PiToGW_t*)RRX;
		if(FS_Command_CheckSum(&(TempACK_PiToGW_t->lenght), sizeof(commanGetACK_PiToGW_t) - 3) == TempACK_PiToGW_t->Checksum)
		{
			TempUnicast.Data_In[0] = TempACK_PiToGW_t->Unicast[0];
			TempUnicast.Data_In[1] = TempACK_PiToGW_t->Unicast[1];
			TempUnicast.Data_In[2] = TempACK_PiToGW_t->Unicast[2];
			TempUnicast.Data_In[3] = TempACK_PiToGW_t->Unicast[3];
			#if 0
				APP_PPRINTF("\n Header =  %x", TempACK_PiToGW_t->Header);
				APP_PPRINTF("\n TypeMsg = %x", TempACK_PiToGW_t->TypeMsg);
				APP_PPRINTF("\n Unicast = %x", TempUnicast);
				APP_PPRINTF("\n Delete = %x", TempACK_PiToGW_t->Delete);
				APP_PPRINTF("\n TIME =  %x", TempACK_PiToGW_t->Time);
				APP_PPRINTF("\n Checksum = %x", TempACK_PiToGW_t->Checksum);
			#endif
			SendMsgGetwayToDevice.Unicast = TempUnicast.Data_Out;
			SendMsgGetwayToDevice.Msg[0] = TempACK_PiToGW_t->Delete;
			SendMsgGetwayToDevice.Msg[1] = TempACK_PiToGW_t->Time;
			FlagCommandSYS.GetACKSensor = 1;
		}
	}
}



/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
