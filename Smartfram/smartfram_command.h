/**
  ******************************************************************************
  * @file    lora_command.h
  * @author  MCD Application Team
  * @brief   Header[2] for driver command.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMARTFRAM_COMMAND_H__
#define __SMARTFRAM_COMMAND_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "smartfram_FlashSys.h"
/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* Character added when a RX error has been detected */
#define SIZE_CREATE_CHECKSUM(x) ((sizeof(x)))
#define SIZE_CHECK_CHECKSUM(x)  ((sizeof(x))-1)

//#define Header[2] 	(0xaa55)



#define LORA_HEADER		2

enum
{
	TypeMSG_START_SCAN = 0x0001,
	TypeMSG_STOP_SCAN = 0x0002,
	TypeMSG_ACK_3SENSOR = 0x0004,
//	TypeMSG_ACK_SOIL_MOISTURE = 0x0005,
	TypeMSG_RESET = 0x0002,
};


typedef struct
{
	uint8_t Header[2];               // 1byte
	uint8_t	lenght;				  // 1byte
	uint8_t TypeMsg;              // 1byte
	uint8_t opcode[2];              // 1byte
	uint8_t Checksum;             // 1byte
	//\t\n					trong frame AI truyen xuong se co 2 byte nay nhung lora se khong lay 2 byte nay
}commanGetScan;   

typedef struct 
{
	uint8_t Header[2];         // 1byte
	uint8_t lenght;         // 1byte
	uint8_t TypeMsg;        // 1byte
	uint8_t opcode[2];      // 1byte
	uint8_t MacAdress[4];   // 8byte
	uint8_t Unicast[4];     // 4byte
	uint8_t TypeSensor;     // 1byte
	uint8_t Checksum;       // 1byte
}commandACKScan_t;        // 15byte

typedef struct 
{
	uint8_t Header[2];         // 1byte
	uint8_t lenght;         // 1byte
	uint8_t TypeMsg;        // 1byte
	uint8_t opcode[2];
	uint8_t Unicast[4];     // 4byte
	uint8_t Data[4];        // 4byte
	uint8_t Pin;            // 1byte
	uint8_t Checksum;       // 1byte
}commandSensorLight_t;    // 1byte

typedef struct 
{
	uint8_t Header[2];                // 1byte
	uint8_t lenght;         // 1byte
	uint8_t TypeMsg;        // 1byte
	uint8_t opcode[2];
	uint8_t Unicast[4];            // 4byte
	uint8_t Data[4];               // 4byte
	uint8_t Pin;                   // 1byte
	uint8_t Checksum;              // 1byte
}commandSensorSoilMoisture_t;    // 14byte


typedef struct 
{
	uint8_t Header[2];         // 1byte
	uint8_t lenght;         // 1byte
	uint8_t TypeMsg;        // 1byte
	uint8_t opcode[2];
	uint8_t Unicast[4];     // 4byte
	uint8_t DataTemp[2];    // 2byte
	uint8_t DataHum[2];     // 2byte
	uint8_t Pin;            // 1byte
	uint8_t Checksum;       // 1byte
}commandSensorHumTemp_t;  // 18byte

typedef struct 
{
	uint8_t Header[2];             // 1byte
	uint8_t lenght;         	   // 1byte
	uint8_t TypeMsg;       		   // 1byte
	uint8_t opcode[2];
	uint8_t Unicast[4];            // 4byte
	uint8_t DataCO2[2];            // 4byte
	uint8_t DataTVOC[2];           // 4byte
	uint8_t Pin;                   // 1byte
	uint8_t Checksum;              // 1byte
}commandSensorCO2_t;               // 18byte

typedef struct 
{
	uint8_t Header[2];             // 1byte
	uint8_t lenght;         	   // 1byte
	uint8_t TypeMsg;       		   // 1byte
	uint8_t opcode[2];
	uint8_t Unicast[4];            // 4byte
	uint8_t DataRain[4];            // 4byte
	uint8_t Pin;                   // 1byte
	uint8_t Checksum;              // 1byte
}commandSensorRain_t;               // 18byte


typedef struct
{
	uint8_t Header[2];            // 1byte               
	uint8_t lenght;        		  // 1byte
	uint8_t TypeMsg;      		  // 1byte
	uint8_t opcode[2];
	uint8_t Unicast[4];           // 4byte
	uint8_t Delete;               // 1byte
	uint8_t Time;                 // 1byte
	uint8_t Checksum;             // 1byte
}commanGetACK_PiToGW_t;           // 9byte

 

extern uint8_t Flag_UART;

typedef struct 
{
	uint8_t StartScanDevice;
	uint8_t StopScanDevice;
	uint8_t GetACKSensor;
	uint8_t ResetDevice;
	
	
	uint8_t Flag_Commant_MSG;
	uint8_t Flag_Commant_Reset_RX;
	uint32_t Count_Commant_Reset_RX;
	uint32_t Count_Commant_1s;
}flagCommandSys_t;

extern flagCommandSys_t FlagCommandSYS;

typedef union 
{
	uint8_t Data_In[4];
	uint32_t Data_Out;
}u8ToU32_u;


typedef union 
{
	float Data_In;
	uint8_t Data_Out[4];
}floatToU8_u;


/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief Initializes command module
  *
  * @param [IN] cb to signal appli that character has been received
  * @retval None
  */
void CMD_Init(void (*CmdProcessNotify)(void));

/**
  * @brief Process the command
  *
  * @param [IN] None
  * @retval None
  */
void CMD_Process(void);

/**
  * @brief Process the command
  *
  * @param [IN] None
  * @retval None
  */
void CMD_Process(void);


void Smartfram_Command_ScanACK(uint32_t MAC1, uint32_t MAC2, uint32_t Unicast, uint8_t TypeSensor,uint8_t KeyWordEnd);

void Smartfram_Command_LIGHT(uint32_t Data, uint8_t Pin, uint8_t Page, uint8_t KeyWordEnd);
void Smartfram_Command_TEMP_HUM(float DataTemp, float DataHum, uint8_t Pin, uint8_t Page, uint8_t KeyWordEnd);
void Smartfram_Command_SoilMoisture(uint32_t Data, uint8_t Pin, uint8_t Page, uint8_t KeyWordEnd);
void Smartfram_Command_CCS811(uint32_t DataCO2, uint32_t DataTVOC, uint8_t Pin, uint8_t Page, uint8_t KeyWordEnd);
void Smartfram_Command_Rain(uint32_t Data, uint8_t Pin,uint8_t Page, uint8_t KeyWordEnd);


/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __LORA_COMMAND_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
