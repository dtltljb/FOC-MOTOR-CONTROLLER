/**
  ******************************************************************************
  * @file    ui_task.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implementes user interface tasks definition
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
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
/* Pre-compiler coherency check */

#include "UITask.h"
#include "mc_config.h"
#include "mc_library_isr_priority_conf.h"
#include "usart_params.h"
#include "ui_exported_functions.h"
#include "parameters_conversion.h"
#include "uart1_frame_communication_protocol.h"
#include "CO_motor_interface.h"


#define OPT_DACX  0x20 /*!<Bit field indicating that the UI uses SPI AD7303 DAC.*/

DAC_UI_Handle_t * pDAC = MC_NULL;
extern DAC_UI_Handle_t DAC_UI_Params;

MCP_Handle_t * pMCP = MC_NULL;
MCP_Handle_t MCP_UI_Params,U1MCP_UI_Params; 

static volatile uint16_t  bUITaskCounter;
static volatile uint16_t  bCOMTimeoutCounter;
static volatile uint16_t  bCOM1TimeoutCounter;
static volatile uint16_t  bCANTimeoutCounter;

static volatile uint16_t  bCOMATRTimeCounter = SERIALCOM_ATR_TIME_TICKS;

#define VECT_TABLE_BASE 0x08030000

/* Setup the exported functions see UIExportedFunctions.h enum. */
void* const exportedFunctions[EF_UI_NUMBERS] =
{
  (void*)(&UI_GetReg),
  (void*)(&UI_ExecSpeedRamp),
  (void*)(&UI_SetReg),
  (void*)(&UI_ExecCmd),
  (void*)(&UI_GetSelectedMCConfig),
  (void*)(&UI_SetRevupData),
  (void*)(&UI_GetRevupData),
  (void*)(&UI_SetDAC),
  (void*)(&UI_SetCurrentReferences)
};

void UI_TaskInit( uint8_t cfg, uint32_t* pUICfg, uint8_t bMCNum, MCI_Handle_t* pMCIList[],
                  MCT_Handle_t* pMCTList[],const char* s_fwVer )
{
      pDAC = &DAC_UI_Params;      
      pDAC->_Super = UI_Params;

      UI_Init(&pDAC->_Super, bMCNum, pMCIList, pMCTList, pUICfg); /* Init UI and link MC obj */
		/*	MI_Init(&pDAC->_Super, bMCNum, pMCIList, pMCTList, pUICfg);	UI_Init() has already run,do not run this */
	
      UI_DACInit(&pDAC->_Super); /* Init DAC */
      UI_SetDAC(&pDAC->_Super, DAC_CH0, DEFAULT_DAC_CHANNEL_1);
      UI_SetDAC(&pDAC->_Super, DAC_CH1, DEFAULT_DAC_CHANNEL_2);

  if (cfg & OPT_COM)
  {
    pMCP = &MCP_UI_Params;
    pMCP->_Super = UI_Params;

    UFCP_Init( & pUSART );
    MCP_Init(pMCP, (FCP_Handle_t *) & pUSART, & UFCP_Send, & UFCP_Receive, & UFCP_AbortReceive, pDAC, s_fwVer);
    UI_Init(&pMCP->_Super, bMCNum, pMCIList, pMCTList, pUICfg); /* Initialize UI and link MC components */
    //UART1 PARAMENT
   // U1FCP_Init( & pUSART1 );
   // pMCP = &U1MCP_UI_Params;
   // MCP_Init(pMCP, (FCP_Handle_t *) & pUSART1, MC_NULL, MC_NULL, MC_NULL, pDAC, s_fwVer);
		

  }
}

void UI_Scheduler(void)
{
  if(bUITaskCounter > 0u)
  {
    bUITaskCounter--;
  }

  if(bCOMTimeoutCounter > 1u)
  {
    bCOMTimeoutCounter--;
  }
  
  if(bCOM1TimeoutCounter > 1u)
  {
    bCOM1TimeoutCounter--;
  }else{
//		LL_USART_EnableIT_TXE(USART1);
	}

  if(bCANTimeoutCounter > 1u)
  {
    bCANTimeoutCounter--;
  }
    
  if(bCOMATRTimeCounter > 1u)
  {
    bCOMATRTimeCounter--;
  }
}

void UI_DACUpdate(uint8_t bMotorNbr)
{
  if (UI_GetSelectedMC(&pDAC->_Super) == bMotorNbr)
  {  
    UI_DACExec(&pDAC->_Super); /* Exec DAC update */
  }
}

void MC_SetDAC(DAC_Channel_t bChannel, MC_Protocol_REG_t bVariable)
{
  UI_SetDAC(&pDAC->_Super, bChannel, bVariable);
}

void MC_SetUserDAC(DAC_UserChannel_t bUserChNumber, int16_t hValue)
{
  UI_SetUserDAC(&pDAC->_Super, bUserChNumber, hValue);
}

MCP_Handle_t * GetMCP(void)
{
  return pMCP;
}

UI_Handle_t * GetDAC(void)
{
  return &pDAC->_Super;
}

bool UI_IdleTimeHasElapsed(void)
{
  bool retVal = false;
  if (bUITaskCounter == 0u)
  {
    retVal = true;
  }
  return (retVal);
}

void UI_SetIdleTime(uint16_t SysTickCount)
{
  bUITaskCounter = SysTickCount;
}


bool UI_SerialCommunicationATRTimeHasElapsed(void)
{
  bool retVal = false;
  if (bCOMATRTimeCounter == 1u)
  {
    bCOMATRTimeCounter = 0u;
    retVal = true;
  }
  return (retVal);
}
/*redirect UART3 function*/
bool UI_SerialCommunicationTimeOutHasElapsed(void)
{
  bool retVal = false;
  if (bCOMTimeoutCounter == 1u)
  {
    bCOMTimeoutCounter = 0u;
    retVal = true;
  }
  return (retVal);
}

void UI_SerialCommunicationTimeOutStop(void)
{
  bCOMTimeoutCounter = 0u;
}

void UI_SerialCommunicationTimeOutStart(void)
{
  bCOMTimeoutCounter = SERIALCOM_TIMEOUT_OCCURENCE_TICKS;
}


/* UART1 function*/
bool UI_Serial1CommunicationTimeOutHasElapsed(void)
{
  bool retVal = false;
  if (bCOM1TimeoutCounter == 1u)
  {
    bCOM1TimeoutCounter = 0u;
    retVal = true;
  }
  return (retVal);
}

void UI_Serial1CommunicationTimeOutStop(void)
{
  bCOM1TimeoutCounter = 0u;
}

void UI_Serial1CommunicationTimeOutStart(void)
{
  bCOM1TimeoutCounter = SERIALCOM_TIMEOUT_OCCURENCE_TICKS;
}


/* UART1 function*/
bool UI_CanCommunicationTimeOutHasElapsed(void)
{
  bool retVal = false;
  if (bCANTimeoutCounter == 1u)
  {
    bCANTimeoutCounter = 0u;
    retVal = true;
  }
  return (retVal);
}

void UI_CanCommunicationTimeOutStop(void)
{
  bCANTimeoutCounter = 0u;
}

void UI_CanCommunicationTimeOutStart(void)
{
  bCANTimeoutCounter = SERIALCOM_TIMEOUT_OCCURENCE_TICKS;
}

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
