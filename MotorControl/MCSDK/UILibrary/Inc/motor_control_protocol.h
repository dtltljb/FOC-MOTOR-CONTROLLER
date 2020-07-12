/**
  ******************************************************************************
  * @file    motor_control_protocol.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          motor_control_protocol component of the Motor Control SDK.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_CONTROL_PROTOCOL_H
#define __MOTOR_CONTROL_PROTOCOL_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "frame_communication_protocol.h"

#include "user_interface.h"
#include "dac_ui.h"

/**
 * @addtogroup MCSDK
 * @{
 */

/**
 * @addtogroup UILib
 * @{
 */

/**
 * @addtogroup MCUI
 * @{
 */

/**
 * @addtogroup motor_control_protocol
 * @{
 */

 /**
  * @brief  Handle structure of motor control protocol component.
  *
  */
typedef struct MCP_Handle_s
{
  UI_Handle_t _Super;     /**< Handle structure of User Interface. */


  FCP_Handle_t *pFCP;
  FCP_SendFct_t fFcpSend;
  FCP_ReceiveFct_t fFcpReceive;
  FCP_AbortReceiveFct_t fFcpAbortReceive;
  uint8_t BufferFrame[FCP_MAX_PAYLOAD_SIZE];  /**< buffer containing data */
  const char *s_fwVer;    /**< String of FW version used */
  DAC_UI_Handle_t * pDAC; /**< Pointer on DAC handle structure. */
  uint8_t BufferSize;      /**< Frame buffer size */



} MCP_Handle_t;

/* Exported types ------------------------------------------------------------*/



/*  Function used to initialize and configure the motor control protocol Component */
void MCP_Init( MCP_Handle_t *pHandle,
               FCP_Handle_t * pFCP,
               FCP_SendFct_t fFcpSend,
               FCP_ReceiveFct_t fFcpReceive,
               FCP_AbortReceiveFct_t fFcpAbortReceive,
               DAC_UI_Handle_t * pDAC,
               const char* s_fwVer);
void MCP_OnTimeOut(MCP_Handle_t *pHandle);

/*  Function used for data decoding */
void MCP_ReceivedFrame(MCP_Handle_t *pHandle, uint8_t Code, uint8_t *buffer, uint8_t Size);

/*  Function used for data transmission */
void MCP_SentFrame(MCP_Handle_t *pHandle, uint8_t Code, uint8_t *buffer, uint8_t Size);

/*  Function used to check next reception frame. */
void MCP_WaitNextFrame(MCP_Handle_t *pHandle);

/*  Allow to report the overrun error message. */
void MCP_SendOverrunMessage(MCP_Handle_t *pHandle);

/*  Allow to report the time out error message. */
void MCP_SendTimeoutMessage(MCP_Handle_t *pHandle);

/*  Allow to send an ATR message. */
void MCP_SendATRMessage(MCP_Handle_t *pHandle);

/*  Allow to send back a BAD CRC message. */
void MCP_SendBadCRCMessage(MCP_Handle_t *pHandle);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
 * @}
 */
#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __MOTOR_CONTROL_PROTOCOL_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
