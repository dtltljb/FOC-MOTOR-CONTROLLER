/**
  ******************************************************************************
  * @file    frame_communication_protocol.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Frame Communication Protocol component of the Motor Control SDK.
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
#include "user_interface.h"
#include "motor_control_protocol.h"
#include "uart1_motor_control_protocol.h"

#include "frame_communication_link_protocol.h"

/**
 * @addtogroup MCSDK
 * @{
 */

/**
 * @addtogroup UILib
 * @{
 */

/**
 * @defgroup FCP Frame Control Protocol
 *
 * @brief A Frame Oriented transmission protocol
 *
 * This protocol is designed to transport the messages of the Motor Control Protocol
 * across physical links such as UARTs.
 *
 * The Frame Communication Protocol component serves as a base to actual implementations
 * of the protocol that operate on specific physical links. So far, the only provided implementation
 * of the protocol uses the USARTs as the physical link.
 *
 * @todo Complete documentation
 * @{
 */

void FCLP_Init( FCLP_Handle_t * pHandle )
{
  pHandle->RxTimeoutCountdown = 0;

  pHandle->TxFrame.Code = 0x0;
  pHandle->TxFrame.Size = 0;
  pHandle->TxFrame.FrameCRC = 0;
  pHandle->TxFrameState = FCLP_TRANSFER_IDLE;
  pHandle->TxFrameLevel = 0;

  pHandle->RxFrame.Code = 0x0;
  pHandle->RxFrame.Size = 0;
  pHandle->RxFrame.FrameCRC = 0;
  pHandle->RxFrameState = FCLP_TRANSFER_IDLE;
  pHandle->RxFrameLevel = 0;
}


void FCLP_SetClient( FCLP_Handle_t * pHandle,
                    struct U1MCP_Handle_s * pClient,
                    FCLP_SentFrameCallback_t pSentFrameCb,
                    FCLP_ReceivedFrameCallback_t pReceviedFrameCb,
                    FCLP_RxTimeoutCallback_t pRxTimeoutCb )
{
  if ( MC_NULL != pHandle )
  {
    pHandle->ClientEntity = pClient;
    pHandle->ClientFrameSentCallback = pSentFrameCb;
    pHandle->ClientFrameReceivedCallback = pReceviedFrameCb;
    pHandle->ClientRxTimeoutCallback = pRxTimeoutCb;
  }
}

void FCLP_SetTimeout( FCLP_Handle_t * pHandle, uint16_t Timeout )
{
  if ( MC_NULL != pHandle )
  {
    pHandle->RxTimeout = Timeout;
  }
}


uint8_t FCLP_IsFrameValid( FCLP_Frame_t * pFrame )
{
  if ( MC_NULL != pFrame )
    return 1;		//return FCLP_CalcCRC(pFrame) == pFrame->Buffer[pFrame->Size];
  else
    return 0;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
