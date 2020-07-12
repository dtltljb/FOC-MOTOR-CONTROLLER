/**
  ******************************************************************************
  * @file    usart_frame_communication_protocol.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Frame Communication Protocol on USART component of the Motor Control SDK.
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
#ifndef __UFCP_H
#define __UFCP_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "frame_communication_protocol.h"

/** @addtogroup MCSDK
  * @{
  */

/**
 * @addtogroup UILib
 * @{
 */

/** @addtogroup UFCP
  * @{
  */

 /**
  * @brief Error Code for an Overrun FCP message frame.
  *
  * Such a payload is sent to the remote when an USART overrun error occurs.
  *
  * The value of the error code is set by the Frame Communication Protocol and is taken in
  * a value space that is shared with the Motor Control Protocol. It thus cannot be
  * changed.
  */
#define UFCP_MSG_OVERRUN 0x08

 /* Exported types ------------------------------------------------------------*/
typedef struct {
  FCP_Handle_t _Super;

  USART_TypeDef * USARTx;
  uint32_t USARTRemapping;
  uint32_t USARTClockSource;
  GPIO_TypeDef * RxPort;
  uint16_t RxPin;
  GPIO_TypeDef * TxPort;
  uint16_t TxPin;
  uint8_t UIIRQn;

} UFCP_Handle_t;

/* Exported functions ------------------------------------------------------- */

void UFCP_Init( UFCP_Handle_t * pHandle );

uint8_t UFCP_Receive( FCP_Handle_t * pHandle );

uint8_t UFCP_Send( FCP_Handle_t * pHandle, uint8_t code, uint8_t *buffer, uint8_t size);

void UFCP_AbortReceive( FCP_Handle_t * pHandle );

void * UFCP_RX_IRQ_Handler( UFCP_Handle_t * pHandle, unsigned short rx_data );

void UFCP_TX_IRQ_Handler( UFCP_Handle_t * pHandle );

void UFCP_OVR_IRQ_Handler( UFCP_Handle_t * pHandle );

void UFCP_TIMEOUT_IRQ_Handler( UFCP_Handle_t * pHandle );

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

#endif /* __UFCP_H */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
