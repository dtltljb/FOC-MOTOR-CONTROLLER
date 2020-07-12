/**
  ******************************************************************************
  * @file    usart_frame_communication_protocol.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Frame Communication Protocol for USART component of the Motor Control SDK.
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


    /* ****************************************** */
    /* *** BEWARE! THIS CODE IS FOR STM32F3xx *** */
    /* ****************************************** */

/* Includes ------------------------------------------------------------------*/

#include "ui_irq_handler.h"

//#include "uart1_motor_control_protocol.h"
#include "frame_communication_link_protocol.h"
#include "uart1_frame_communication_protocol.h"


/** @addtogroup MCSDK
  * @{
  */

/** @defgroup UFCP USART Frame Communication Protocol
  * @brief UFCP UFCP component of the Motor Control SDK
  *
  * Detailed documentation for the component.
  * @{
  */

/* Private macros ------------------------------------------------------------*/

#define U1FCP_IRQ_FLAG_RX      0
#define U1FCP_IRQ_FLAG_TX      1
#define U1FCP_IRQ_FLAG_OVERRUN 2
#define U1FCP_IRQ_FLAG_TIMEOUT 3
#define U1FCP_IRQ_FLAG_ATR     4


#define ENTER_PRESS_KEY     0X0D


/* Private function prototypes -----------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
static const uint16_t UFCP_Usart_Timeout_none = 0;
static const uint16_t UFCP_Usart_Timeout_start = 1;
static const uint16_t UFCP_Usart_Timeout_stop = 2;

/* Functions ---------------------------------------------------------*/

void U1FCP_Init( U1FCP_Handle_t * pHandle )
{

  /* Initialize generic component part */
  FCLP_Init( & pHandle->_Super );
}

/*
 *receive data startup timeout check,Timeout output timeout flag 
 */
void * U1FCP_RX_IRQ_Handler( U1FCP_Handle_t * pHandle, unsigned short rx_data )
{
  void * ret_val = (void *) & UFCP_Usart_Timeout_start;		//check receive frame distance
  FCLP_Handle_t * pBaseHandle = & pHandle->_Super;
  uint8_t error_code;

//  if ( FCLP_TRANSFER_IDLE != pBaseHandle->RxFrameState )
//  {
    uint8_t rx_byte = (uint8_t) rx_data;
        if( pBaseHandle->RxFrameLevel < FCLP_MAX_PAYLOAD_SIZE) 
        {
          // read byte is for the payload
          pBaseHandle->RxFrame.Buffer[pBaseHandle->RxFrameLevel] = rx_byte;
        	pBaseHandle->RxFrameLevel++;					
          pBaseHandle->RxFrame.Size	=	pBaseHandle->RxFrameLevel;
        	pBaseHandle->RxTimeoutCountdown = pBaseHandle->RxTimeout;		
        }
        else
        {
        	ret_val = (void *) & UFCP_Usart_Timeout_stop;
        	pBaseHandle->RxFrameLevel=0;
          pBaseHandle->RxFrameState = FCLP_TRANSFER_IDLE;
        }
				
//    switch ( pBaseHandle->RxFrameLevel )
//    {
    	
//      case 0: // First Byte received --> The Code
//        pBaseHandle->Addr = rx_byte;
//        /* Start Rx Timeout */
//        pBaseHandle->RxTimeoutCountdown = pBaseHandle->RxTimeout;
//        pBaseHandle->RxFrameLevel++;
//        break;

//      case 1: // Second Byte received --> frame function code
//        pBaseHandle->Code = rx_byte;
//        pBaseHandle->RxFrameLevel++;
//        break;

//      default: 
//        if( pBaseHandle->RxFrameLevel < FCLP_MAX_PAYLOAD_SIZE) 
//        {
//          // read byte is for the payload
//          pBaseHandle->RxFrame.Buffer[pBaseHandle->RxFrameLevel] = rx_byte;
//        	pBaseHandle->RxFrameLevel++;					
//          pBaseHandle->RxFrame.Size	=	pBaseHandle->RxFrameLevel;
//        	pBaseHandle->RxTimeoutCountdown = pBaseHandle->RxTimeout;


//          /* Stop Rx Timeout */
//          pBaseHandle->RxTimeoutCountdown = 0;
//          /* Disable the reception IRQ */
//          LL_USART_DisableIT_RXNE(pHandle->USARTx);
//          /* Indicate the reception is complete. */
//          pBaseHandle->RxFrameState = FCLP_TRANSFER_IDLE;
//           /* OK. the frame is considered correct. Let's forward to client. */
//          pBaseHandle->ClientFrameReceivedCallback( pBaseHandle->ClientEntity,
//                                                      pBaseHandle->RxFrame.Code,
//                                                      pBaseHandle->RxFrame.Buffer,
//                                                      pBaseHandle->RxFrame.Size );

//        }
//        else
//        {
//        	ret_val = (void *) & UFCP_Usart_Timeout_stop;
//        	pBaseHandle->RxFrameLevel=0;
//          pBaseHandle->RxFrameState = FCLP_TRANSFER_IDLE;
//        }
//    } /* end of switch ( pBaseHandle->RxFrameLevel ) */
	
//  } /* end of if ( FCLP_TRANSFER_IDLE != pBaseHandle->RxFrameState ) */

  return ret_val;
}

/*
 * send TxFrame.Size data,over disable txe irq callbank client
 */
void U1FCP_TX_IRQ_Handler( U1FCP_Handle_t * pHandle )
{
  FCLP_Handle_t * pBaseHandle = & pHandle->_Super;
  uint16_t tx_data;
  
   // if ( FCLP_TRANSFER_IDLE != pBaseHandle->TxFrameState )
  // {
    // switch ( pBaseHandle->TxFrameLevel )
    // {
      // case 0:
        // tx_data = (uint16_t) pBaseHandle->TxFrame.Addr;
        // break;
      // case 1:
        // tx_data = (uint16_t) pBaseHandle->TxFrame.Code;
        // break;
			
      // default:
        // if ( pBaseHandle->TxFrameLevel < pBaseHandle->TxFrame.Size  )
        // {
          // tx_data = (uint16_t) pBaseHandle->TxFrame.Buffer[ pBaseHandle->TxFrameLevel ];
        // }
        // else
        // {
          // tx_data = (uint16_t) pBaseHandle->TxFrame.FrameCRC;
        // }
        
        // break;
    // }/* end of switch ( pBaseHandle->TxFrameLevel ) */
    
	// tx_data = (uint16_t) pBaseHandle->TxFrame.Buffer[ pBaseHandle->TxFrameLevel ];
    // /* Send the data byte */
    // LL_USART_TransmitData8(pHandle->USARTx, tx_data);

    // if ( pBaseHandle->TxFrameLevel < pBaseHandle->TxFrame.Size  )
    // {
      // pBaseHandle->TxFrameLevel++;
    // }
    // else
    // {
      // LL_USART_DisableIT_TXE(pHandle->USARTx);
      // pBaseHandle->TxFrameState = FCLP_TRANSFER_IDLE;

     //pBaseHandle->ClientFrameSentCallback( pBaseHandle->ClientEntity );
    // }

  // }  /* end of if ( FCLP_TRANSFER_IDLE != pBaseHandle->TxFrameState ) */
  
  	tx_data = (uint16_t) pBaseHandle->TxFrame.Buffer[ pBaseHandle->TxFrameLevel ];
    /* Send the data byte */
    //LL_USART_TransmitData8(pHandle->USARTx, tx_data);
	LL_USART_TransmitData8(USART1, tx_data);
    if ( pBaseHandle->TxFrameLevel < pBaseHandle->TxFrame.Size -1 )
    {
      pBaseHandle->TxFrameLevel++;
    }
    else
    {
      LL_USART_DisableIT_TXE(USART1);
      pBaseHandle->TxFrameState = FCLP_TRANSFER_IDLE;
	  pBaseHandle->TxFrameLevel = 0;
     // pBaseHandle->ClientFrameSentCallback( pBaseHandle->ClientEntity );
    }
  
}

/*
 *
 */
void U1FCP_OVR_IRQ_Handler( U1FCP_Handle_t * pHandle )
{
  FCLP_Handle_t * pBaseHandle = & pHandle->_Super;
  uint8_t error_code;

  error_code = U1FCP_MSG_OVERRUN;
  (void) U1FCP_Send( pBaseHandle, FCLP_CODE_NACK, & error_code, 1 );

}

/*
 *
 */
void U1FCP_TIMEOUT_IRQ_Handler( U1FCP_Handle_t * pHandle )
{
  FCLP_Handle_t * pBaseHandle = & pHandle->_Super;
  uint8_t error_code;

  error_code = FCLP_MSG_RX_TIMEOUT;
  (void) U1FCP_Send( pBaseHandle, FCLP_CODE_NACK, & error_code, 1 );

}

uint8_t U1FCP_Receive( FCLP_Handle_t * pHandle )
{
  uint8_t ret_val;

  if ( FCLP_TRANSFER_IDLE == pHandle->RxFrameState )
  {
    U1FCP_Handle_t * pActualHandle = (U1FCP_Handle_t *) pHandle;

    pHandle->RxFrameLevel = 0;
    pHandle->RxFrameState = FCLP_TRANSFER_ONGOING;

    LL_USART_EnableIT_RXNE(pActualHandle->USARTx);
    ret_val = FCLP_STATUS_WAITING_TRANSFER;
  }
  else
  {
    ret_val = FCLP_STATUS_TRANSFER_ONGOING;
  }

  return ret_val;
}

uint8_t U1FCP_Send( FCLP_Handle_t * pHandle, uint8_t code, uint8_t *buffer, uint8_t size)
{
  uint8_t ret_val;

  if ( FCLP_TRANSFER_IDLE == pHandle->TxFrameState )
  {
    U1FCP_Handle_t * pActualHandle = (U1FCP_Handle_t *) pHandle;
    uint8_t *dest = pHandle->TxFrame.Buffer;

    pHandle->TxFrame.Code = code;
    pHandle->TxFrame.Size = size;
    while ( size-- ) *dest++ = *buffer++;
    //pHandle->TxFrame.FrameCRC = FCLP_CalcCRC( & pHandle->TxFrame );

    pHandle->TxFrameLevel = 0;
    pHandle->TxFrameState = FCLP_TRANSFER_ONGOING;

    //LL_USART_EnableIT_TXE(pActualHandle->USARTx);
		LL_USART_EnableIT_TXE(USART1);
    ret_val = FCLP_STATUS_WAITING_TRANSFER;
  }
  else
  {
    ret_val = FCLP_STATUS_TRANSFER_ONGOING;
  }

  return ret_val;
}

void U1FCP_AbortReceive( FCLP_Handle_t * pHandle )
{
  pHandle->RxFrameState = FCLP_TRANSFER_IDLE;
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
