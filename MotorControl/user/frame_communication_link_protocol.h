/**
  ******************************************************************************
  * @file    frame_communication_protocol.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Frame Communication Protocol component of the Motor Control SDK.
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
#ifndef __FCLP_H
#define __FCLP_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */


#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

 /**
  * @addtogroup UILib
  * @{
  */

/** @addtogroup FCP
  * @{
  */

/* Imported types (delayed definition). This type is needed to link with
 * the client using the Frame Control Protocol. As of this release, the
 * Motor Control Protocol is the only client. */
struct U1MCP_Handle_s;


/* Exported constants --------------------------------------------------------*/

/** @brief Size of the Header of an FCP frame */
#define FCLP_HEADER_SIZE       2
/** @brief Maximum size of the payload of an FCP frame */
#define FCLP_MAX_PAYLOAD_SIZE  255
/** @brief Size of the Control Sum (actually not a CRC) of an FCP frame */
#define FCLP_CRC_SIZE          2
/** @brief Maximum size of an FCP frame, all inclusive */
#define FCLP_MAX_FRAME_SIZE    (FCLP_HEADER_SIZE + FCP_MAX_PAYLOAD_SIZE + FCLP_CRC_SIZE)

#define FCLP_CODE_ERROR_FLAG  	0x80
#define FCLP_SLAVE_DEV_FAILURE 	0xff


#define FCLP_STATUS_TRANSFER_ONGOING  0x01
#define FCLP_STATUS_WAITING_TRANSFER  0x02
#define FCLP_STATUS_INVALID_PARAMETER 0x03
#define FCLP_STATUS_TIME_OUT          0x04
#define FCLP_STATUS_INVALID_FRAME     0x05


/**
 * @brief Error Code for an RX Timeout FCP message frame.
 *
 * Such a payload is sent to the remote when a reception timeout occurs
 * (too much time elapses between the reception of the start of a frame
 * and that of its end).
 *
 * The value of the error code is set by the Frame Communication Protocol and is taken in
 * a value space that is shared with the Motor Control Protocol. It thus cannot be
 * changed.
 */
#define FCLP_MSG_RX_TIMEOUT  0x09
/**
 * @brief Error Code for an RX Bad CRC FCP message frame.
 *
 * Such a payload is sent to the remote on reception of a frame with a bad CRC.
 *
 * The value of the error code is set by the Frame Communication Protocol and is taken in
 * a value space that is shared with the Motor Control Protocol. It thus cannot be
 * changed.
 */
#define FCLP_MSG_RX_BAD_CRC  0x0A

/* Exported types ------------------------------------------------------------*/
typedef enum FCLP_FrameTransferState_e
{
  FCLP_TRANSFER_IDLE,
  FCLP_TRANSFER_ONGOING,
  FCLP_TRANSFER_HEADER,
  FCLP_TRANSFER_BUFFER,
  FCLP_TRANSFER_CRC,
  FCLP_TRANSFER_PENDING,
} FCLP_FrameTransferState_t;

/**
 * @brief This structure contains and formats a Frame Communication Protocol's frame
 */
typedef struct FCLP_Frame_s {
  uint8_t	Addr;
  uint8_t Code;                         /**< Identifier of the Frame. States the nature of the Frame. */
  uint16_t Size;                         /**< Size of the Payload of the frame in bytes. */
  uint8_t Buffer[FCLP_MAX_PAYLOAD_SIZE]; /**< buffer containing the Payload of the frame. */
  uint8_t FrameCRC;                     /**< "CRC" of the Frame. Computed on the whole frame (Code,   */
} FCLP_Frame_t;

/** @brief Callback to invoke on Frame Transmission completion in order to notify the sender. */
typedef void (* FCLP_SentFrameCallback_t)( struct U1MCP_Handle_s * o );
/** @brief Callback to invoke on Frame Reception completion in order to notify the receiver. */
typedef void (* FCLP_ReceivedFrameCallback_t)( struct U1MCP_Handle_s * o, uint8_t Code, uint8_t *Buffer, uint8_t Size );
/** @brief Callback to invoke on a Reception Timeout -- Seems unused today */
typedef void (* FCLP_RxTimeoutCallback_t)( struct U1MCP_Handle_s * o );

/**
 * @brief Frame Communication Protocol component handle structure
 */
typedef struct {
	
  struct U1MCP_Handle_s * ClientEntity;                       /**< A pointer on the protocol entity client */
  
  FCLP_SentFrameCallback_t ClientFrameSentCallback;          /**< Client's callback function, executed when sending a frame has completed */
  FCLP_ReceivedFrameCallback_t ClientFrameReceivedCallback;  /**< Client's callback function, executed when a frame has been received */
  FCLP_RxTimeoutCallback_t ClientRxTimeoutCallback;          /**< Client's callback function, executed on a frame transmission time out */

  uint16_t RxTimeout;                     /**< Frame reception timeout. Currently unused */
  uint16_t RxTimeoutCountdown;            /**< Time remaining before a reception timeout occurs. Currently unused */

  FCLP_Frame_t TxFrame;                    /**< Structure storing a frame to transmit */
  FCLP_FrameTransferState_t TxFrameState;  /**< Transmission state of the frame to transmit */
  uint8_t TxFrameLevel;                   /**< Number of bytes already sent for the frame to transmit */

  FCLP_Frame_t RxFrame;                    /**< Structure storing a frame being received */
  FCLP_FrameTransferState_t RxFrameState;  /**< Reception of the frame being received */
  uint8_t RxFrameLevel;                   /**< Number of bytes already received for the frame to receive */
} FCLP_Handle_t;

/**
 * @brief Prototype of a Start Receive function
 */
typedef uint8_t (*FCLP_ReceiveFct_t)( FCLP_Handle_t * pHandle );

/**
 * @brief Prototype of a Start Send function
 */
typedef uint8_t (*FCLP_SendFct_t)( FCLP_Handle_t * pHandle, uint8_t code, uint8_t *buffer, uint8_t size);

/**
 * @brief Prototype of an Abort Receive function
 */
typedef void (* FCLP_AbortReceiveFct_t)( FCLP_Handle_t * pHandle );

/* Exported functions ------------------------------------------------------- */

/* Initializes an FCP Component */
void FCLP_Init( FCLP_Handle_t * pHandle );

/*  */
void FCLP_SetClient( FCLP_Handle_t * pHandle,
                    struct U1MCP_Handle_s * pClient,
                    FCLP_SentFrameCallback_t pSentFrameCb,
                    FCLP_ReceivedFrameCallback_t pReceviedFrameCb,
                    FCLP_RxTimeoutCallback_t pRxTimeoutCb );

void FCLP_SetTimeout( FCLP_Handle_t * pHandle, uint16_t Timeout );


uint8_t FCLP_IsFrameValid( FCLP_Frame_t * pFrame );

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

#endif /* __FCP_H */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
