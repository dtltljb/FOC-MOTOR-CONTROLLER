/**
  ******************************************************************************
  * @file    unidirectional_fast_com.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          undirectional_fast_com component of the Motor Control SDK.
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
#ifndef __UNIDIRECTIONALFASTCOM_USERINTERFACE_H
#define __UNIDIRECTIONALFASTCOM_USERINTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/

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
 * @addtogroup UnidirectionalFastCom
 * @{
 */

/**
  * @brief  UnidirectionalFastCom parameters definition
  */
typedef struct
{
  /* HW Settings */
  USART_TypeDef*     USARTx;
  uint32_t           wUSARTRemapping;
  uint32_t           wUSARTClockSource;
  GPIO_TypeDef*      hTxPort;
  uint16_t           hTxPin;
  uint8_t            bUIIRQn;
  USART_InitTypeDef* USART_InitStructure;
  NVIC_InitTypeDef*  NVIC_InitStructure;

  /* Functional settings */
  MC_Protocol_REG_t  bDefChannel1;          /*!< Code of default variables to be sent Ch1.*/
  MC_Protocol_REG_t  bDefChannel2;          /*!< Code of default variables to be sent Ch2.*/
  uint8_t            bDefMotor;             /*!< Default motor selected. */
  uint8_t            bCh1ByteNum;           /*!< Number of bytes transmitted for Ch1 */
  uint8_t            bCh2ByteNum;           /*!< Number of bytes transmitted for Ch2 */
  uint8_t            bChNum;                /*!< Number of channel to be transmitted. */
} UDFastCom_Params_t;

typedef struct
{
  UI_Handle_t _Super;      		 /*!<   */

  bool comON;                    /*!< True to establish the communication false to stop it */
  MC_Protocol_REG_t bChannel[2]; /*!< Codes of variables to be sent. */
  uint8_t bChTransmitted;        /*!< Current channel to be transmitted. */
  uint8_t bByteTransmitted;      /*!< Current byte to be transmitted. */
  int32_t wBuffer;               /*!< Transmission buffer 4 bytes. */
  uint8_t bChByteNum[2];         /*!< Number of bytes transmitted. */
  uint8_t bChNum;                /*!< Number of channel to be transmitted. */

  UDFastCom_Params_t Hw;         /*!< Hardware related struct. */
} UDFastCom_Handle_t;

/* Exported functions ------------------------------------------------------- */
void UFC_Init(UDFastCom_Handle_t *pHandle, UDFastCom_Params_t *pParams);
void UFC_StartCom(UDFastCom_Handle_t *pHandle);
void UFC_StopCom(UDFastCom_Handle_t *pHandle);
void UFC_TX_IRQ_Handler(UDFastCom_Handle_t *pHandle);

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

#endif /* __UNIDIRECTIONALFASTCOM_USERINTERFACE_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
