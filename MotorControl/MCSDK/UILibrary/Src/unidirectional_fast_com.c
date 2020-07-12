/**
  ******************************************************************************
  * @file    unidirectional_fast_com.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the unidirectional_fast_com_f3xx_user_interface component of the Motor Control SDK:
  *           < Add below the list of features, one per line.>
  *           +
  *           +
  *           +
  *           +
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
#include "unidirectional_fast_com.h"
#include "mc_type.h"
#include "ui_irq_handler.h"

#define GPIO_AF_USART1  GPIO_AF_7
#define GPIO_AF_USART2  GPIO_AF_7
#define GPIO_AF_USART3  GPIO_AF_7
#define GPIO_AF_UART4   GPIO_AF_7
#define GPIO_AF_UART5   GPIO_AF_7

/* Private function prototypes -----------------------------------------------*/
static uint16_t USART_GPIOPin2Source(uint16_t GPIO_Pin);
static uint8_t USART_AF(USART_TypeDef* USARTx);

/* Private function prototypes -----------------------------------------------*/
void* UFC_IRQ_Handler(UDFastCom_Handle_t *pHandle, unsigned char flags, unsigned short rx_data);

/** @addtogroup MCSDK
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

/** @defgroup UnidirectionalFastCom Unidirectional Fast Communication
  * @brief Unidirectional Fast Serial Communication component of the Motor Control SDK
  *
  * @todo Complete documentation
  * @{
  */

/**
  * @brief  Initializes a UnidirectionalFastCom component Handle
  *
  * It initialize all vars.
  *
  * @param  pHandle Pointer on a UDFastCom_Handle_t structure to initialize
  */
void UFC_Init( UDFastCom_Handle_t *pHandle, UDFastCom_Params_t *pParams )
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Init Hardware. */
  pHandle->Hw = *pParams;

  /* Init Vars */
  pHandle->_Super.bSelectedDrive = pHandle->Hw.bDefMotor;
  pHandle->bChannel[0]           = pHandle->Hw.bDefChannel1;
  pHandle->bChannel[1]           = pHandle->Hw.bDefChannel2;
  pHandle->bChByteNum[0]         = pHandle->Hw.bCh1ByteNum;
  pHandle->bChByteNum[1]         = pHandle->Hw.bCh2ByteNum;
  pHandle->comON                 = false;
  pHandle->bChTransmitted        = 0;
  pHandle->bByteTransmitted      = 0;
  pHandle->bChNum                = pHandle->Hw.bChNum ;

  /* Enable USART clock: UASRT1 -> APB2, USART2-5 -> APB1 */
  if (pHandle->Hw.wUSARTClockSource == RCC_APB2Periph_USART1)
  {
    RCC_APB2PeriphClockCmd(pHandle->Hw.wUSARTClockSource, ENABLE);
  }
  else
  {
    RCC_APB1PeriphClockCmd(pHandle->Hw.wUSARTClockSource, ENABLE);
  }

  /* USART Init structure */
  /* Configure USART */
  USART_Init(pHandle->Hw.USARTx, pHandle->Hw.USART_InitStructure);

  GPIO_PinAFConfig(pHandle->Hw.hTxPort, USART_GPIOPin2Source(pHandle->Hw.hTxPin), USART_AF(pHandle->Hw.USARTx));

  /* Configure Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin   = pHandle->Hw.hTxPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(pHandle->Hw.hTxPort, &GPIO_InitStructure);

  if (pHandle->Hw.NVIC_InitStructure->NVIC_IRQChannelCmd == ENABLE)
  {
    /* Enable the USARTy Interrupt */
    NVIC_Init(pHandle->Hw.NVIC_InitStructure);
  }

  /* Setup IRQ. */
  pHandle->_Super.pFctIrqHandler = (UI_IrqHandler_Cb_t)UFC_IRQ_Handler;

  UIIRQ_SetIrqHandler(pHandle->Hw.bUIIRQn,
                      (UIIRQ_Handler_t)(pHandle->_Super.pFctIrqHandler),
                      (void*)(&pHandle->_Super)
                     );

  /* Enable the USART */
  USART_Cmd(pHandle->Hw.USARTx, ENABLE);
}

/*******************************************************************************
* Function Name  : USART_IRQ_Handler
* Description    : Interrupt function for the serial communication
* Input          : none
* Return         : none
*******************************************************************************/
void UFC_TX_IRQ_Handler(UDFastCom_Handle_t *pHandle)
{
  uint8_t txData = 0;
  uint8_t* pBuff;

  if (pHandle->comON)
  {
    if (pHandle->bByteTransmitted == 0)
    {
      /* First byte to be transmitted, read value and populate the buffer */
      pHandle->wBuffer = UI_GetReg(&pHandle->_Super, pHandle->bChannel[pHandle->bChTransmitted]) >> 8;
    }

    pBuff = (uint8_t*)(&(pHandle->wBuffer));
    txData = pBuff[pHandle->bByteTransmitted];

    /* Write one byte to the transmit data register */
    USART_SendData(pHandle->Hw.USARTx, txData);

    pHandle->bByteTransmitted++;
    if (pHandle->bByteTransmitted == pHandle->bChByteNum[pHandle->bChTransmitted])
    {
      pHandle->bByteTransmitted = 0;
      pHandle->bChTransmitted++;
      if (pHandle->bChTransmitted == pHandle->Hw.bChNum)
      {
        pHandle->bChTransmitted = 0;
      }
    }
  }
}

/**
  * @brief  Starts the fast unidirectional communication.
  * @param  pHandle Handle on the target unidirectional fast communication instance
  */
void UFC_StartCom( UDFastCom_Handle_t *pHandle )
{
  pHandle->comON = true;
  USART_SendData(pHandle->Hw.USARTx, ' ');
  /* Enable USART Transmit interrupts */
  USART_ITConfig(pHandle->Hw.USARTx, USART_IT_TXE, ENABLE);
}

/**
  * @brief  Stops the fast unidirectional communication.
  * @param  pHandle Handle on the target unidirectional fast communication instance
  */
void UFC_StopCom(UDFastCom_Handle_t *pHandle)
{
  pHandle->comON = false;
  /* Disable USART Transmit interrupts */
  USART_ITConfig(pHandle->Hw.USARTx, USART_IT_TXE, DISABLE);
}

/**
 * @internal
 * @brief  An internal function used to compute the GPIO Source value from GPIO pin value.
 *
 * The GPIO Source value is used for AF remapping.
 *
 * @param  GPIO_Pin Pin value to be converted.
 * @retval uint16_t The GPIO pin source value converted.
 */
static uint16_t USART_GPIOPin2Source(uint16_t GPIO_Pin)
{
  uint16_t hRetVal;
  switch (GPIO_Pin)
  {
  case GPIO_Pin_0:
    {
      hRetVal = GPIO_PinSource0;
      break;
    }
  case GPIO_Pin_1:
    {
      hRetVal = GPIO_PinSource1;
      break;
    }
  case GPIO_Pin_2:
    {
      hRetVal = GPIO_PinSource2;
      break;
    }
  case GPIO_Pin_3:
    {
      hRetVal = GPIO_PinSource3;
      break;
    }
  case GPIO_Pin_4:
    {
      hRetVal = GPIO_PinSource4;
      break;
    }
  case GPIO_Pin_5:
    {
      hRetVal = GPIO_PinSource5;
      break;
    }
  case GPIO_Pin_6:
    {
      hRetVal = GPIO_PinSource6;
      break;
    }
  case GPIO_Pin_7:
    {
      hRetVal = GPIO_PinSource7;
      break;
    }
  case GPIO_Pin_8:
    {
      hRetVal = GPIO_PinSource8;
      break;
    }
  case GPIO_Pin_9:
    {
      hRetVal = GPIO_PinSource9;
      break;
    }
  case GPIO_Pin_10:
    {
      hRetVal = GPIO_PinSource10;
      break;
    }
  case GPIO_Pin_11:
    {
      hRetVal = GPIO_PinSource11;
      break;
    }
  case GPIO_Pin_12:
    {
      hRetVal = GPIO_PinSource12;
      break;
    }
  case GPIO_Pin_13:
    {
      hRetVal = GPIO_PinSource13;
      break;
    }
  case GPIO_Pin_14:
    {
      hRetVal = GPIO_PinSource14;
      break;
    }
  case GPIO_Pin_15:
    {
      hRetVal = GPIO_PinSource15;
      break;
    }
  default:
    {
      hRetVal = 0u;
      break;
    }
  }
  return hRetVal;
}

static uint8_t USART_AF(USART_TypeDef* USARTx)
{
  uint8_t hRetVal = 0u;
  if (USARTx == USART1)
  {
    hRetVal = GPIO_AF_USART1;
  }
  else if (USARTx == USART2)
  {
    hRetVal = GPIO_AF_USART2;
  }
  else if (USARTx == USART3)
  {
    hRetVal = GPIO_AF_USART3;
  }
  else if (USARTx == UART4)
  {
    hRetVal = GPIO_AF_UART4;
  }
  else if (USARTx == UART5)
  {
    hRetVal = GPIO_AF_UART5;
  }
  return hRetVal;
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

/**
 * @}
 */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
