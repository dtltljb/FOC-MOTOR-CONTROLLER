/**
 ******************************************************************************
 * @file    digital_output.h
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          digital output component of the Motor Control SDK.
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
#ifndef __DIGITALOUTPUT_H
#define __DIGITALOUTPUT_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

  /* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup digital output
 * @{
 */


/* Exported constants --------------------------------------------------------*/
#define DOutputActiveHigh       1u
#define DOutputActiveLow        0u


/* Exported types ------------------------------------------------------------*/

/**
 * @brief  digital output handler definition
 */
typedef struct
{
  DOutputState_t OutputState;       /*!< indicates the state of the digital output */
  GPIO_TypeDef* hDOutputPort;       /*!< GPIO output port. It must be equal
									                     to GPIOx x= A, B, ...*/
  uint16_t hDOutputPin;             /*!< GPIO output pin. It must be equal to
                                   	   GPIO_Pin_x x= 0, 1, ...*/
  uint8_t  bDOutputPolarity;        /*!< GPIO output polarity. It must be equal
                                       to DOutputActiveHigh or DOutputActiveLow */
}DOUT_handle_t;

/**
 * @brief  Initializes object variables, port and pin. It must be called only
 *         after PWMnCurrFdbk object initialization and DigitalOutput object
 *         creation.
 * @param pHandle handler address of the digital output component.
 * @retval none.
 */
void DOUT_Init(DOUT_handle_t *pHandle);

/**
 * @brief Accordingly with selected polarity, it sets to active or inactive the
 *        digital output
 * @param pHandle handler address of the digital output component.
 * @param OutputState_t New requested state
 * @retval none
 */
void DOUT_SetOutputState(DOUT_handle_t *pHandle, DOutputState_t State);

/**
 * @brief It returns the state of the digital output
 * @param pHandle pointer on component's handle
 * @retval OutputState_t Digital output state (ACTIVE or INACTIVE)
 */
DOutputState_t DOUT_GetOutputState(DOUT_handle_t *pHandle);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __DIGITALOUTPUT_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
