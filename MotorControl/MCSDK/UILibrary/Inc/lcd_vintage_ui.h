/**
  ******************************************************************************
  * @file    lcd_vintage.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions of the LCDVintage module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCDVINTAGE_USERINTERFACE_H
#define __LCDVINTAGE_USERINTERFACE_H

#include "dac_ui.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

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

/** @addtogroup UserInterface_LCDVintage LCD "Vintage" User Interface
  * @{
  */

/**
  * @brief  LCDVintage class members definition
  */
typedef struct
{
  UI_Handle_t _Super;      /*!<   */

  DAC_UI_Handle_t * pDAC;

  uint8_t bDAC_CH0_Sel;          /*!< Selected DAC CH0 element */
  MC_Protocol_REG_t bDAC_CH0_ID; /*!< Selected DAC CH0 MC Protocol code */
  uint8_t bDAC_CH1_Sel;          /*!< Selected DAC CH1 element */
  MC_Protocol_REG_t bDAC_CH1_ID; /*!< Selected DAC CH1 MC Protocol code */
  uint8_t bDAC_Size;             /*!< Number of valid DAC variables */
  Curr_Components Iqdref;        /*!< Iqdref setled by LCD user */
} LCDV_Handle_t;

void LCDV_Init(UI_Handle_t *pHandle, UI_Handle_t *pDAC, const char* s_fwVer);
void LCDV_Exec(UI_Handle_t *pHandle,  UI_Handle_t *pDAC);
void LCDV_UpdateAll(UI_Handle_t *pHandle);
void LCDV_UpdateMeasured(UI_Handle_t *pHandle);
void Display_LCD(UI_Handle_t *pHandle);
void KEYS_process(UI_Handle_t *pHandle);


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

#endif /*__LCDVINTAGE_USERINTERFACE_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
