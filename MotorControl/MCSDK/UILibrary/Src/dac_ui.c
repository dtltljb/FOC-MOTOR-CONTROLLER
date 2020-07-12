/**
  ******************************************************************************
  * @file    dac_ui.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the dac component of the Motor Control SDK:
  *           + dac initialization
  *           + dac execution
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
#include "dac_common_ui.h"
#include "dac_ui.h"


 
#define DACOFF 32768



/** @addtogroup MCSDK
  * @{
  */

/**
 * @addtogroup UILib
 * @{
 */

/** @addtogroup MCUI
  * @{
  */

/**
 * @defgroup DAC_UserInterface DAC User Interface
 *
 * @brief DAC User Interface
 *
 * @todo Complete Documentation. What difference with dac_common_ui?
 * @{
 */

/**
  * @brief  Hardware and software initialization of the DAC object. This is the
  *         implementation of the virtual function.
  * @param  pHandle pointer on related component instance. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  */
void DAC_Init(UI_Handle_t *pHandle)
{  
  DAC_UI_Handle_t *pDacHandle = (DAC_UI_Handle_t *)pHandle;

  if(pDacHandle->hDAC_CH1_ENABLED == ENABLE)
  {
    /* Enable DAC Channel1 */
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);

  }
#if defined(DAC_CHANNEL2_SUPPORT)
  if(pDacHandle->hDAC_CH2_ENABLED == ENABLE)
  {
    /* Enable DAC Channel2 */
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_2);

  }
#endif
}

/**
  * @brief  This method is used to update the DAC outputs. The selected 
  *         variables will be provided in the related output channels. This is 
  *         the implementation of the virtual function.
  * @param  pHandle pointer on related component instance. It must be a DACx_UI object casted
  *         to CUI otherwise the DACInit method will have no effect.
  */
void DAC_Exec(UI_Handle_t *pHandle)
{
  DAC_UI_Handle_t *pDacHandle = (DAC_UI_Handle_t *)pHandle;
  MC_Protocol_REG_t bCh_var;
  
  if(pDacHandle->hDAC_CH1_ENABLED == ENABLE)
  {
  bCh_var = pDacHandle->bChannel_variable[DAC_CH0];
    LL_DAC_ConvertData12LeftAligned(DAC1, LL_DAC_CHANNEL_1,
                    DACOFF + ((int16_t)UI_GetReg(pHandle,bCh_var)));
    LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_1);
  }
#if defined(DAC_CHANNEL2_SUPPORT)
  if(pDacHandle->hDAC_CH2_ENABLED == ENABLE)
  {
  bCh_var = pDacHandle->bChannel_variable[DAC_CH1];
  LL_DAC_ConvertData12LeftAligned(DAC1, LL_DAC_CHANNEL_2,
                     DACOFF + ((int16_t)UI_GetReg(pHandle,bCh_var)));
    LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_2);

  }
#endif
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

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
