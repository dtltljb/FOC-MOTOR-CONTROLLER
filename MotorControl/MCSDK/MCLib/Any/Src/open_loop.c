/**
  ******************************************************************************
  * @file    open_loop.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Open Loop component.
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
#include "open_loop.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup OpenLoop Control Open Loop Control
  * @brief Open Loop component of the Motor Control SDK
  *
  * Used to run the motor in open loop mode.
  *
  * @todo Document the Bus Open Loop "module".
  *
  * @{
  */

/* Private defines -----------------------------------------------------------*/

/**
  * @brief  Initialize OpenLoop variables.
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  * @param  pVSS: Pointer on virtual speed sensor structure.
  *  @retval none
  */
void OL_Init(OpenLoop_Handle_t *pHandle, VirtualSpeedSensor_Handle_t * pVSS)
{
  pHandle->hVoltage = pHandle->hDefaultVoltage;
  pHandle->pVSS = pVSS;
}

/**
  * @brief  Set Vqd according to open loop phase voltage.
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  *  @retval Voltage components Vqd conditioned values.
  */
Volt_Components OL_VqdConditioning(OpenLoop_Handle_t *pHandle)
{
  Volt_Components Vqd;

  Vqd.qV_Component1 = pHandle->hVoltage;
  Vqd.qV_Component2 = 0;

 return(Vqd);
}

/**
  * @brief  Allow to set new open loop phase voltage.
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  * @param  hNewVoltage: New voltage value to apply.
  * @retval None
  */
void OL_UpdateVoltage(OpenLoop_Handle_t *pHandle, int16_t hNewVoltage)
{
  pHandle->hVoltage = hNewVoltage;
}

/**
  * @brief  Compute phase voltage to apply according to average mechanical speed (V/F Mode).
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  * @retval None
  */
void OL_Calc(OpenLoop_Handle_t *pHandle)
{

  if (pHandle->VFMode == true)
  {
    /* V/F mode true means enabled */
    int16_t hMecSpeed01Hz = pHandle->pVSS->_Super.hAvrMecSpeed01Hz;
    int16_t hOLVoltage = pHandle->hVFOffset + (pHandle->hVFSlope*hMecSpeed01Hz);
    pHandle->hVoltage = hOLVoltage;
  }
}

/**
  * @brief  Allow activation of the Voltage versus Frequency mode (V/F mode).
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  * @param  VFEnabling: Flag to enable the V/F mode.
  * @retval None
  */
void OL_VF(OpenLoop_Handle_t *pHandle, bool VFEnabling)
{
  pHandle->VFMode = VFEnabling;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
