/**
  ******************************************************************************
  * @file    enc_align_ctrl.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Encoder Alignment Control component of the Motor Control SDK.
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
#include "enc_align_ctrl.h"

#include "mc_type.h"
  
/** @addtogroup MCSDK
  * @{
  */

/** @defgroup EncAlignCtrl Encoder Alignment Controller
  * @brief Encoder Alignment Controller component of the Motor Control SDK
  *
  * Initial encoder calibration which comprises a rotor alignment in a given position
  * necessary to make the information coming from a quadrature encoder absolute.
  * @{
  */

/*
* @brief  It initializes the handle
* @param  pHandle: handler of the current instance of the EncAlignCtrl component.
* @param  pSTC: the speed and torque controller used by the EAC.
* @param  pVSS: the virtual speed sensor used by the EAC.
* @param  pENC: the encoder used by the EAC.
* @retval none.
*/
void EAC_Init(EncAlign_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pSTC, VirtualSpeedSensor_Handle_t *pVSS, ENCODER_Handle_t *pENC )
{
  pHandle->pSTC = pSTC;
  pHandle->pVSS = pVSS;
  pHandle->pENC = pENC;
  pHandle->EncAligned = false;
  pHandle->EncRestart = false;
}

/**
  * @brief  It start the encoder alignment procedure.
  *	    It configures the VSS with the required angle and sets the STC to 
  *         execute the required torque ramp.
  * @param  pHandle: handler of the current instance of the EncAlignCtrl component.
  * @retval none.
  */
void EAC_StartAlignment(EncAlign_Handle_t *pHandle)
{
  uint32_t wAux;
  
  /* Set pVSS mechanical speed to zero.*/
  VSS_SetMecAcceleration(pHandle->pVSS, 0, 0u);
  
  /* Set pVSS mechanical angle.*/
  VSS_SetMecAngle(pHandle->pVSS, pHandle->hElAngle);
  
  /* Set pSTC in STC_TORQUE_MODE.*/
  STC_SetControlMode(pHandle->pSTC, STC_TORQUE_MODE);
  
  /* Set starting torque to Zero */
  STC_ExecRamp(pHandle->pSTC, 0, 0u);
  
  /* Execute the torque ramp.*/
  STC_ExecRamp(pHandle->pSTC, pHandle->hFinalTorque, (uint32_t)(pHandle->hDurationms));
  
  /* Compute hRemainingTicks, the number of thick of alignment phase.*/
  wAux = (uint32_t)pHandle->hDurationms * (uint32_t)pHandle->hEACFrequencyHz;
  wAux /= 1000u;
  pHandle->hRemainingTicks = (uint16_t)(wAux);
  pHandle->hRemainingTicks++;  
}

/**
  * @brief  It clocks the encoder alignment controller and must be called with a 
  *         frequency equal to the one settled in the parameters 
  *         hEACFrequencyHz. Calling this method the EAC is possible to verify 
  *         if the alignment duration has been finished. At the end of alignment 
  *         the encoder is set to the defined mechanical angle.
  *         Note: STC, VSS, ENC are not clocked by EAC_Exec.
  * @param  pHandle: handler of the current instance of the EncAlignCtrl component.
  * @retval bool It returns true when the programmed alignment has been 
  *         completed. 
  */
bool EAC_Exec(EncAlign_Handle_t *pHandle)
{
  bool retVal = true;

  if (pHandle->hRemainingTicks > 0u)
  {
    pHandle->hRemainingTicks--;
    
    if (pHandle->hRemainingTicks == 0u)
    {
      /* Set pVSS mechanical angle.*/
      ENC_SetMecAngle (pHandle->pENC, pHandle->hElAngle / (int16_t)(pHandle->bElToMecRatio));
      pHandle->EncAligned = true;
      retVal = true;
    }
    else
    {
      retVal = false;
    }
  }
  
  return retVal;
}

/**
  * @brief  It returns true if the encoder has been aligned at least 
  *         one time, false if hasn't been never aligned.
  * @param  pHandle: handler of the current instance of the EncAlignCtrl component.
  */
bool EAC_IsAligned(EncAlign_Handle_t *pHandle)
{
  return pHandle->EncAligned;
}

/**
  * @brief  It sets a restart after an encoder alignment.
  * @param  pHandle: handler of the current instance of the EncAlignCtrl component.
  * @param  restart: Set to true if a restart is programmed else false
  * @retval none. 
  */
void EAC_SetRestartState(EncAlign_Handle_t *pHandle, bool restart)
{
  pHandle->EncRestart = restart;
}

/**
  * @brief  Returns true if a restart after an encoder alignment has been requested.
  * @param  pHandle: handler of the current instance of the EncAlignCtrl component.
  */
bool EAC_GetRestartState(EncAlign_Handle_t *pHandle)
{
  return pHandle->EncRestart;
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
