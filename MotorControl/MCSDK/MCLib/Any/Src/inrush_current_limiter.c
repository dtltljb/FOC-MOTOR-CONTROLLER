/**
  ******************************************************************************
  * @file    inrush_current_limiter.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions implementing the 
  *          InrushCurrentLimiter feature of the Motor Control SDK :
  *
  *          * ICL_Init() to initialize dedicated variables
  *          * ICL_Exec() to manage the ICL state machine
  *          * ICL_GetState() to get the current ICL state machine
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
#include "inrush_current_limiter.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup ICL Inrush Current Limiter
  * @brief Inrush Current Limiter component of the Motor Control SDK
  *
  * @todo Document the Inrush Current Limiter "module".
  *
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  It initializes all the needed ICL component variables.
  *         It shall be called only once, right after the ICL instance creation.
  *         It makes the bus voltage sensor and digital output assignment needed
  *         for the ICL instance usage.
  * @param  pHandle: handler of the current instance of the ICL component
  * @param  pVBS the bus voltage sensor used by the ICL.
  * @param  pDOUT the digital output used by the ICL.
  * @retval none.
  */
void ICL_Init(ICL_Handle_t *pHandle, BusVoltageSensor_Handle_t *pVBS, DOUT_handle_t *pDOUT)
{
  uint32_t wAux;
  
  pHandle->pVBS = pVBS;
  pHandle->pDOUT = pDOUT;
  pHandle->ICLstate = ICL_ACTIVE;
  DOUT_SetOutputState(pDOUT, ACTIVE);
  pHandle->hICLTicksCounter = 0u;
  wAux = (uint32_t)(pHandle->hICLDurationms);
  wAux *= (uint32_t)(pHandle->hICLFrequencyHz);
  wAux /= 1000;
  wAux -= 1;
  if (wAux > UINT16_MAX)
  {
    wAux = UINT16_MAX;
  }
  if (wAux < 1)
  {
    wAux = 1;
  }
  pHandle->hICLTotalTicks = (uint16_t)(wAux);
}

/**
  * @brief  It clocks the Inrush Current Limiter and must be called with a
  *         frequency equal to the one set in the hEACFrequencyHz parameter.
  * @param  pHandle: handler of the current instance of the ICL component
  * @retval ICLState_t returns the current ICL state machine
  */
ICL_State_t ICL_Exec(ICL_Handle_t *pHandle)
{
  /* ICL actions.*/
  switch (pHandle->ICLstate)
  {
  case ICL_ACTIVATION:
    {
      /* ICL activation: counting the step before pass in ICL_ACTIVE */
      if (pHandle->hICLTicksCounter == 0u)
      {
        pHandle->ICLstate = ICL_ACTIVE;
      }
      else
      {
        pHandle->hICLTicksCounter--;
      }
    }
    break;

  case ICL_DEACTIVATION:
    {
      /* ICL deactivation: counting the step before pass in ICL_INACTIVE.*/
      if (pHandle->hICLTicksCounter == 0u)
      {
        pHandle->ICLstate = ICL_INACTIVE;
      }
      else
      {
        pHandle->hICLTicksCounter--;
      }
    }
    break;

  case ICL_ACTIVE:
    {
      /* ICL is active: if bus is present deactivate the ICL */
      if (VBS_CheckVbus(pHandle->pVBS) != MC_UNDER_VOLT)
      {
        DOUT_SetOutputState(pHandle->pDOUT, INACTIVE);
        pHandle->ICLstate = ICL_DEACTIVATION;
        pHandle->hICLTicksCounter = pHandle->hICLTotalTicks;
      }
    }
    break;

  case ICL_INACTIVE:
    {
      /* ICL is inactive: if bus is not present activate the ICL */
      if (VBS_CheckVbus(pHandle->pVBS) == MC_UNDER_VOLT)
      {
        DOUT_SetOutputState(pHandle->pDOUT, ACTIVE);
        pHandle->ICLstate = ICL_ACTIVATION;
        pHandle->hICLTicksCounter = pHandle->hICLTotalTicks;
      }
    }
    break;

  case ICL_IDLE:
  default:
    {
    }
    break;
  }

  return pHandle->ICLstate;
}


/**
  * @brief It returns the current state of the ICL state machine.
  * @param  pHandle: handler of the current instance of the ICL component
  * @retval ICLState_t returns the current ICL state machine
  */
ICL_State_t ICL_GetState(ICL_Handle_t *pHandle)
{
  return pHandle->ICLstate;
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
