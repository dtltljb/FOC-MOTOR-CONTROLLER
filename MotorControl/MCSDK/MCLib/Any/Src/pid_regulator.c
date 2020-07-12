/**
 ******************************************************************************
 * @file    pid_regulator.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the following features
 *          of the PID regulator component of the Motor Control SDK:
 *
 *           * proportional, integral and derivative computation funcions
 *           * read and write gain functions
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
#include "pid_regulator.h"

#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/**
 * @defgroup PIDRegulator PID Regulator
 * @brief PID regulator component of the Motor Control SDK
 *
 * The PID regulator component implements the following control function:
 *
 * @f[
 * u(t) = K_{p} e(t) + K_{i} \int_0^t e(\tau) \,d\tau + K_{d} \frac{de(t)}{dt}
 * @f]
 *
 * with the proportional, integral and derivative gains expressed as rational numbers, with a gain and a divisor parameter :
 *
 * @f[
 * K_{p} = \frac{K_{pg}}{K_{pd}}
 * @f]
 * @f[
 * K_{i} = \frac{K_{ig}}{K_{id}}
 * @f]
 * @f[
 * K_{d} = \frac{K_{dg}}{K_{dd}}
 * @f]
 *
 *  Each of the gain and divisor parameters, @f$K_{{p}g}@f$, @f$K_{{i}g}@f$, @f$K_{{d}g}@f$, @f$K_{{p}d}@f$,
 * @f$K_{id}@f$, @f$K_{dd}@f$, can be set independently. via the PID_SetKP(), PID_SetKPDivisorPOW2(), PID_SetKI(),
 * PID_SetKIDivisorPOW2(), PID_SetKD()
 *
 * @{
 */

/**
 * @brief  It initializes the handle
 * @param  pHandle: handler of the current instance of the PID component
 * @retval None
 */
void PID_HandleInit(PID_Handle_t *pHandle)
{
  pHandle->hKpGain =  pHandle->hDefKpGain;
  pHandle->hKiGain =  pHandle->hDefKiGain;
  pHandle->hKdGain =  pHandle->hDefKdGain;
  pHandle->wIntegralTerm = 0x00000000UL;
  pHandle->wPrevProcessVarError = 0x00000000UL;
}

/**
 * @brief  It updates the Kp gain
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKpGain: new Kp gain
 * @retval None
 */
void PID_SetKP(PID_Handle_t *pHandle, int16_t hKpGain)
{
  pHandle->hKpGain = hKpGain;
}

/**
 * @brief  It updates the Ki gain
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKiGain: new Ki gain
 * @retval None
 */
void PID_SetKI(PID_Handle_t *pHandle, int16_t hKiGain)
{
  pHandle->hKiGain = hKiGain;
}

/**
 * @brief  It returns the Kp gain
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Kp gain
 */
int16_t PID_GetKP(PID_Handle_t *pHandle)
{
  return(pHandle->hKpGain);
}

/**
 * @brief  It returns the Ki gain
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Ki gain
 */
int16_t PID_GetKI(PID_Handle_t *pHandle)
{
  return(pHandle->hKiGain);
}

/**
 * @brief  It returns the Default Kp gain
 * @param  pHandle: handler of the current instance of the PID component
 * @retval default Kp gain
 */
int16_t PID_GetDefaultKP(PID_Handle_t *pHandle)
{
  return(pHandle->hDefKpGain);
}

/**
 * @brief  It returns the Default Ki gain of the passed PI object
 * @param  pHandle: handler of the current instance of the PID component
 * @retval default Ki gain
 */
int16_t PID_GetDefaultKI(PID_Handle_t *pHandle)
{
  return(pHandle->hDefKiGain);
}

/**
 * @brief  It set a new value into the PI integral term
 * pHandle: handler of the current instance of the PID component
 * @param  wIntegralTermValue: new integral term value
 * @retval None
 */
void PID_SetIntegralTerm(PID_Handle_t *pHandle, int32_t wIntegralTermValue)
{
  pHandle->wIntegralTerm = wIntegralTermValue;

  return;
}

/**
 * @brief  It returns the Kp gain divisor
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Kp gain divisor
 */
uint16_t PID_GetKPDivisor(PID_Handle_t *pHandle)
{
  return (pHandle->hKpDivisor);
}

/**
 * @brief  It updates the Kp divisor
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKpDivisorPOW2: new Kp divisor expressed as power of 2
 * @retval None
 */
void PID_SetKPDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKpDivisorPOW2)
{
  pHandle->hKpDivisorPOW2 = hKpDivisorPOW2;
  pHandle->hKpDivisor = ((uint16_t)(1u) << hKpDivisorPOW2);
}

/**
 * @brief  It returns the Ki gain divisor of the passed PI object
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Ki gain divisor
 */
uint16_t PID_GetKIDivisor(PID_Handle_t *pHandle)
{
  return (pHandle->hKiDivisor);
}

/**
 * @brief  It updates the Ki divisor
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKiDivisorPOW2: new Ki divisor expressed as power of 2
 * @retval None
 */
void PID_SetKIDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKiDivisorPOW2)
{
  int32_t wKiDiv = ((int32_t)(1u) << hKiDivisorPOW2);
  pHandle->hKiDivisorPOW2 = hKiDivisorPOW2;
  pHandle->hKiDivisor = (uint16_t)(wKiDiv);
  PID_SetUpperIntegralTermLimit(pHandle, (int32_t)INT16_MAX * wKiDiv);
  PID_SetLowerIntegralTermLimit(pHandle, (int32_t)-INT16_MAX * wKiDiv);
}

/**
 * @brief  It set a new value for lower integral term limit
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wLowerLimit: new lower integral term limit value
 * @retval None
 */
void PID_SetLowerIntegralTermLimit(PID_Handle_t *pHandle, int32_t wLowerLimit)
{
  pHandle->wLowerIntegralLimit = wLowerLimit;
}

/**
 * @brief  It set a new value for upper integral term limit
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wUpperLimit: new upper integral term limit value
 * @retval None
 */
void PID_SetUpperIntegralTermLimit(PID_Handle_t *pHandle, int32_t wUpperLimit)
{
  pHandle->wUpperIntegralLimit = wUpperLimit;
}

/**
 * @brief  It set a new value for lower output limit
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hLowerLimit: new lower output limit value
 * @retval None
 */
void PID_SetLowerOutputLimit(PID_Handle_t *pHandle, int16_t hLowerLimit)
{
  pHandle->hLowerOutputLimit = hLowerLimit;
}

/**
 * @brief  It set a new value for upper output limit
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hUpperLimit: new upper output limit value
 * @retval None
 */
void PID_SetUpperOutputLimit(PID_Handle_t *pHandle, int16_t hUpperLimit)
{
  pHandle->hUpperOutputLimit = hUpperLimit;
}

/**
 * @brief  It set a new value into the PID Previous error variable required to
 *         compute derivative term
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wPrevProcessVarError: New previous error variable
 * @retval None
 */
void PID_SetPrevError(PID_Handle_t *pHandle, int32_t wPrevProcessVarError)
{
  pHandle->wPrevProcessVarError = wPrevProcessVarError;
  return;
}

/**
 * @brief  It updates the Kd gain
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKdGain: new Kd gain
 * @retval None
 */
void PID_SetKD(PID_Handle_t *pHandle, int16_t hKdGain)
{
  pHandle->hKdGain = hKdGain;
}

/**
 * @brief  It returns the Kd gain
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Kd gain
 */
int16_t PID_GetKD(PID_Handle_t *pHandle)
{
  return pHandle->hKdGain;
}

/**
 * @brief  It returns the Kd gain divisor of the PID object passed
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Kd gain divisor
 */
uint16_t PID_GetKDDivisor(PID_Handle_t *pHandle)
{
  return (pHandle->hKdDivisor);
}

/**
 * @brief Sets @f$K_{dd}@f$, the derivative divisor parameter of the PID component
 *
 * @param pHandle handle on the instance of the PID component to update
 * @param hKdDivisorPOW2
 */
void PID_SetKDDivisorPOW2( PID_Handle_t *pHandle, uint16_t hKdDivisorPOW2 );


#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
 * @brief  This function compute the output of a PI regulator sum of its
 *         proportional and integral terms
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wProcessVarError: current process variable error, intended as the reference
 *         value minus the present process variable value
 * @retval computed PI output
 */
int16_t PI_Controller(PID_Handle_t *pHandle, int32_t wProcessVarError)
{
  int32_t wProportional_Term, wIntegral_Term, wOutput_32,wIntegral_sum_temp;
  int32_t wDischarge = 0;
  int16_t hUpperOutputLimit = pHandle->hUpperOutputLimit;
  int16_t hLowerOutputLimit = pHandle->hLowerOutputLimit;

  /* Proportional term computation*/
  wProportional_Term = pHandle->hKpGain * wProcessVarError;

  /* Integral term computation */
  if (pHandle->hKiGain == 0)
  {
    pHandle->wIntegralTerm = 0;
  }
  else
  {
    wIntegral_Term = pHandle->hKiGain * wProcessVarError;
    wIntegral_sum_temp = pHandle->wIntegralTerm + wIntegral_Term;

    if (wIntegral_sum_temp < 0)
    {
      if (pHandle->wIntegralTerm > 0)
      {
        if (wIntegral_Term > 0)
        {
          wIntegral_sum_temp = INT32_MAX;
        }
      }
    }
    else
    {
      if (pHandle->wIntegralTerm < 0)
      {
        if (wIntegral_Term < 0)
        {
          wIntegral_sum_temp = -INT32_MAX;
        }
      }
    }

    if (wIntegral_sum_temp > pHandle->wUpperIntegralLimit)
    {
      pHandle->wIntegralTerm = pHandle->wUpperIntegralLimit;
    }
    else if (wIntegral_sum_temp < pHandle->wLowerIntegralLimit)
    {
      pHandle->wIntegralTerm = pHandle->wLowerIntegralLimit;
    }
    else
    {
      pHandle->wIntegralTerm = wIntegral_sum_temp;
    }
  }

#ifdef FULL_MISRA_C_COMPLIANCY
  wOutput_32 = (wProportional_Term/(int32_t)pHandle->hKpDivisor) + (pHandle->wIntegralTerm/(int32_t)pHandle->hKiDivisor);
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
             that Cortex-M3 assembly instruction ASR (arithmetic shift right)
             is used by the compiler to perform the shifts (instead of LSR
             logical shift right)*/
  wOutput_32 = (wProportional_Term >>pHandle->hKpDivisorPOW2) + (pHandle->wIntegralTerm >> pHandle->hKiDivisorPOW2);
#endif

  if (wOutput_32 > hUpperOutputLimit)
  {

    wDischarge = hUpperOutputLimit - wOutput_32;
    wOutput_32 = hUpperOutputLimit;
  }
  else if (wOutput_32 < hLowerOutputLimit)
  {

    wDischarge = hLowerOutputLimit - wOutput_32;
    wOutput_32 = hLowerOutputLimit;
  } else { /* Nothing to do here */ }

  pHandle->wIntegralTerm += wDischarge;

  return((int16_t)(wOutput_32));
}

#if 0
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
 * @brief  This function compute the output of a PID regulator sum of its
 *         proportional, integral and derivative terms
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wProcessVarError: current process variable error, intended as the
 *         reference value minus the present process variable value
 * @retval PID computed output
 */

int16_t PID_Controller(PID_Handle_t *pHandle, int32_t wProcessVarError)
{
  int32_t wDifferential_Term;
  int32_t wDeltaError;
  int32_t wTemp_output;

  if (pHandle->hKdGain != 0) /* derivative terms not used */
  {
    wDeltaError = wProcessVarError - pHandle->wPrevProcessVarError;
    wDifferential_Term = pHandle->hKdGain * wDeltaError;

#ifdef FULL_MISRA_C_COMPLIANCY
    wDifferential_Term /= (int32_t)pHandle->hKdDivisor;
#else
    /* WARNING: the below instruction is not MISRA compliant, user should verify
  that Cortex-M3 assembly instruction ASR (arithmetic shift right)
  is used by the compiler to perform the shifts (instead of LSR
  logical shift right)*/
    wDifferential_Term >>= pHandle->hKdDivisorPOW2;
#endif

    pHandle->wPrevProcessVarError = wProcessVarError;

    wTemp_output = PI_Controller(pHandle, wProcessVarError) + wDifferential_Term;

    if (wTemp_output > pHandle->hUpperOutputLimit)
    {
      wTemp_output = pHandle->hUpperOutputLimit;
    }
    else if (wTemp_output < pHandle->hLowerOutputLimit)
    {
      wTemp_output = pHandle->hLowerOutputLimit;
    }
    else
    {}
  }
  else
  {
    wTemp_output = PI_Controller(pHandle, wProcessVarError);
  }
  return((int16_t) wTemp_output);
}
#endif
/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
