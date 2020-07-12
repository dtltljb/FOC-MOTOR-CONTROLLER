/**
  ******************************************************************************
  * @file    encoder_speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the Encoder component of the Motor Control SDK:
  *
  *           - computes & stores average mechanical speed [01Hz]
  *           - computes & stores average mechanical acceleration [01Hz/SpeedSamplingFreq]
  *           - computes & stores  the instantaneous electrical speed [dpp]
  *           - calculates the rotor electrical and mechanical angle
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
#include "encoder_speed_pos_fdbk.h"
#include "mc_irq_handler.h"

#include "mc_library_isr_priority_conf.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

 /** @addtogroup SpeednPosFdbk
   * @{
   */

/** @defgroup Encoder Encoder Speed & Position Feedback
  * @brief Quadrature Encoder based Speed & Position Feedback implementation
  *
  * This component is used in applications controlling a motor equipped with a quadrature encoder.
  *
  * This component uses the output of a quadrature encoder to provide a measure of the speed and
  * the position of the rotor of the motor.
  *
  * @todo Document the Encoder Speed & Position Feedback "module".
  *
  * @{
  */

/* Private defines -----------------------------------------------------------*/


/**
  * @brief  It initializes the hardware peripherals (TIMx, GPIO and NVIC) 
            required for the speed position sensor management using ENCODER 
            sensors.
  * @param  pHandle: handler of the current instance of the encoder component
  * @retval none
  */
void ENC_Init(ENCODER_Handle_t *pHandle)
{
  
  TIM_TypeDef* TIMx = pHandle->TIMx;
  uint8_t BufferSize;
  uint8_t Index;
  
  
  #ifdef TIM_CNT_UIFCPY 
    LL_TIM_EnableUIFRemap (TIMx);
    #define ENC_MAX_OVERFLOW_NB     ((uint16_t)2048) /* 2^11*/
  #else
    #define ENC_MAX_OVERFLOW_NB     (1) 
  #endif
  /* Reset counter */
  LL_TIM_SetCounter (TIMx,0);
  
  /*Calculations of convenience*/
  pHandle->U32MAXdivPulseNumber = UINT32_MAX/(uint32_t)(pHandle->PulseNumber);
  pHandle->SpeedSamplingFreqHz = pHandle->SpeedSamplingFreq01Hz/ 10u;
  
  LL_TIM_ClearFlag_UPDATE (TIMx);
  LL_TIM_EnableIT_UPDATE (TIMx);
    
  /* Enable the counting timer*/
  LL_TIM_EnableCounter (TIMx);  

  /* Erase speed buffer */
  BufferSize = pHandle->SpeedBufferSize;
  
  for (Index=0u;Index<BufferSize;Index++)
  {
    pHandle->DeltaCapturesBuffer[Index]=0;
  }  
}

/**
* @brief  Clear software FIFO where are "pushed" rotor angle variations captured
*         This function must be called before starting the motor to initialize
*	        the speed measurement process.
* @param  pHandle: handler of the current instance of the encoder component
* @retval none
*/
void ENC_Clear(ENCODER_Handle_t *pHandle)
{  
  uint8_t Index;
  for (Index=0u;Index<pHandle->SpeedBufferSize;Index++)
  {
    pHandle->DeltaCapturesBuffer[Index]=0;
  }  
  pHandle->SensorIsReliable = true;
}

/**
* @brief  It calculates the rotor electrical and mechanical angle, on the basis
*         of the instantaneous value of the timer counter.
* @param  pHandle: handler of the current instance of the encoder component
* @retval int16_t Measured electrical angle in s16degree format.
*/
int16_t ENC_CalcAngle(ENCODER_Handle_t *pHandle)
{
  int32_t wtemp1;
  int32_t wtemp2;
  int16_t htemp1;
  int16_t htemp2;
  
  wtemp1 = (int32_t)( LL_TIM_GetCounter(pHandle->TIMx)) *
    (int32_t)(pHandle->U32MAXdivPulseNumber);
  
  /*Computes and stores the rotor electrical angle*/
  wtemp2 = wtemp1 * (int32_t)pHandle->_Super.bElToMecRatio;
  htemp1 = (int16_t)(wtemp2/65536);  

  pHandle->_Super.hElAngle = htemp1;
  
  /*Computes and stores the rotor mechanical angle*/
  htemp2 = (int16_t)(wtemp1/65536);
  
  pHandle->_Super.hMecAngle = htemp2;
  
  /*Returns rotor electrical angle*/  
  return(htemp1);
}

/**
  * @brief  This method must be called with the periodicity defined by parameter
  *         hSpeedSamplingFreq01Hz. The method generates a capture event on
  *         a channel, computes & stores average mechanical speed [01Hz] (on the
  *         basis of the buffer filled by CCx IRQ), computes & stores average 
  *         mechanical acceleration [01Hz/SpeedSamplingFreq], computes & stores
  *         the instantaneous electrical speed [dpp], updates the index of the
  *         speed buffer, then checks & stores & returns the reliability state
  *         of the sensor.
  * @param  pHandle: handler of the current instance of the encoder component
  * @param  pMecSpeed01Hz pointer used to return the rotor average mechanical speed (01Hz)
  * @retval true = sensor information is reliable
  *         false = sensor information is not reliable
  */
bool ENC_CalcAvrgMecSpeed01Hz(ENCODER_Handle_t *pHandle, int16_t *pMecSpeed01Hz)
{
  TIM_TypeDef* TIMx = pHandle->TIMx;
  int32_t wOverallAngleVariation = 0;
  int32_t wtemp1;
  int32_t wtemp2;
  uint8_t bBufferIndex = 0u;
  bool bReliability = true;
  uint8_t bBufferSize = pHandle->SpeedBufferSize;
  uint32_t OverflowCntSample;
  uint32_t CntCapture;
  uint32_t directionSample;
  uint8_t OFbit=0;
  
  #ifdef TIM_CNT_UIFCPY
  /* disable Interrupt generation */
  LL_TIM_DisableIT_UPDATE (TIMx);
  #endif
  CntCapture =  LL_TIM_GetCounter (TIMx);
  OverflowCntSample = pHandle->TimerOverflowNb;
  pHandle->TimerOverflowNb = 0;
  directionSample =  LL_TIM_GetDirection(TIMx);
  #ifdef TIM_CNT_UIFCPY
  OFbit = __LL_TIM_GETFLAG_UIFCPY(CntCapture);
  if (OFbit) 
  { 
    /* If OFbit is set, overflow has occured since IT has been disabled.
	We have to take this overflow into account in the angle computation,
	but we must not take it into account a second time in the accmulator,
	so we have to clear the pending flag. If the OFbit is not set, it does not mean
	that an Interrupt has not occured since the last read, but it has not been taken
	into accout, we must not clear the interrupt in order to accumulate it */
    LL_TIM_ClearFlag_UPDATE(TIMx);
  }
  LL_TIM_EnableIT_UPDATE (TIMx);
  CLEAR_BIT(CntCapture, TIM_CNT_UIFCPY);
  #endif  
  /* If UIFCPY is not present, OverflowCntSample can not be used safely for 
  speed computation, but we still use it to check that we do not exceed one overflow 
 (sample frequency not less than mechanical motor speed */
  if ((OverflowCntSample + OFbit) > ENC_MAX_OVERFLOW_NB )
  {
	pHandle->TimerOverflowError = true;  
  }
 
  /*Calculation of delta angle*/
    if (directionSample == LL_TIM_COUNTERDIRECTION_DOWN)  
    {/* encoder timer down-counting*/
     /* if UIFCPY not present Overflow counter can not be safely used -> limitation to 1 OF. */
      #ifndef TIM_CNT_UIFCPY
      OverflowCntSample = (CntCapture > pHandle->PreviousCapture) ? 1 :0;
      #endif
      pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex]= 
        (int32_t)(CntCapture) - (int32_t)(pHandle->PreviousCapture) -
          ((int32_t)(OverflowCntSample) + OFbit ) * (int32_t)(pHandle->PulseNumber);
    }
    else  
    {/* encoder timer up-counting*/
      /* if UIFCPY not present Overflow counter can not be safely used -> limitation to 1 OF. */
      #ifndef TIM_CNT_UIFCPY
      OverflowCntSample = (CntCapture < pHandle->PreviousCapture) ? 1 :0;
      #endif
      pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex]= 
        (int32_t)(CntCapture) - (int32_t)(pHandle->PreviousCapture) +
          ((int32_t)(OverflowCntSample) + OFbit) * (int32_t)(pHandle->PulseNumber);
    }

    
  /*Computes & returns average mechanical speed [01Hz], var wtemp1*/
  for (bBufferIndex=0u;bBufferIndex<bBufferSize;bBufferIndex++)
  {
    wOverallAngleVariation += pHandle->DeltaCapturesBuffer[bBufferIndex];
  }
  wtemp1 = wOverallAngleVariation * (int32_t)(pHandle->SpeedSamplingFreq01Hz);
  wtemp2 = (int32_t)(pHandle->PulseNumber)*
    (int32_t)(pHandle->SpeedBufferSize);
  wtemp1 /= wtemp2;  
  *pMecSpeed01Hz = (int16_t)(wtemp1);
  
  /*Computes & stores average mechanical acceleration [01Hz/SpeedSamplingFreq]*/
  pHandle->_Super.hMecAccel01HzP = (int16_t)(wtemp1 - 
              pHandle->_Super.hAvrMecSpeed01Hz);
    
  /*Stores average mechanical speed [01Hz]*/
  pHandle->_Super.hAvrMecSpeed01Hz = (int16_t)wtemp1;
  
  /*Computes & stores the instantaneous electrical speed [dpp], var wtemp1*/
  wtemp1 = pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] *
    (int32_t)(pHandle->SpeedSamplingFreqHz) *
      (int32_t)pHandle->_Super.bElToMecRatio;
  wtemp1 /= (int32_t)(pHandle->PulseNumber);
  wtemp1 *= (int32_t)UINT16_MAX;
  wtemp1 /= (int32_t)(pHandle->_Super.hMeasurementFrequency); 

  pHandle->_Super.hElSpeedDpp = (int16_t)wtemp1;
  
  /*last captured value update*/
  pHandle->PreviousCapture = CntCapture;
  /*Buffer index update*/
  pHandle->DeltaCapturesIndex++;
  
  if (pHandle->DeltaCapturesIndex == pHandle->SpeedBufferSize)
  {
    pHandle->DeltaCapturesIndex = 0u;
  }
  
  /*Checks the reliability status, then stores and returns it*/
  if (pHandle->TimerOverflowError)
  {
    bReliability = false;
    pHandle->SensorIsReliable = false;
    pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
    
  }
  else {
    bReliability = SPD_IsMecSpeedReliable(&pHandle->_Super, pMecSpeed01Hz);
  }

  return(bReliability);
}

/**
  * @brief  It set istantaneous rotor mechanical angle. 
  *         As a consequence, timer counted is computed and updated.
  * @param  pHandle: handler of the current instance of the encoder component
  * @param  hMecAngle new value of rotor mechanical angle (s16degrees)
  * @retval none
  */
void ENC_SetMecAngle(ENCODER_Handle_t *pHandle, int16_t hMecAngle)
{
  TIM_TypeDef* TIMx = pHandle->TIMx;
  
  uint16_t hAngleCounts;
  uint16_t hMecAngleuint;
  
  if (hMecAngle < 0)
  {
    hMecAngle *= -1;
    hMecAngleuint = 65535u - (uint16_t)hMecAngle;
  }
  else
  {
    hMecAngleuint = (uint16_t)hMecAngle;
  }
  
  hAngleCounts = (uint16_t)(((uint32_t)hMecAngleuint *
                    (uint32_t)pHandle->PulseNumber)/65535u);
   
  TIMx->CNT = (uint16_t)(hAngleCounts);
}

/**
  * @brief  IRQ implementation of the TIMER ENCODER 
  * @param  pHandle: handler of the current instance of the encoder component
  * @param  flag used to distinguish between various IRQ sources
  * @retval none
  */
void * ENC_IRQHandler(void *pHandleVoid)
{
  ENCODER_Handle_t *pHandle = (ENCODER_Handle_t *) pHandleVoid;
  
  /*Updates the number of overflows occurred*/
  /* the handling of overflow herror is done in ENC_CalcAvrgMecSpeed01Hz */
  pHandle->TimerOverflowNb +=1u;

  return MC_NULL;
}
/**
  * @}
  */
  
/**
  * @}
  */

/** @} */


/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
