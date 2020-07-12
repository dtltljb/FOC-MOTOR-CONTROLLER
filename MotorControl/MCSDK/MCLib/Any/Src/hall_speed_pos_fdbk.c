/**
  ******************************************************************************
  * @file    hall_speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features of
  *          the Hall Speed & Position Feedback component of the Motor Control SDK.
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
#include "speed_pos_fdbk.h"
#include "hall_speed_pos_fdbk.h"
#include "mc_irq_handler.h"

#include "mc_library_isr_priority_conf.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/**
 * @defgroup hall_speed_pos_fdbk Hall Speed & Position Feedback
 *
 * @brief Hall Sensor based Speed & Position Feedback implementation
 *
  * This component is used in applications controlling a motor equipped with Hall effect sensors.
  *
  * This component uses the output of two Hall effects sensors to provide a measure of the speed
  * and the position of the rotor of the motor.
  *
 * @todo Document the Hall Speed & Position Feedback "module".
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/

/* Lower threshold to reques a decrease of clock prescaler */
#define LOW_RES_THRESHOLD   ((uint16_t)0x5500u)

#define HALL_COUNTER_RESET  ((uint16_t) 0u)

#define S16_120_PHASE_SHIFT (int16_t)(65536/3)
#define S16_60_PHASE_SHIFT  (int16_t)(65536/6)

#define STATE_0 (uint8_t)0
#define STATE_1 (uint8_t)1
#define STATE_2 (uint8_t)2
#define STATE_3 (uint8_t)3
#define STATE_4 (uint8_t)4
#define STATE_5 (uint8_t)5
#define STATE_6 (uint8_t)6
#define STATE_7 (uint8_t)7

#define NEGATIVE          (int8_t)-1
#define POSITIVE          (int8_t)1
#define NEGATIVE_SWAP     (int8_t)-2
#define POSITIVE_SWAP     (int8_t)2

/* With digit-per-PWM unit (here 2*PI rad = 0xFFFF): */
#define HALL_MAX_PSEUDO_SPEED        ((int16_t)0x7FFF)

#define CCER_CC1E_Set               ((uint16_t)0x0001)
#define CCER_CC1E_Reset             ((uint16_t)0xFFFE)

static void HALL_Init_Electrical_Angle(HALL_Handle_t *pHandle);
static int16_t HALL_CalcAvrgElSpeedDpp(HALL_Handle_t *pHandle);

/**
  * @brief  It initializes the hardware peripherals (TIMx, GPIO and NVIC) 
            required for the speed position sensor management using HALL 
            sensors.
  * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
  * @retval none
  */
void HALL_Init(HALL_Handle_t *pHandle)
{
  TIM_TypeDef* TIMx = pHandle->TIMx;
  
  uint16_t hMinReliableElSpeed01Hz = pHandle->_Super.hMinReliableMecSpeed01Hz *
    pHandle->_Super.bElToMecRatio;
  uint16_t hMaxReliableElSpeed01Hz = pHandle->_Super.hMaxReliableMecSpeed01Hz *
     pHandle->_Super.bElToMecRatio;
  uint8_t bSpeedBufferSize;
  uint8_t bIndex;

  /* Adjustment factor: minimum measurable speed is x time less than the minimum
  reliable speed */
  hMinReliableElSpeed01Hz /= 4u;
  
  /* Adjustment factor: maximum measurable speed is x time greather than the
  maximum reliable speed */
  hMaxReliableElSpeed01Hz *= 2u;
  
  pHandle->OvfFreq = (uint16_t)(pHandle->TIMClockFreq / 65536u);
  
  /* SW Init */
  if (hMinReliableElSpeed01Hz == 0u)
  {
    /* Set fixed to 150 ms */
    pHandle->HallTimeout = 150u; 
  }
  else
  {
    /* Set accordingly the min reliable speed */
    /* 10000 comes from mS and 01Hz 
    * 6 comes from the fact that sensors are toggling each 60 deg  */
    pHandle->HallTimeout = 10000u / (6u * hMinReliableElSpeed01Hz);
  }
    
  /* Compute the prescaler to the closet value of the TimeOut (in mS )*/
  pHandle->HALLMaxRatio = (pHandle->HallTimeout * pHandle->OvfFreq) / 1000 ;
   
  /* Align MaxPeriod to a multiple of Overflow.*/
  pHandle->MaxPeriod = (pHandle->HALLMaxRatio) * 65536uL;
 
  pHandle->SatSpeed = hMaxReliableElSpeed01Hz;
  
  pHandle->PseudoFreqConv = ((pHandle->TIMClockFreq / 6u) 
                     / (pHandle->_Super.hMeasurementFrequency)) * 65536u;
    
  pHandle->MinPeriod = ((10u * pHandle->TIMClockFreq) / 6u) 
                     / hMaxReliableElSpeed01Hz;
    
  pHandle->PWMNbrPSamplingFreq = (pHandle->_Super.hMeasurementFrequency / 
                pHandle->SpeedSamplingFreqHz) - 1u;
  
  /* Reset speed reliability */
  pHandle->SensorIsReliable = true;
  
  /* Force the TIMx prescaler with immediate access (gen update event) 
  */ 
  LL_TIM_SetPrescaler (TIMx, pHandle->HALLMaxRatio);
  LL_TIM_GenerateEvent_UPDATE (TIMx);
  
  
  /* Clear the TIMx's pending flags */
   LL_TIM_WriteReg(TIMx, SR, 0);

  /* Selected input capture and Update (overflow) events generate interrupt */

  /* Source of Update event is only counter overflow/underflow */
  LL_TIM_SetUpdateSource (TIMx, LL_TIM_UPDATESOURCE_COUNTER);
  
  LL_TIM_EnableIT_CC1 (TIMx);
  LL_TIM_EnableIT_UPDATE (TIMx);
  LL_TIM_SetCounter (TIMx, HALL_COUNTER_RESET);
 
  LL_TIM_CC_EnableChannel  (TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableCounter (TIMx);
  
  
  /* Erase speed buffer */
  bSpeedBufferSize = pHandle->SpeedBufferSize;
  
  for (bIndex = 0u; bIndex < bSpeedBufferSize; bIndex++)
  {
    pHandle->SensorSpeed[bIndex]  = 0;
  }
}

/**
* @brief  Clear software FIFO where are "pushed" latest speed information
*         This function must be called before starting the motor to initialize
*	        the speed measurement process.
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component*
* @retval none
*/
void HALL_Clear(HALL_Handle_t *pHandle)
{
  TIM_TypeDef* TIMx = pHandle->TIMx;
  
  /* Mask interrupts to insure a clean intialization */
  LL_TIM_DisableIT_CC1 (TIMx);
  
  pHandle->RatioDec = false;
  pHandle->RatioInc = false;
  
  /* Reset speed reliability */
  pHandle->SensorIsReliable = true;
  
  /* Acceleration measurement not implemented.*/
  pHandle->_Super.hMecAccel01HzP = 0;
  
  pHandle->FirstCapt = 0u;
  pHandle->BufferFilled = 0u;
  pHandle->OVFCounter = 0u;  

  pHandle->CompSpeed = 0;
  pHandle->ElSpeedSum = 0;
    
  pHandle->Direction = POSITIVE;
    
  /* Initialize speed buffer index */
  pHandle->SpeedFIFOIdx = 0u;
  
  /* Clear new speed acquisitions flag */
  pHandle->NewSpeedAcquisition = 0;
  
  /* Re-initialize partly the timer */
  LL_TIM_SetPrescaler (TIMx, pHandle->HALLMaxRatio);
  
  LL_TIM_SetCounter (TIMx, HALL_COUNTER_RESET);
  
  LL_TIM_EnableCounter (TIMx);
  
  LL_TIM_EnableIT_CC1 (TIMx);
  
  HALL_Init_Electrical_Angle(pHandle);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
* @brief  Update the rotor electrical angle integrating the last measured 
*         instantaneous electrical speed express in dpp.
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval int16_t Measured electrical angle in s16degree format.
*/
int16_t HALL_CalcElAngle(HALL_Handle_t *pHandle)
{
    
  if (pHandle->_Super.hElSpeedDpp != HALL_MAX_PSEUDO_SPEED)
  {
    pHandle->MeasuredElAngle += pHandle->_Super.hElSpeedDpp;
    pHandle->TargetElAngle += pHandle->_Super.hElSpeedDpp;
    pHandle->_Super.hElAngle += pHandle->_Super.hElSpeedDpp + pHandle->CompSpeed;
    pHandle->PrevRotorFreq = pHandle->_Super.hElSpeedDpp;
  }
  else
  {
    pHandle->_Super.hElAngle += pHandle->PrevRotorFreq;
  }
      
  return pHandle->_Super.hElAngle;
}


/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed.
  *         This method compute and store rotor istantaneous el speed (express 
  *         in dpp considering the measurement frequency) in order to provide it
  *         to HALL_CalcElAngle function and SPD_GetElAngle. 
  *         Then compute rotor average el speed (express in dpp considering the 
  *         measurement frequency) based on the buffer filled by IRQ, then - as 
  *         a consequence - compute, store and return - through parameter 
  *         hMecSpeed01Hz - the rotor average mech speed, expressed in 01Hz.
  *         Then check, store and return the reliability state of
  *         the sensor; in this function the reliability is measured with 
  *         reference to specific parameters of the derived
  *         sensor (HALL) through internal variables managed by IRQ.
  * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
  * @param  hMecSpeed01Hz pointer to int16_t, used to return the rotor average
  *         mechanical speed (01Hz)
  * @retval true = sensor information is reliable
  *         false = sensor information is not reliable
  */
bool HALL_CalcAvrgMecSpeed01Hz(HALL_Handle_t *pHandle, int16_t *hMecSpeed01Hz)
{
  TIM_TypeDef* TIMx = pHandle->TIMx;
  int16_t SpeedMeasAux;
  bool bReliability;
    
    /* Computing the rotor istantaneous el speed */
    SpeedMeasAux = pHandle->CurrentSpeed;
    
    if(pHandle->SensorIsReliable)
    {
      /* No errors have been detected during rotor speed information 
      extrapolation */
      if ( LL_TIM_GetPrescaler (TIMx) >= pHandle->HALLMaxRatio )
      {                           
        /* At start-up or very low freq */
        /* Based on current prescaler value only */
        pHandle->_Super.hElSpeedDpp = 0;
        *hMecSpeed01Hz = 0;
      }
      else
      {
        pHandle->_Super.hElSpeedDpp = SpeedMeasAux;
        if( SpeedMeasAux == 0 ) 
        {
          /* Speed is too low */          
          *hMecSpeed01Hz = 0;
        }
        else
        {  
          /* Check if speed is not to fast */
          if ( SpeedMeasAux != HALL_MAX_PSEUDO_SPEED )
          {                      
            #ifdef HALL_MTPA
            {
              pHandle->CompSpeed = 0;
            }
            #else
            {
              pHandle->TargetElAngle = pHandle->MeasuredElAngle;
              pHandle->DeltaAngle = pHandle->MeasuredElAngle - pHandle->_Super.hElAngle;
              pHandle->CompSpeed = (int16_t)
                ((int32_t)(pHandle->DeltaAngle)/
                 (int32_t)(pHandle->PWMNbrPSamplingFreq));
            }
            #endif
            
            *hMecSpeed01Hz = HALL_CalcAvrgElSpeedDpp(pHandle);
                        
            /* Converto el_dpp to Mec01Hz */
            *hMecSpeed01Hz = (int16_t)((*hMecSpeed01Hz * 
                              (int32_t)pHandle->_Super.hMeasurementFrequency * 10)/
                              (65536 * (int32_t)pHandle->_Super.bElToMecRatio));
            
          }
          else
          {
            *hMecSpeed01Hz = (int16_t)pHandle->SatSpeed;
          }
        }
      }
      bReliability = SPD_IsMecSpeedReliable(&pHandle->_Super, hMecSpeed01Hz);
    }
    else 
    {
       bReliability = false;
       pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
       /* If speed is not reliable the El and Mec speed is set to 0 */
       pHandle->_Super.hElSpeedDpp = 0;
       *hMecSpeed01Hz = 0; 
    }

  pHandle->_Super.hAvrMecSpeed01Hz = *hMecSpeed01Hz;
  
  return (bReliability);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
* @brief  Example of private method of the class HALL to implement an MC IRQ function
*         to be called when TIMx capture event occurs
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval none
*/
void * HALL_TIMx_CC_IRQHandler(void *pHandleVoid)
{
  HALL_Handle_t *pHandle = (HALL_Handle_t *) pHandleVoid; 
  TIM_TypeDef* TIMx = pHandle->TIMx;
  uint8_t bPrevHallState;
  uint32_t wCaptBuf;
  uint16_t hPrscBuf;
  uint16_t hHighSpeedCapture;

  if (pHandle->SensorIsReliable)
  {
    /* A capture event generated this interrupt */
    bPrevHallState = pHandle->HallState;

    if (pHandle->SensorPlacement == DEGREES_120)
    {
      pHandle->HallState  = LL_GPIO_IsInputPinSet(pHandle->H3Port, pHandle->H3Pin)<<2
                            | LL_GPIO_IsInputPinSet(pHandle->H2Port, pHandle->H2Pin)<<1
                            | LL_GPIO_IsInputPinSet(pHandle->H1Port, pHandle->H1Pin);
    }
    else {
      pHandle->HallState  = (LL_GPIO_IsInputPinSet(pHandle->H2Port, pHandle->H2Pin)^1)<<2
                            | LL_GPIO_IsInputPinSet(pHandle->H3Port, pHandle->H3Pin)<<1
                            | LL_GPIO_IsInputPinSet(pHandle->H1Port, pHandle->H1Pin);
    }

    switch(pHandle->HallState)
    {
      case STATE_5:
        if (bPrevHallState == STATE_4)
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = pHandle->PhaseShift;
        }
        else if (bPrevHallState == STATE_1)
        {
          pHandle->Direction = NEGATIVE;
            pHandle->MeasuredElAngle = (int16_t)(pHandle->PhaseShift+S16_60_PHASE_SHIFT);
        }
        else
        {
        }
        break;

      case STATE_1:
        if (bPrevHallState == STATE_5)
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = pHandle->PhaseShift+S16_60_PHASE_SHIFT;
        }
        else if (bPrevHallState == STATE_3)
        {
          pHandle->Direction = NEGATIVE;
            pHandle->MeasuredElAngle = (int16_t)(pHandle->PhaseShift+S16_120_PHASE_SHIFT);
        }
        else
        {
        }
        break;

      case STATE_3:
        if (bPrevHallState == STATE_1)
        {
          pHandle->Direction = POSITIVE;
            pHandle->MeasuredElAngle =(int16_t)(pHandle->PhaseShift + S16_120_PHASE_SHIFT);
        }
        else if (bPrevHallState == STATE_2)
        {
          pHandle->Direction = NEGATIVE;
            pHandle->MeasuredElAngle = (int16_t)(pHandle->PhaseShift+S16_120_PHASE_SHIFT+
                  S16_60_PHASE_SHIFT);
        }
        else
        {
        }

        break;

      case STATE_2:
        if (bPrevHallState == STATE_3)
        {
          pHandle->Direction = POSITIVE;
            pHandle->MeasuredElAngle =(int16_t)(pHandle->PhaseShift + S16_120_PHASE_SHIFT
                  + S16_60_PHASE_SHIFT);
        }
        else if (bPrevHallState == STATE_6)
        {
          pHandle->Direction = NEGATIVE;
            pHandle->MeasuredElAngle = (int16_t)(pHandle->PhaseShift-S16_120_PHASE_SHIFT);
        }
        else
        {
        }
        break;

      case STATE_6:
        if (bPrevHallState == STATE_2)
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle =(int16_t)(pHandle->PhaseShift - S16_120_PHASE_SHIFT); 
        }
        else if (bPrevHallState == STATE_4)
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle =(int16_t)(pHandle->PhaseShift - S16_60_PHASE_SHIFT);  
        }
        else
        {
        }
        break;

      case STATE_4:
        if (bPrevHallState == STATE_6)
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle =(int16_t)(pHandle->PhaseShift - S16_60_PHASE_SHIFT); 
        }
        else if (bPrevHallState == STATE_5)
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle =(int16_t)(pHandle->PhaseShift);  
        }
        else
        {
        }
        break;

      default:
        /* Bad hall sensor configutarion so update the speed reliability */
        pHandle->SensorIsReliable = false;

        break;
    }


#ifdef HALL_MTPA
    {
      pHandle->_Super.hElAngle = pHandle->MeasuredElAngle;
    }
#endif

    /* Discard first capture */
    if (pHandle->FirstCapt == 0u)
    {
      pHandle->FirstCapt++;
      LL_TIM_IC_GetCaptureCH1(TIMx);
    }
    else
    {
      /* used to validate the average speed measurement */
      if (pHandle->BufferFilled < pHandle->SpeedBufferSize)
      {
        pHandle->BufferFilled++;
      }

      /* Store the latest speed acquisition */
      hHighSpeedCapture = LL_TIM_IC_GetCaptureCH1(TIMx);
      wCaptBuf = (uint32_t)hHighSpeedCapture;
      hPrscBuf =  LL_TIM_GetPrescaler (TIMx);

      /* Add the numbers of overflow to the counter */
      wCaptBuf += (uint32_t)pHandle->OVFCounter * 0x10000uL;

      if (pHandle->OVFCounter != 0u)
      {
        /* Adjust the capture using prescaler */
        uint16_t hAux;
        hAux = hPrscBuf + 1u;
        wCaptBuf *= hAux;

        if (pHandle->RatioInc)
        {
          pHandle->RatioInc = false;	/* Previous capture caused overflow */
          /* Don't change prescaler (delay due to preload/update mechanism) */
        }
        else
        {
          if ( LL_TIM_GetPrescaler (TIMx) < pHandle->HALLMaxRatio) /* Avoid OVF w/ very low freq */
          {
            LL_TIM_SetPrescaler (TIMx,LL_TIM_GetPrescaler (TIMx)+1); /* To avoid OVF during speed decrease */
            pHandle->RatioInc = true;	  /* new prsc value updated at next capture only */
          }
        }
      }
      else
      {
        /* If prsc preload reduced in last capture, store current register + 1 */
        if (pHandle->RatioDec)  /* and don't decrease it again */
        {
          /* Adjust the capture using prescaler */
          uint16_t hAux;
          hAux = hPrscBuf + 2u;
          wCaptBuf *= hAux;

          pHandle->RatioDec = false;
        }
        else  /* If prescaler was not modified on previous capture */
        {
          /* Adjust the capture using prescaler */
          uint16_t hAux = hPrscBuf + 1u;
          wCaptBuf *= hAux;

          if (hHighSpeedCapture < LOW_RES_THRESHOLD)/* If capture range correct */
          {
            if(LL_TIM_GetPrescaler (TIMx) > 0u) /* or prescaler cannot be further reduced */
            {
              LL_TIM_SetPrescaler (TIMx, LL_TIM_GetPrescaler (TIMx)-1);	/* Increase accuracy by decreasing prsc */
              /* Avoid decrementing again in next capt.(register preload delay) */
              pHandle->RatioDec = true;
            }
          }
        }
      }

#if 0
      /* Store into the buffer */
      /* Null Speed is detected, erase the buffer */
      if( wCaptBuf > pHandle->MaxPeriod) {
        uint8_t bIndex;
        for (bIndex = 0u; bIndex < pHandle->SpeedBufferSize; bIndex++)
        {
          pHandle->SensorSpeed[bIndex]  = 0;
        }
        pHandle->BufferFilled = 0 ;
        pHandle->SpeedFIFOSetIdx=1;
        pHandle->SpeedFIFOGetIdx =0;
        /* Indicate new speed acquisitions */
        pHandle->NewSpeedAcquisition = 1;
        pHandle->ElSpeedSum =0;
      }
      /* Filtering to fast speed... could be a glitch  ? */
      /* the HALL_MAX_PSEUDO_SPEED is temporary in the buffer, and never included in average computation*/
      else
#endif
        if ( wCaptBuf < pHandle->MinPeriod )
        {
          pHandle->CurrentSpeed = HALL_MAX_PSEUDO_SPEED;
          pHandle->NewSpeedAcquisition = 0;
        }
        else
        {
          pHandle->ElSpeedSum -= pHandle->SensorSpeed[pHandle->SpeedFIFOIdx]; /* value we gonna removed from the accumulator */
          if (wCaptBuf >= pHandle->MaxPeriod)
          {
            pHandle->SensorSpeed[pHandle->SpeedFIFOIdx]  = 0;
          }
          else {
            pHandle->SensorSpeed[pHandle->SpeedFIFOIdx] = (int16_t) (pHandle->PseudoFreqConv/wCaptBuf);
            pHandle->SensorSpeed[pHandle->SpeedFIFOIdx]*= pHandle->Direction;
            pHandle->ElSpeedSum += pHandle->SensorSpeed[pHandle->SpeedFIFOIdx];
          }
          /* Update pointers to speed buffer */
          pHandle->CurrentSpeed = pHandle->SensorSpeed[pHandle->SpeedFIFOIdx];
          pHandle->SpeedFIFOIdx++;
          if (pHandle->SpeedFIFOIdx == pHandle->SpeedBufferSize)
          {
            pHandle->SpeedFIFOIdx = 0u;
          }
          /* Indicate new speed acquisitions */
          pHandle->NewSpeedAcquisition = 1;
        }
      /* Reset the number of overflow occurred */
      pHandle->OVFCounter = 0u;
    }
  }
  return MC_NULL;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
* @brief  Example of private method of the class HALL to implement an MC IRQ function
*         to be called when TIMx update event occurs
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval none
*/
void * HALL_TIMx_UP_IRQHandler(void *pHandleVoid)
{
  HALL_Handle_t *pHandle = (HALL_Handle_t *) pHandleVoid;
  TIM_TypeDef* TIMx = pHandle->TIMx;

  if (pHandle->SensorIsReliable)
  {
    uint16_t hMaxTimerOverflow;
    /* an update event occured for this interrupt request generation */
    pHandle->OVFCounter++;

    hMaxTimerOverflow = (uint16_t)(((uint32_t)pHandle->HallTimeout * pHandle->OvfFreq)
      /((LL_TIM_GetPrescaler (TIMx)+1) * 1000u));
    if (pHandle->OVFCounter >= hMaxTimerOverflow)
    {
        /* Set rotor speed to zero */
      pHandle->_Super.hElSpeedDpp = 0;

      /* Reset the electrical angle according the hall sensor configuration */
      HALL_Init_Electrical_Angle(pHandle);

      /* Reset the overflow counter */
      pHandle->OVFCounter = 0u;


#if 1
      /* Reset the SensorSpeed buffer*/
        uint8_t bIndex;
        for (bIndex = 0u; bIndex < pHandle->SpeedBufferSize; bIndex++)
          {
            pHandle->SensorSpeed[bIndex]  = 0;
          }
        pHandle->BufferFilled = 0 ;
        pHandle->CurrentSpeed = 0;
        pHandle->SpeedFIFOIdx=1;
        pHandle->ElSpeedSum =0;
#else
        pHandle->ElSpeedSum -= pHandle->SensorSpeed[pHandle->SpeedFIFOIdx];
        pHandle->SensorSpeed[pHandle->SpeedFIFOSetIdx]  = 0;
        pHandle->CurrentSpeed = 0;
        pHandle->SpeedFIFOIdx++;
        if (pHandle->SpeedFIFOIdx == pHandle->SpeedBufferSize)
        {
          pHandle->SpeedFIFOIdx = 0u;
        }
#endif
    }
  }
  return MC_NULL;
}

/**
* @brief  Compute and returns the average rotor electrical speed express in dpp
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval int16_t the average rotor electrical speed express in dpp
*/
static int16_t HALL_CalcAvrgElSpeedDpp(HALL_Handle_t *pHandle)
{
  
  if (pHandle->NewSpeedAcquisition == 1)
  {
 
    if (pHandle->BufferFilled < pHandle->SpeedBufferSize)
    {
      pHandle->AvrElSpeedDpp = (int16_t)  pHandle->CurrentSpeed;
    }
    else
    {
       pHandle->AvrElSpeedDpp = (int16_t)(pHandle->ElSpeedSum / (int32_t)(pHandle->SpeedBufferSize));        /* Average value */
    }
           
    /* Clear new speed acquisitions flag */
    pHandle->NewSpeedAcquisition = 0;
  }
  
  return pHandle->AvrElSpeedDpp;
}

/**
* @brief  Read the logic level of the three Hall sensor and individuates in this 
*         way the position of the rotor (+/- 30�). Electrical angle is then 
*         initialized.
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval none
*/
static void HALL_Init_Electrical_Angle(HALL_Handle_t *pHandle)
{
 
    if (pHandle->SensorPlacement == DEGREES_120) 
    {
      pHandle->HallState  = LL_GPIO_IsInputPinSet(pHandle->H3Port, pHandle->H3Pin)<<2
                            | LL_GPIO_IsInputPinSet(pHandle->H2Port, pHandle->H2Pin)<<1
                            | LL_GPIO_IsInputPinSet(pHandle->H1Port, pHandle->H1Pin);
    }
    else {
      pHandle->HallState  = (LL_GPIO_IsInputPinSet(pHandle->H2Port, pHandle->H2Pin)^1)<<2
                            | LL_GPIO_IsInputPinSet(pHandle->H3Port, pHandle->H3Pin)<<1
                            | LL_GPIO_IsInputPinSet(pHandle->H1Port, pHandle->H1Pin);
    }
 
  switch(pHandle->HallState)
    {
    case STATE_5:
      pHandle->_Super.hElAngle = (int16_t)(pHandle->PhaseShift+S16_60_PHASE_SHIFT/2);
      break;
    case STATE_1:
      pHandle->_Super.hElAngle =(int16_t)(pHandle->PhaseShift+S16_60_PHASE_SHIFT+
                               S16_60_PHASE_SHIFT/2);
      break;
    case STATE_3:
      pHandle->_Super.hElAngle =(int16_t)(pHandle->PhaseShift+S16_120_PHASE_SHIFT+
                               S16_60_PHASE_SHIFT/2);      
      break;
    case STATE_2:
      pHandle->_Super.hElAngle =(int16_t)(pHandle->PhaseShift-S16_120_PHASE_SHIFT-
                               S16_60_PHASE_SHIFT/2);      
      break;
    case STATE_6:
      pHandle->_Super.hElAngle =(int16_t)(pHandle->PhaseShift-S16_60_PHASE_SHIFT-
                               S16_60_PHASE_SHIFT/2);          
      break;
    case STATE_4:
      pHandle->_Super.hElAngle =(int16_t)(pHandle->PhaseShift-S16_60_PHASE_SHIFT/2);          
      break;    
    default:
      /* Bad hall sensor configutarion so update the speed reliability */
      pHandle->SensorIsReliable = false;
      break;
    }
  
  /* Initialize the measured angle */
  pHandle->MeasuredElAngle = pHandle->_Super.hElAngle;
  
}

/**
  * @brief  It could be used to set istantaneous information on rotor mechanical
  *         angle.
  *         Note: Mechanical angle management is not implemented in this 
  *         version of Hall sensor class.
  * @param  pHandle pointer on related component instance
  * @param  hMecAngle istantaneous measure of rotor mechanical angle
  * @retval none
  */
void HALL_SetMecAngle(HALL_Handle_t *pHandle, int16_t hMecAngle)
{
}


/**
  * @}
  */

/**
  * @}
  */

/** @} */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
