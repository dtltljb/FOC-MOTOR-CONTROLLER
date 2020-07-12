/**
  ******************************************************************************
  * @file    r3_2_f30x_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement current sensor
  *          class to be stantiated when the three shunts current sensing
  *          topology is used. It is specifically designed for STM32F30X
  *          microcontrollers and implements the successive sampling of two motor
  *          current using shared ADC.
  *           + MCU peripheral and handle initialization function
  *           + three shunt current sesnsing
  *           + space vector modulation function
  *           + ADC sampling function
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
#include "pwm_curr_fdbk.h"
#include "r3_2_f30x_pwm_curr_fdbk.h"
#include "mc_irq_handler.h"

#include "mc_library_isr_priority_conf.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/**
 * @defgroup r3_2_f30x_pwm_curr_fdbk R3 2 ADCs F30x PWM & Current Feedback
 *
 * @brief STM32F3, Shared Resources, 3-Shunt PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F3 MCU, using a three
 * shunt resistors current sensing topology and 2 ADC peripherals to acquire the current
 * values.
 *
 * It is designed to be used in applications that drive two motors in which case two instances of
 * the component are used that share the 2 ADCs.
 *
 * @todo: TODO: complete documentation.
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/
#define TIMxCCER_MASK_CH123          (LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N |\
                                     LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N)
#define NB_CONVERSIONS 16u
#define CCMR2_CH4_DISABLE 0x8FFFu   //
#define CCMR2_CH4_PWM1    0x6000u   //
#define CCMR2_CH4_PWM2    0x7000u
#define OPAMP_CSR_DEFAULT_MASK  ((uint32_t)0xFFFFFF93u)

/* Private typedef -----------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/

/* These function overloads the TIM_BDTRConfig and TIM_BDTRStructInit
   of the standard library */
static void R3_2_F30X_TIMxInit(TIM_TypeDef* TIMx, PWMC_Handle_t *pHdl);
static uint16_t R3_2_F30X_WriteTIMRegisters(PWMC_Handle_t *pHdl);
static void R3_2_F30X_HFCurrentsCalibrationAB(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents);
static void R3_2_F30X_HFCurrentsCalibrationC(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents);
static void R3_2_F30X_SetAOReferenceVoltage(uint32_t DAC_Channel, uint16_t hDACVref);
static uint32_t R3_2_F30X_ADC_InjectedChannelConfig(ADC_TypeDef* ADCx,
                                                    uint8_t ADC_Channel,
                                                    uint8_t Rank,
                                                    uint8_t ADC_SampleTime,
                                                    uint8_t SequencerLength,
                                                    uint16_t ADC_ExternalTriggerInjectedPolarity,
                                                    uint16_t ADC_ExternalTriggerInjected);
/**
  * @brief  It initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading
  *         in three shunt topology using STM32F30X and shared ADC
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void R3_2_F30X_Init(PWMC_R3_2_F3_Handle_t *pHandle)
{
  pR3_2_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;
  COMP_TypeDef* COMP_OCPAx = pHandle->pParams_str->wCompOCPASelection;
  COMP_TypeDef* COMP_OCPBx = pHandle->pParams_str->wCompOCPBSelection;
  COMP_TypeDef* COMP_OCPCx = pHandle->pParams_str->wCompOCPCSelection;
  COMP_TypeDef* COMP_OVPx = pHandle->pParams_str->wCompOVPSelection;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  uint16_t ADC_ExternalTriggerInjected;

  if ((uint32_t)pHandle == (uint32_t)&pHandle->_Super)
  {
    R3_2_F30X_TIMxInit(TIMx, &pHandle->_Super);

    if(TIMx == TIM1)
    {
      /* TIM1 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM1_STOP);
    }
    else
    {
      /* TIM8 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM8_STOP);
    }

    if (pDOPAMPParams_str)
    {
      LL_OPAMP_Enable(OPAMP1);
      LL_OPAMP_Enable(OPAMP3);
    }

    /* Over current protection phase A */
    if (COMP_OCPAx)
    {
      /* Inverting input*/
      if (pHandle->pParams_str->bCompOCPAInvInput_MODE != EXT_MODE)
      {
        if (LL_COMP_GetInputMinus(COMP_OCPAx) == LL_COMP_INPUT_MINUS_DAC1_CH1)
        {
          R3_2_F30X_SetAOReferenceVoltage(LL_DAC_CHANNEL_1, (uint16_t)(pHandle->pParams_str->hDAC_OCP_Threshold));
        }
        else if (LL_COMP_GetInputMinus(COMP_OCPAx) == LL_COMP_INPUT_MINUS_DAC1_CH2)
        {
          R3_2_F30X_SetAOReferenceVoltage(LL_DAC_CHANNEL_2, (uint16_t)(pHandle->pParams_str->hDAC_OCP_Threshold));
        }
        else
        {
        }
      }

      /* Wait to stabilize DAC voltage */
      volatile uint16_t waittime = 0u;
      for(waittime=0u;waittime<1000u;waittime++)
      {}

      /* Output */
      LL_COMP_Enable (COMP_OCPAx);
      LL_COMP_Lock(COMP_OCPAx);
    }

    /* Over current protection phase B */
    if (COMP_OCPBx)
    {
      LL_COMP_Enable (COMP_OCPBx);
      LL_COMP_Lock(COMP_OCPBx);
    }

    /* Over current protection phase C */
    if (COMP_OCPCx)
    {
      LL_COMP_Enable (COMP_OCPCx);
      LL_COMP_Lock(COMP_OCPCx);
    }

    /* Over voltage protection */
    if (COMP_OVPx)
    {
      /* Inverting input*/
      if (pHandle->pParams_str->bCompOVPInvInput_MODE != EXT_MODE)
      {
        if (LL_COMP_GetInputMinus(COMP_OVPx) == LL_COMP_INPUT_MINUS_DAC1_CH1)
        {
          R3_2_F30X_SetAOReferenceVoltage(LL_DAC_CHANNEL_1, (uint16_t)(pHandle->pParams_str->hDAC_OVP_Threshold));
        }
        else if (LL_COMP_GetInputMinus(COMP_OVPx) == LL_COMP_INPUT_MINUS_DAC1_CH2)
        {
          R3_2_F30X_SetAOReferenceVoltage(LL_DAC_CHANNEL_2, (uint16_t)(pHandle->pParams_str->hDAC_OVP_Threshold));
        }
        else
        {
        }
      }

      /* Wait to stabilize DAC voltage */
      volatile uint16_t waittime = 0u;
      for(waittime=0u;waittime<1000u;waittime++)
      {}

      /* Output */
      LL_COMP_Enable (COMP_OVPx);
      LL_COMP_Lock(COMP_OVPx);
    }

    if (pHandle->_Super.bMotor == M1)
    {
      LL_ADC_EnableInternalRegulator(pHandle->pParams_str->ADCx_1);
      LL_ADC_EnableInternalRegulator(pHandle->pParams_str->ADCx_2);

      /* Wait for Regulator Startup time, once for both */
      uint16_t waittime = 0u;
      for(waittime=0u;waittime<65000u;waittime++)
      {}

      LL_ADC_StartCalibration(pHandle->pParams_str->ADCx_1,LL_ADC_SINGLE_ENDED);
      while (LL_ADC_IsCalibrationOnGoing(pHandle->pParams_str->ADCx_1))
      {}

      LL_ADC_StartCalibration(pHandle->pParams_str->ADCx_2,LL_ADC_SINGLE_ENDED);
      while (LL_ADC_IsCalibrationOnGoing(pHandle->pParams_str->ADCx_2))
      {}

      if ((pHandle->pParams_str->regconvADCx != pHandle->pParams_str->ADCx_1) &&
          (pHandle->pParams_str->regconvADCx != pHandle->pParams_str->ADCx_2))
      {
        {
          LL_ADC_EnableInternalRegulator(pHandle->pParams_str->regconvADCx);

          /* Wait for Regulator Startup time, once for both */
          uint16_t waittime = 0u;
          for(waittime=0u;waittime<65000u;waittime++)
          {}

          LL_ADC_StartCalibration(pHandle->pParams_str->regconvADCx, LL_ADC_SINGLE_ENDED);
          while (LL_ADC_IsCalibrationOnGoing(pHandle->pParams_str->regconvADCx))
          {}
        }
      }

      /* Enable ADCx_1 and ADCx_2 */
      LL_ADC_Enable(pHandle->pParams_str->ADCx_1);
      LL_ADC_Enable(pHandle->pParams_str->ADCx_2);
    }
    else
    {
      /* already done by the first motor */
    }

    if ((pHandle->pParams_str->regconvADCx != pHandle->pParams_str->ADCx_1) &&
        (pHandle->pParams_str->regconvADCx != pHandle->pParams_str->ADCx_2))
    {
      LL_ADC_Enable(pHandle->pParams_str->regconvADCx);
    }
    LL_ADC_REG_SetSequencerLength(pHandle->pParams_str->regconvADCx,LL_ADC_REG_SEQ_SCAN_DISABLE);

    if (TIMx == TIM1)
    {
      ADC_ExternalTriggerInjected = (uint16_t)LL_ADC_INJ_TRIG_EXT_TIM1_TRGO;
    }
    else
    {
      ADC_ExternalTriggerInjected = (uint16_t)LL_ADC_INJ_TRIG_EXT_TIM8_TRGO;
    }


    if (pHandle->pParams_str->pOPAMPParams)
    {
      /* ADC channel config when using internal OPAMP */
      pHandle->wADC_JSQR_phA = R3_2_F30X_ADC_InjectedChannelConfig(pHandle->pParams_str->ADCx_1,
                                                                   ADC_CHANNEL_3,
                                                                   1u,
                                                                   pHandle->pParams_str->b_IaSamplingTime,
                                                                   1u,
                                                                   LL_ADC_INJ_TRIG_EXT_RISING,
                                                                   ADC_ExternalTriggerInjected);
      pHandle->wADC_JSQR_phB = R3_2_F30X_ADC_InjectedChannelConfig(pHandle->pParams_str->ADCx_2,
                                                                   ADC_CHANNEL_1,
                                                                   1u,
                                                                   pHandle->pParams_str->b_IbSamplingTime,
                                                                   1u,
                                                                   LL_ADC_INJ_TRIG_EXT_RISING,
                                                                   ADC_ExternalTriggerInjected);
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phB;
    }
    else
    {
      /* ADC channel config when using external OPAMP */
      pHandle->wADC_JSQR_phA = R3_2_F30X_ADC_InjectedChannelConfig(pHandle->pParams_str->ADCx_1, pHandle->pParams_str->bIaChannel, 1u, pHandle->pParams_str->b_IaSamplingTime, 1u, LL_ADC_INJ_TRIG_EXT_RISING, ADC_ExternalTriggerInjected);
      pHandle->wADC_JSQR_phB = R3_2_F30X_ADC_InjectedChannelConfig(pHandle->pParams_str->ADCx_2, pHandle->pParams_str->bIbChannel, 1u, pHandle->pParams_str->b_IbSamplingTime, 1u, LL_ADC_INJ_TRIG_EXT_RISING, ADC_ExternalTriggerInjected);
      pHandle->wADC_JSQR_phC = R3_2_F30X_ADC_InjectedChannelConfig(pHandle->pParams_str->ADCx_2, pHandle->pParams_str->bIcChannel, 1u, pHandle->pParams_str->b_IcSamplingTime, 1u, LL_ADC_INJ_TRIG_EXT_RISING, ADC_ExternalTriggerInjected);
      R3_2_F30X_ADC_InjectedChannelConfig(pHandle->pParams_str->ADCx_2, pHandle->pParams_str->bIaChannel, 1u, pHandle->pParams_str->b_IaSamplingTime, 1u, LL_ADC_INJ_TRIG_EXT_RISING, ADC_ExternalTriggerInjected);
      R3_2_F30X_ADC_InjectedChannelConfig(pHandle->pParams_str->ADCx_1, pHandle->pParams_str->bIbChannel, 1u, pHandle->pParams_str->b_IbSamplingTime, 1u, LL_ADC_INJ_TRIG_EXT_RISING, ADC_ExternalTriggerInjected);
      R3_2_F30X_ADC_InjectedChannelConfig(pHandle->pParams_str->ADCx_1, pHandle->pParams_str->bIcChannel, 1u, pHandle->pParams_str->b_IcSamplingTime, 1u, LL_ADC_INJ_TRIG_EXT_RISING, ADC_ExternalTriggerInjected);
    }

    LL_ADC_INJ_StopConversion(pHandle->pParams_str->ADCx_1);
    LL_ADC_INJ_StopConversion(pHandle->pParams_str->ADCx_2);
    pHandle->pParams_str->ADCx_1->JSQR = R3_2_F30X_ADC_InjectedChannelConfig(pHandle->pParams_str->ADCx_1, 0u, 1u, 0u, 1u, LL_ADC_INJ_TRIG_EXT_RISING, ADC_ExternalTriggerInjected);
    pHandle->pParams_str->ADCx_2->JSQR = R3_2_F30X_ADC_InjectedChannelConfig(pHandle->pParams_str->ADCx_2, 0u, 1u, 0u, 1u, LL_ADC_INJ_TRIG_EXT_RISING, ADC_ExternalTriggerInjected);
    LL_ADC_INJ_StartConversion(pHandle->pParams_str->ADCx_1);
    LL_ADC_INJ_StartConversion(pHandle->pParams_str->ADCx_2);

    LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);
    LL_TIM_OC_SetCompareCH4(TIMx,0xFFFFu);
    LL_TIM_OC_SetCompareCH4(TIMx,0x0u);
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);

    while (LL_ADC_IsActiveFlag_JEOS(pHandle->pParams_str->ADCx_1) == RESET)
    {}
    while (LL_ADC_IsActiveFlag_JEOS(pHandle->pParams_str->ADCx_2) == RESET)
    {}

    /* ADCx_1 Injected conversions end interrupt enabling */
    LL_ADC_ClearFlag_JEOS(pHandle->pParams_str->ADCx_1);
    LL_ADC_ClearFlag_JEOS(pHandle->pParams_str->ADCx_2);
    
    NVIC_ClearPendingIRQ(ADC1_2_IRQn);
    LL_ADC_EnableIT_JEOS(pHandle->pParams_str->ADCx_1);

    /* Clear the flags */
    pHandle->OverVoltageFlag = false;
    pHandle->OverCurrentFlag = false;

    pHandle->_Super.DTTest = 0u;
    pHandle->_Super.DTCompCnt = pHandle->_Super.hDTCompCnt;
  }
}

/**
  * @brief  It initializes TIMx peripheral for PWM generation
  * @param TIMx: Timer to be initialized
  * @param pHandle: handler of the current instance of the PWM component
  * @retval none
  */
static void R3_2_F30X_TIMxInit(TIM_TypeDef* TIMx, PWMC_Handle_t *pHdl)
{
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;

  /* disable main TIM counter to ensure
   * a synchronous start by TIM2 trigger */
  LL_TIM_DisableCounter(TIMx);
  
  /* Enables the TIMx Preload on CC1 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  /* Enables the TIMx Preload on CC2 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  /* Enables the TIMx Preload on CC3 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  /* Enables the TIMx Preload on CC4 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  
  LL_TIM_ClearFlag_BRK(TIMx);

  if ((pHandle->pParams_str->bBKIN2Mode) != NONE)
  {
    LL_TIM_ClearFlag_BRK2(TIMx);
  }
  LL_TIM_EnableIT_BRK(TIMx);

  /* Prepare timer for synchronization */
  LL_TIM_GenerateEvent_UPDATE(TIMx);
  if (pHandle->pParams_str->bFreqRatio == 2u)
  {
    if (pHandle->pParams_str->bIsHigherFreqTim == HIGHER_FREQ)
    {
      if (pHandle->pParams_str->bRepetitionCounter == 3u)
      {
        /* Set TIMx repetition counter to 1 */
        LL_TIM_SetRepetitionCounter(TIMx, 1);
        LL_TIM_GenerateEvent_UPDATE(TIMx);
        /* Repetition counter will be set to 3 at next Update */
        LL_TIM_SetRepetitionCounter(TIMx, 3);
      }
    }
    LL_TIM_SetCounter(TIMx, (uint32_t)(pHandle->Half_PWMPeriod)-1u);
  }
  else /* bFreqRatio equal to 1 or 3 */
  {
    if (pHandle->_Super.bMotor == M1)
    {
      if(pHandle->pParams_str->bRepetitionCounter == 1u)
      {
        LL_TIM_SetCounter(TIMx, (uint32_t)(pHandle->Half_PWMPeriod)-1u);
      }
      else if (pHandle->pParams_str->bRepetitionCounter == 3u)
      {
        /* Set TIMx repetition counter to 1 */
        LL_TIM_SetRepetitionCounter(TIMx, 1);
        LL_TIM_GenerateEvent_UPDATE(TIMx);
        /* Repetition counter will be set to 3 at next Update */
        LL_TIM_SetRepetitionCounter(TIMx, 3);
      }
    }
  }
}

/**
* @brief  It perform the start of all the timers required by the control. 
          It uses TIM2 as temporary timer to achieve synchronization between
          PWM signals.
          When this function is called, TIM1 and/or TIM8 must be in frozen state
          with CNT, ARR, REP RATE and trigger correctly set (these setting are 
          usually performed in the Init method accordingly with the configuration)
* @param  none
* @retval none
*/
 void R3_2_F3XX_StartTimers(void)
{
   uint32_t isTIM2ClockOn;
   uint32_t trigOut;

   isTIM2ClockOn = LL_APB1_GRP1_IsEnabledClock (LL_APB1_GRP1_PERIPH_TIM2);
   if (isTIM2ClockOn == 0)
   {  /* Temporary Enable TIM2 clock if not already on */
     LL_APB1_GRP1_EnableClock (LL_APB1_GRP1_PERIPH_TIM2);
     LL_TIM_GenerateEvent_UPDATE (TIM2);
     LL_APB1_GRP1_DisableClock (LL_APB1_GRP1_PERIPH_TIM2);
   }
   else
   {
     trigOut = LL_TIM_ReadReg(TIM2, CR2) & TIM_CR2_MMS;
     LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);
     LL_TIM_GenerateEvent_UPDATE (TIM2);
     LL_TIM_SetTriggerOutput(TIM2, trigOut);
   }
}


/**
  * @brief  It stores into the component the voltage present on Ia and
  *         Ib current feedback analog channels when no current is flowin into the
  *         motor
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void R3_2_F30X_CurrentReadingCalibration(PWMC_Handle_t *pHdl)
{
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;
  pR3_2_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  uint16_t hCalibrationPeriodCounter;
  uint16_t hMaxPeriodsNumber;
  
  pHandle-> wPhaseAOffset = 0u;
  pHandle-> wPhaseBOffset = 0u;
  pHandle-> wPhaseCOffset = 0u;
  
  pHandle->bIndex=0u;
  
  /* It forces inactive level on TIMx CHy and CHyN */
  TIMx->CCER &= (uint16_t)(~TIMxCCER_MASK_CH123);
   
  /* Offset calibration for A & B phases */
  /* Change function to be executed in ADCx_ISR */ 
  pHandle->_Super.pFctGetPhaseCurrents = &R3_2_F30X_HFCurrentsCalibrationAB;
  
  if (pDOPAMPParams_str)
  {
    pHandle->wOAMP1CR = OPAMP1->CSR;
    pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
    pHandle->wOAMP1CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHB);
    pHandle->wOAMP2CR = OPAMP3->CSR;
    pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
    pHandle->wOAMP2CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHA);
  }
  else
  {
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phB;
    pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phA;
  }
  
  R3_2_F30X_SwitchOnPWM(&pHandle->_Super);
  
  /* Wait for NB_CONVERSIONS to be executed */
  hMaxPeriodsNumber=(NB_CONVERSIONS+1u)*(((uint16_t)(pHandle->pParams_str->bRepetitionCounter)+1u)>>1);
  LL_TIM_ClearFlag_CC1 (TIMx);
  hCalibrationPeriodCounter = 0u;
  while (pHandle->bIndex < NB_CONVERSIONS)
  {
    if (LL_TIM_IsActiveFlag_CC1(TIMx))
    {
      LL_TIM_ClearFlag_CC1 (TIMx);
      hCalibrationPeriodCounter++;
      if (hCalibrationPeriodCounter >= hMaxPeriodsNumber)
      {
        if (pHandle->bIndex < NB_CONVERSIONS)
        {
          pHandle->_Super.SWerror = 1u;
          break;
        }
      }
    }
  }
  
  R3_2_F30X_SwitchOffPWM(&pHandle->_Super);

  /* Offset calibration for C phase */
  /* Reset bIndex */
  pHandle->bIndex=0u;

  /* Change function to be executed in ADCx_ISR */ 
  pHandle->_Super.pFctGetPhaseCurrents = &R3_2_F30X_HFCurrentsCalibrationC;

  if (pDOPAMPParams_str)
  {
    pHandle->wOAMP2CR = OPAMP3->CSR;
    pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
    pHandle->wOAMP2CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHC);
  }
  else
  {
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phC;
  }
  
  R3_2_F30X_SwitchOnPWM(&pHandle->_Super);
  
  /* Wait for NB_CONVERSIONS to be executed */
  LL_TIM_ClearFlag_CC1(TIMx);
  hCalibrationPeriodCounter = 0u;
  while (pHandle->bIndex < NB_CONVERSIONS)
  {
    if (LL_TIM_IsActiveFlag_CC1(TIMx))
    {
      LL_TIM_ClearFlag_CC1 (TIMx);
      hCalibrationPeriodCounter++;
      if (hCalibrationPeriodCounter >= hMaxPeriodsNumber)
      {
        if (pHandle->bIndex < NB_CONVERSIONS)
        {
          pHandle->_Super.SWerror = 1u;
          break;
        }
      }
    }
  }
  
  R3_2_F30X_SwitchOffPWM(&pHandle->_Super);
  
  pHandle->wPhaseAOffset >>=4;
  pHandle->wPhaseBOffset >>=4;
  pHandle->wPhaseCOffset >>=4;

  /* Change back function to be executed in ADCx_ISR */ 
  pHandle->_Super.pFctGetPhaseCurrents = &R3_2_F30X_GetPhaseCurrents;

  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to 
     force 50% duty cycle on the three inverer legs */
  /* Disable TIMx preload */  
  TIMx->CCMR1 &= 0xF7F7u;
  TIMx->CCMR2 &= 0xF7F7u;
  TIMx->CCR1 = pHandle->Half_PWMPeriod;
  TIMx->CCR2 = pHandle->Half_PWMPeriod;
  TIMx->CCR3 = pHandle->Half_PWMPeriod;
  
  /* Enable TIMx preload */
  TIMx->CCMR1 |= 0x0808u;
  TIMx->CCMR2 |= 0x0808u;
  
  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  TIMx->CCER |= 0x555u;
  
  pHandle->BrakeActionLock = false;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  It computes and return latest converted motor phase currents motor
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval Ia and Ib current in Curr_Components format
  */
void R3_2_F30X_GetPhaseCurrents(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents)
{
  uint8_t bSector;
  int32_t wAux;
  uint16_t hReg1,hReg2;

  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;
  
  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pHandle->bSoFOC = 0u;
  
  hReg1 = (uint16_t)(pHandle->pParams_str->ADCx_1->JDR1);
  hReg2 = (uint16_t)(pHandle->pParams_str->ADCx_2->JDR1);
  
  bSector = (uint8_t)pHandle->_Super.hSector;

  switch (bSector)
  {
  case SECTOR_4:
  case SECTOR_5: 
    /* Current on Phase C is not accessible     */
    /* Ia = PhaseAOffset - ADC converted value) */
    wAux = (int32_t)(pHandle->wPhaseAOffset)-(int32_t)(hReg2);
    
    /* Saturation of Ia */
    if (wAux < -INT16_MAX)
    {
      pStator_Currents->qI_Component1= -INT16_MAX;
    }  
    else  if (wAux > INT16_MAX)
    { 
      pStator_Currents->qI_Component1= INT16_MAX;
    }
    else
    {
      pStator_Currents->qI_Component1= (int16_t)wAux;
    }
    
    /* Ib = PhaseBOffset - ADC converted value) */
    wAux = (int32_t)(pHandle->wPhaseBOffset)-(int32_t)(hReg1);
    
    /* Saturation of Ib */
    if (wAux < -INT16_MAX)
    {
      pStator_Currents->qI_Component2= -INT16_MAX;
    }  
    else  if (wAux > INT16_MAX)
    { 
      pStator_Currents->qI_Component2= INT16_MAX;
    }
    else
    {
      pStator_Currents->qI_Component2= (int16_t)wAux;
    }
    break;
    
  case SECTOR_6:
  case SECTOR_1:  
    /* Current on Phase A is not accessible     */
    /* Ib = PhaseBOffset - ADC converted value) */
    wAux = (int32_t)(pHandle->wPhaseBOffset)-(int32_t)(hReg1);
    
    /* Saturation of Ib */
    if (wAux < -INT16_MAX)
    {
      pStator_Currents->qI_Component2= -INT16_MAX;
    }  
    else  if (wAux > INT16_MAX)
    { 
      pStator_Currents->qI_Component2= INT16_MAX;
    }
    else
    {
      pStator_Currents->qI_Component2= (int16_t)wAux;
    }
    
    /* Ia = -Ic -Ib */
    wAux = (int32_t)(hReg2) - (int32_t)(pHandle->wPhaseCOffset); /* -Ic */
    wAux -= (int32_t)pStator_Currents->qI_Component2;               /* Ia  */

    /* Saturation of Ia */
    if (wAux> INT16_MAX)
    {
      pStator_Currents->qI_Component1 = INT16_MAX;
    }
    else  if (wAux <-INT16_MAX)
    {
      pStator_Currents->qI_Component1 = -INT16_MAX;
    }
    else
    {  
      pStator_Currents->qI_Component1 = (int16_t)wAux;
    }
    break;
    
  case SECTOR_2:
  case SECTOR_3:
    /* Current on Phase B is not accessible     */
    /* Ia = PhaseAOffset - ADC converted value) */
    wAux = (int32_t)(pHandle->wPhaseAOffset)-(int32_t)(hReg1);
    
    /* Saturation of Ia */
    if (wAux < -INT16_MAX)
    {
      pStator_Currents->qI_Component1= -INT16_MAX;
    }  
    else  if (wAux > INT16_MAX)
    { 
      pStator_Currents->qI_Component1= INT16_MAX;
    }
    else
    {
      pStator_Currents->qI_Component1= (int16_t)wAux;
    }
    
    /* Ib = -Ic -Ia */
    wAux = (int32_t)(hReg2) - (int32_t)(pHandle->wPhaseCOffset); /* -Ic */
    wAux -= (int32_t)pStator_Currents->qI_Component1;               /* Ib */

    /* Saturation of Ib */
    if (wAux> INT16_MAX)
    {
      pStator_Currents->qI_Component2=INT16_MAX;
    }
    else  if (wAux <-INT16_MAX)
    {  
      pStator_Currents->qI_Component2 = -INT16_MAX;
    }
    else  
    {
      pStator_Currents->qI_Component2 = (int16_t)wAux;
    }                     
    break;
    
  default:
    break;
  }   

  pHandle->_Super.hIa = pStator_Currents->qI_Component1;
  pHandle->_Super.hIb = pStator_Currents->qI_Component2;
  pHandle->_Super.hIc = -pStator_Currents->qI_Component1 - pStator_Currents->qI_Component2;
}

/**
  * @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during
  *         calibration. It sum up injected conversion data into wPhaseAOffset and
  *         wPhaseBOffset to compute the offset introduced in the current feedback
  *         network. It is requied to proper configure ADC inputs before to enable
  *         the offset computation.
  * @param  pHandle Pointer on the target component instance
  * @retval It always returns {0,0} in Curr_Components format
  */
static void R3_2_F30X_HFCurrentsCalibrationAB(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents)
{  
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;
  
  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pHandle->bSoFOC = 0u;
  
  if (pHandle->bIndex < NB_CONVERSIONS)
  {
    pHandle-> wPhaseAOffset += pHandle->pParams_str->ADCx_2->JDR1;
    pHandle-> wPhaseBOffset += pHandle->pParams_str->ADCx_1->JDR1;
    pHandle->bIndex++;
  }
}

/**
* @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during 
*         calibration. It sum up injected conversion data into wPhaseCOffset
*         to compute the offset introduced in the current feedback
*         network. It is requied to proper configure ADC input before to enable
*         the offset computation.
* @param  pHandle Pointer on the target component instance
* @retval It always returns {0,0} in Curr_Components format
*/
static void R3_2_F30X_HFCurrentsCalibrationC(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents)
{
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;
  
  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pHandle->bSoFOC = 0u;
  
  if (pHandle->bIndex < NB_CONVERSIONS)
  {
    pHandle-> wPhaseCOffset += pHandle->pParams_str->ADCx_2->JDR1;
    pHandle->bIndex++;
  }
}

/**
  * @brief  It turns on low sides switches. This function is intended to be 
  *         used for charging boot capacitors of driving section. It has to be 
  *         called each motor start-up when using high voltage drivers
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void R3_2_F30X_TurnOnLowSides(PWMC_Handle_t *pHdl)
{
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  
  pHandle->_Super.bTurnOnLowSidesAction = true;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->TIMx);
  
  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1(TIMx,0u);
  LL_TIM_OC_SetCompareCH2(TIMx,0u);
  LL_TIM_OC_SetCompareCH3(TIMx,0u);
  
  /* Wait until next update */
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx)==RESET)
  {}
  
  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);
  
  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  return; 
}


/**
  * @brief  It enables PWM generation on the proper Timer peripheral acting on MOE
  *         bit
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void R3_2_F30X_SwitchOnPWM(PWMC_Handle_t *pHdl)
{  
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = false;
  
  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == RESET)
  {}
  LL_TIM_ClearFlag_UPDATE(TIMx);
  
  /* Set all duty to 50% */
  /* Set ch5 ch6 for triggering */
  /* Clear Update Flag */
  LL_TIM_OC_SetCompareCH1 (TIMx,(uint32_t)(pHandle->Half_PWMPeriod) >> 1);
  LL_TIM_OC_SetCompareCH2 (TIMx,(uint32_t)(pHandle->Half_PWMPeriod) >> 1);
  LL_TIM_OC_SetCompareCH3 (TIMx,(uint32_t)(pHandle->Half_PWMPeriod) >> 1);
  LL_TIM_OC_SetCompareCH4 (TIMx,(uint32_t)(pHandle->Half_PWMPeriod) - 5u);
  
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == RESET)
  {}
  
  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  LL_TIM_EnableAllOutputs (TIMx);
  
  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  /* Enable Update IRQ */
  LL_TIM_EnableIT_UPDATE(TIMx);
}


/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void R3_2_F30X_SwitchOffPWM(PWMC_Handle_t *pHdl)
{ 
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  
  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  if (pHandle->BrakeActionLock == true)
  {
  }
  else
  {
    TIMx->BDTR &= ~((uint32_t)(TIM_OSSI_ENABLE));
    
    if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
  }
  LL_TIM_DisableAllOutputs(TIMx);
  
  /* Disable UPDATE ISR */
  LL_TIM_DisableIT_UPDATE(TIMx);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  Stores into the component's handle the voltage present on Ia and
  *         Ib current feedback analog channels when no current is flowin into the
  *         motor
  * @param  pHandle handler of the current instance of the PWM component
  * @retval none
  */
static uint16_t R3_2_F30X_WriteTIMRegisters(PWMC_Handle_t *pHdl)
{
  uint16_t hAux;
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
    
  TIMx->CCR1 = pHandle->_Super.hCntPhA;
  TIMx->CCR2 = pHandle->_Super.hCntPhB;
  TIMx->CCR3 = pHandle->_Super.hCntPhC;
    
  /* Limit for update event */
  /* Check the status of SOFOC flag. If it is set, an update event has occurred 
  and thus the FOC rate is too high */
  if (pHandle->bSoFOC != 0u)
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  if (pHandle->_Super.SWerror == 1u)
  {
    hAux = MC_FOC_DURATION;
    pHandle->_Super.SWerror = 0u;
  }
  return hAux;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  Configure the ADC for the current sampling related to sector 1.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHandle Pointer on the target component instance
  * @retval none
  */
uint16_t R3_2_F30X_SetADCSampPointSect1(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  pR3_2_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;
  
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = OPAMP1->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHB);
      pHandle->wOAMP2CR = OPAMP3->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHA);
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phB;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phA;
    } 
  }
  else
  {
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhB);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhA - pHandle->pParams_str->hTbefore;
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhA + pHandle->pParams_str->hTafter;
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        { 
          /* Set CC4 as PWM mode 1 */
          TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
          TIMx->CCMR2 |= CCMR2_CH4_PWM1;

          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }
    
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = OPAMP1->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHB);
      pHandle->wOAMP2CR = OPAMP3->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHC);
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phB;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phC;
    }
    
    /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp; 
  }
  return R3_2_F30X_WriteTIMRegisters(&pHandle->_Super);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  Configure the ADC for the current sampling related to sector 2.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHandle Pointer on the target component instance
  * @retval none
  */
uint16_t R3_2_F30X_SetADCSampPointSect2(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  pR3_2_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;
  
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = OPAMP1->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHB);
      pHandle->wOAMP2CR = OPAMP3->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHA);
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phB;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phA;
    }
  }
  else
  {
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhA);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhB - pHandle->pParams_str->hTbefore;
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhB + pHandle->pParams_str->hTafter;
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* Set CC4 as PWM mode 1 */
          TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
          TIMx->CCMR2 |= CCMR2_CH4_PWM1;
          
          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }
    
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = OPAMP1->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHA);
      pHandle->wOAMP2CR = OPAMP3->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHC);
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phC;
    }
    
    /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp; 
  }
  return R3_2_F30X_WriteTIMRegisters(&pHandle->_Super);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  Configure the ADC for the current sampling related to sector 3.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHandle Pointer on the target component instance
  * @retval none
  */
uint16_t R3_2_F30X_SetADCSampPointSect3(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  pR3_2_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;

  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;
  
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = OPAMP1->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHB);
      pHandle->wOAMP2CR = OPAMP3->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHA);
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phB;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phA;
    }
  }
  else
  {
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhC);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhB - pHandle->pParams_str->hTbefore;
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhB + pHandle->pParams_str->hTafter;
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* Set CC4 as PWM mode 1 */
          TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
          TIMx->CCMR2 |= CCMR2_CH4_PWM1;
          
          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }
    
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = OPAMP1->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHA);
      pHandle->wOAMP2CR = OPAMP3->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHC);
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phA;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phC;
    }
    
    /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp; 
  }
  return R3_2_F30X_WriteTIMRegisters(&pHandle->_Super);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  Configure the ADC for the current sampling related to sector 4.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHandle Pointer on the target component instance
  * @retval none
  */
uint16_t R3_2_F30X_SetADCSampPointSect4(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  pR3_2_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;
  
  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;
  
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = OPAMP1->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHB);
      pHandle->wOAMP2CR = OPAMP3->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHA);
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phB;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phA;
    }
  }
  else
  {
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhB);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhC - pHandle->pParams_str->hTbefore;
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhC + pHandle->pParams_str->hTafter;
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* Set CC4 as PWM mode 1 */
          TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
          TIMx->CCMR2 |= CCMR2_CH4_PWM1;
          
          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }
    
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = OPAMP1->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHB);
      pHandle->wOAMP2CR = OPAMP3->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHA);
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phB;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phA;
    }
    
    /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp; 
  }
  return R3_2_F30X_WriteTIMRegisters(&pHandle->_Super);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  Configure the ADC for the current sampling related to sector 5.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHandle Pointer on the target component instance
  * @retval none
  */
uint16_t R3_2_F30X_SetADCSampPointSect5(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  pR3_2_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;
  
  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;
  
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = OPAMP1->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHB);
      pHandle->wOAMP2CR = OPAMP3->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHA);
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phB;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phA;
    }
  }
  else
  {
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhA);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhC - pHandle->pParams_str->hTbefore;
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhC + pHandle->pParams_str->hTafter;
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* Set CC4 as PWM mode 1 */
          TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
          TIMx->CCMR2 |= CCMR2_CH4_PWM1;
          
          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }
    
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = OPAMP1->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHB);
      pHandle->wOAMP2CR = OPAMP3->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHA);
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phB;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phA;
    }
    
    /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp; 
  }
  return R3_2_F30X_WriteTIMRegisters(&pHandle->_Super);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  Configure the ADC for the current sampling related to sector 6.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHandle Pointer on the target component instance
  * @retval none
  */
uint16_t R3_2_F30X_SetADCSampPointSect6(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  pR3_2_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;
  
  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;
  
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter))
  {
    TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_4; /* Dummy just for the GetPhaseCurrent */
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = OPAMP1->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHB);
      pHandle->wOAMP2CR = OPAMP3->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHA);
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phB;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phA;
    }
  }
  else
  {
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    {
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhC);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhA - pHandle->pParams_str->hTbefore;
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhA + pHandle->pParams_str->hTafter;
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {   
          /* Set CC4 as PWM mode 1 */
          TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
          TIMx->CCMR2 |= CCMR2_CH4_PWM1;
          
          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }
    
    if (pDOPAMPParams_str)
    {
      pHandle->wOAMP1CR = OPAMP1->CSR;
      pHandle->wOAMP1CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP1CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP_InvertingInput | pDOPAMPParams_str->wOPAMP_NonInvertingInput_PHB);
      pHandle->wOAMP2CR = OPAMP3->CSR;
      pHandle->wOAMP2CR &= (uint32_t) (OPAMP_CSR_DEFAULT_MASK);
      pHandle->wOAMP2CR |= (uint32_t)(pDOPAMPParams_str->wOPAMP2_InvertingInput | pDOPAMPParams_str->wOPAMP2_NonInvertingInput_PHC);
    }
    else
    {
      pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phB;
      pHandle->wADC2_JSQR = pHandle->wADC_JSQR_phC;
    }
    
    /* Set TIMx_CH4 value */
    TIMx->CCR4 = hCntSmp; 
  }
  return R3_2_F30X_WriteTIMRegisters(&pHandle->_Super);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  It contains the TIMx Update event interrupt
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void *R3_2_F30X_TIMx_UP_IRQHandler(PWMC_Handle_t *pHdl)
{
    PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;
    pR3_2_F30XOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;
    
    /* If Opamps are enabled wait until queue becomes empty*/
    if (pDOPAMPParams_str)
    {
      while (pHandle->pParams_str->ADCx_1->JSQR != 0x00000000u)
      {}
      OPAMP1->CSR = pHandle->wOAMP1CR;
      OPAMP3->CSR = pHandle->wOAMP2CR;
    }
    
    /* Write value in the Queue */
    pHandle->pParams_str->ADCx_1->JSQR = pHandle->wADC1_JSQR;
    pHandle->pParams_str->ADCx_2->JSQR = pHandle->wADC2_JSQR;
    
    /* Set the SOFOC flag to indicate the execution of Update IRQ*/
    pHandle->bSoFOC = 1u;
  return &(pHandle->_Super.bMotor);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  It contains the TIMx Break2 event interrupt
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void *R3_2_F30X_BRK2_IRQHandler(PWMC_Handle_t *pHdl)
{
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;

  if (pHandle->BrakeActionLock == false)
  {
    if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
        LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
        LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
        LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
  }
  pHandle->OverCurrentFlag = true;

  return &(pHandle->_Super.bMotor);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  It contains the TIMx Break1 event interrupt
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void *R3_2_F30X_BRK_IRQHandler(PWMC_Handle_t *pHdl)
{
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;

  pHandle->pParams_str->TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  pHandle->OverVoltageFlag = true;
  pHandle->BrakeActionLock = true;

  return &(pHandle->_Super.bMotor);
}
/**
  * @brief  Execute a regular conversion using ADCx.
  *         The function is not re-entrant (can't executed twice at the same time)
  * @param  this related object of class CR3_2_F30X_PWMC
  * @retval It returns converted value or oxFFFF for conversion error
  */
uint16_t R3_2_F30X_ExecRegularConv(PWMC_Handle_t *pHdl, uint8_t bChannel)
{
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;
  ADC_TypeDef* ADCx = pHandle->pParams_str->regconvADCx;
  
  LL_ADC_REG_SetSequencerRanks(pHandle->pParams_str->regconvADCx,
                               LL_ADC_REG_RANK_1,
                               (bChannel<<ADC_CFGR_AWD1CH_Pos));
  LL_ADC_REG_StartConversion(ADCx);
  
  /* Wait until end of regular conversion */
  while(LL_ADC_IsActiveFlag_EOC(ADCx) == 0u)
  {}
  
  pHandle->hRegConv = LL_ADC_REG_ReadConversionData12(ADCx);
  return (pHandle->hRegConv);

}

/**
  * @brief  It sets the specified sampling time for the specified ADC channel
  *         on ADCx. It must be called once for each channel used
  * @param  ADC channel, sampling time
  * @retval none
  */
void R3_2_F30X_ADC_SetSamplingTime(PWMC_Handle_t *pHdl, ADConv_t ADConv_struct)
{ 
  uint32_t tmpreg2 = 0u;
  uint8_t ADC_Channel = ADConv_struct.Channel;
  uint8_t ADC_SampleTime = ADConv_struct.SamplTime;
  
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;

  /* Channel sampling configuration */
  /* if ADC_CHANNEL_10 ... ADC_CHANNEL_18 is selected */
  if (ADC_Channel > ADC_CHANNEL_9)
  {
    uint32_t wAux,wAux2;
    /* Get the old register value */
    /* Calculate the mask to clear */
    wAux = ADC_SMPR2_SMP10;
    wAux2 = 3u * ((uint32_t)(ADC_Channel) - 10u);
    tmpreg2 = wAux << wAux2;
    /* Clear the old channel sample time */
    pHandle->pParams_str->regconvADCx->SMPR2 &= ~tmpreg2;
    /* Calculate the mask to set */
    wAux = (uint32_t)(ADC_SampleTime);
    pHandle->pParams_str->regconvADCx->SMPR2 |= wAux << wAux2;
    
  }
  else /* ADC_Channel include in ADC_CHANNEL_[0..9] */
  {
    uint32_t wAux,wAux2;
    /* Get the old register value */
    /* Calculate the mask to clear */
    wAux = ADC_SMPR1_SMP1;
    wAux2 = 3u * ((uint32_t)(ADC_Channel) - 1u);
    tmpreg2 = wAux << wAux2;
    /* Clear the old channel sample time */
    pHandle->pParams_str->regconvADCx->SMPR1 &= ~tmpreg2;
    /* Calculate the mask to set */
    wAux = (uint32_t)(ADC_SampleTime);
    wAux2 = 3u * ((uint32_t)(ADC_Channel));
    pHandle->pParams_str->regconvADCx->SMPR1 |= wAux << wAux2;
  }
}
/**
  * @brief  It is used to check if an overcurrent occurred since last call.
  * @param  pHandle Pointer on the target component instance
  * @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
  *                  detected since last method call, MC_NO_FAULTS otherwise.
  */
uint16_t R3_2_F30X_IsOverCurrentOccurred(PWMC_Handle_t *pHdl)
{
  PWMC_R3_2_F3_Handle_t *pHandle = (PWMC_R3_2_F3_Handle_t *)pHdl;

  uint16_t retVal = MC_NO_FAULTS;
  
  if (pHandle->OverVoltageFlag == true)
  {
    retVal = MC_OVER_VOLT;
    pHandle->OverVoltageFlag = false;
  }
  
  if (pHandle->OverCurrentFlag == true )
  {
    retVal |= MC_BREAK_IN;
    pHandle->OverCurrentFlag = false;
  }
  
  return retVal;
}

/**
  * @brief  It is used to configure the analog output used for protection
  *         thresholds.
  * @param  DAC_Channel: the selected DAC channel.
  *          This parameter can be:
  *            @arg DAC_Channel_1: DAC Channel1 selected
  *            @arg DAC_Channel_2: DAC Channel2 selected
  * @param  hDACVref Value of DAC reference expressed as 16bit unsigned integer.
  *         Ex. 0 = 0V 65536 = VDD_DAC.
  * @retval none
  */
static void R3_2_F30X_SetAOReferenceVoltage(uint32_t DAC_Channel, uint16_t hDACVref)
{ 

  if (DAC_Channel == LL_DAC_CHANNEL_2)
  {
    LL_DAC_ConvertData12LeftAligned (DAC1, LL_DAC_CHANNEL_2, hDACVref );
  }
  else
  {
    LL_DAC_ConvertData12LeftAligned (DAC1, LL_DAC_CHANNEL_1, hDACVref );
  }
  
  /* Enable DAC Channel */
  LL_DAC_TrigSWConversion (DAC1, DAC_Channel);
  LL_DAC_Enable (DAC1, DAC_Channel );

}

static uint32_t R3_2_F30X_ADC_InjectedChannelConfig(ADC_TypeDef* ADCx,
                                                    uint8_t ADC_Channel,
                                                    uint8_t Rank,
                                                    uint8_t ADC_SampleTime,
                                                    uint8_t SequencerLength,
                                                    uint16_t ADC_ExternalTriggerInjectedPolarity,
                                                    uint16_t ADC_ExternalTriggerInjected)
{
  uint32_t tmpreg1 = 0u, tmpreg2 = 0u, tmpregA = 0u;
  uint32_t wAux,wAux2;
  
  /*  ADC_InjectedSequencerLengthConfig(ADCx,1); */
  tmpregA = ADCx->JSQR;
  /* Clear the old injected sequnence lenght JL bits */
  tmpregA &= ~(uint32_t)ADC_JSQR_JL;
  /* Set the injected sequnence lenght JL bits */
  tmpregA |= ((uint32_t)(SequencerLength) - 1u); /* first value is sequencer lenght */
  
  /* Disable the selected ADC conversion on external event */
  tmpregA &= ~ADC_JSQR_JEXTEN;
  tmpregA |= ADC_ExternalTriggerInjectedPolarity; 
 
  /* Disable the selected ADC conversion on external event */
  tmpregA &= ~ADC_JSQR_JEXTSEL;
  tmpregA |= ADC_ExternalTriggerInjected;
  
  /* Channel sampling configuration */
  /* if ADC_CHANNEL_10 ... ADC_CHANNEL_18 is selected */
  if (ADC_Channel > ADC_CHANNEL_9 )
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SMPR2;
    /* Calculate the mask to clear */
    wAux = ADC_SMPR2_SMP10;
    wAux2 = 3u * ((uint32_t)(ADC_Channel) - 10u);
    tmpreg2 = wAux << wAux2;
    /* Clear the old channel sample time */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    wAux = (uint32_t)(ADC_SampleTime);
    tmpreg2 = wAux << wAux2;
    /* Set the new channel sample time */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADCx->SMPR2 = tmpreg1;
  }
  else if (ADC_Channel != 0u)
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SMPR1;
    /* Calculate the mask to clear */
    wAux = ADC_SMPR1_SMP0;
    wAux2 = 3u * (uint32_t)(ADC_Channel);
    tmpreg2 =  wAux << wAux2;
    /* Clear the old channel sample time */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    wAux = (uint32_t)ADC_SampleTime;
    wAux2 = 3u * (uint32_t)(ADC_Channel);
    tmpreg2 =  wAux << wAux2;
    /* Set the new channel sample time */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADCx->SMPR1 = tmpreg1;
  }
  else
  {
  }

  /* Rank configuration */
  /* Get the old register value */
  tmpreg1 = tmpregA;
  /* Calculate the mask to clear */
  wAux = ADC_JSQR_JSQ1;
  wAux2 = 6u * ((uint32_t)(Rank) - 1u);
  tmpreg2 = wAux << wAux2;
  /* Clear the old SQx bits for the selected rank */
  tmpreg1 &= ~tmpreg2;
  /* Calculate the mask to set */
  wAux = ADC_Channel;
  wAux2 = 6u * (uint32_t)(Rank) + 2u;
  tmpreg2 = wAux << wAux2;
  /* Set the SQx bits for the selected rank */
  tmpreg1 |= tmpreg2;
  /* Store the new register value */
  
  return (tmpreg1);
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

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
