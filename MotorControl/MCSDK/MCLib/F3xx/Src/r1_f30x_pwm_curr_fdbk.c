/**
  ******************************************************************************
  * @file    r1_f30x_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the CCC component of the Motor Control SDK:
  *           + initializes MCU peripheral for 1 shunt topology and F3 family
  *           + performs PWM duty cycle computation and generation
  *           + performs current sensing
  *           +
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
#include "r1_f30x_pwm_curr_fdbk.h"
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
 * @defgroup r1_f30x_pwm_curr_fdbk R1 F30x PWM & Current Feedback
 *
 * @brief STM32F3, 1-Shunt PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F3 MCU
 * and using a single shunt resistor current sensing topology.
 *
 * @todo: TODO: complete documentation.
 * @{
 */

/* Constant values -----------------------------------------------------------*/
#define TIMxCCER_MASK_CH123              (TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|\
                                          TIM_CCER_CC1NE|TIM_CCER_CC2NE|TIM_CCER_CC3NE)
#define CC12_PRELOAD_ENABLE_MASK         (TIM_CCMR1_OC1PE|TIM_CCMR1_OC2PE)
#define CC3_PRELOAD_ENABLE_MASK          TIM_CCMR2_OC3PE
#define CC1_PRELOAD_DISABLE_MASK         ~TIM_CCMR1_OC1PE
#define CC2_PRELOAD_DISABLE_MASK         ~TIM_CCMR1_OC2PE
#define CC3_PRELOAD_DISABLE_MASK         ~TIM_CCMR2_OC3PE
#define TIMxCCR56_PRELOAD_DISABLE_MASK   ~(TIM_CCMR3_OC5PE|TIM_CCMR3_OC6PE)
#define TIMxCCR56_PRELOAD_ENABLE_MASK    (TIM_CCMR3_OC5PE|TIM_CCMR3_OC6PE)
#define NB_CONVERSIONS 16u

/* boundary zone definition */
#define REGULAR         ((uint8_t)0u)
#define BOUNDARY_1      ((uint8_t)1u)  /* Two small, one big */
#define BOUNDARY_2      ((uint8_t)2u)  /* Two big, one small */
#define BOUNDARY_3      ((uint8_t)3u)  /* Three equal        */

#define INVERT_NONE 0u
#define INVERT_A 1u
#define INVERT_B 2u
#define INVERT_C 3u

#define SAMP_NO 0u
#define SAMP_IA 1u
#define SAMP_IB 2u
#define SAMP_IC 3u
#define SAMP_NIA 4u
#define SAMP_NIB 5u
#define SAMP_NIC 6u
#define SAMP_OLDA 7u
#define SAMP_OLDB 8u
#define SAMP_OLDC 9u

static const uint8_t REGULAR_SAMP_CUR1[6] = {SAMP_NIC,SAMP_NIC,SAMP_NIA,SAMP_NIA,SAMP_NIB,SAMP_NIB};
static const uint8_t REGULAR_SAMP_CUR2[6] = {SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA};
static const uint8_t BOUNDR1_SAMP_CUR2[6] = {SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA,SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR1[6] = {SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC,SAMP_IC,SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR2[6] = {SAMP_IC,SAMP_IA,SAMP_IA,SAMP_IB,SAMP_IB,SAMP_IC};

/* Private typedef -----------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/
static void R1F30X_HFCurrentsCalibration(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents);
static void R1F30X_SetAOReferenceVoltage(uint32_t DAC_Channel, uint16_t hDACVref);
static void R1F30X_1ShuntMotorVarsInit(PWMC_Handle_t *pHdl);
static void R1F30X_1ShuntMotorVarsRestart(PWMC_Handle_t *pHdl);
static void R1F30X_TIMxInit(TIM_TypeDef* TIMx, PWMC_R1_F3_Handle_t *pHdl);
static void R1F30X_RLGetPhaseCurrents(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents);
static void R1F30X_RLTurnOnLowSides(PWMC_Handle_t *pHdl);
static void R1F30X_RLSwitchOnPWM(PWMC_Handle_t *pHdl);
static void R1F30X_RLSwitchOffPWM(PWMC_Handle_t *pHdl);

/**
 * @brief  It initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading
 *         in ICS configuration using STM32F103x High Density
 * @param pHandle: handler of the current instance of the PWM component
 * @retval none
 */
void R1F30X_Init(PWMC_R1_F3_Handle_t *pHandle)
{
  OPAMP_TypeDef* OPAMPx = pHandle->pParams_str->wOPAMP_Selection;
  COMP_TypeDef* COMP_OCPx = pHandle->pParams_str->wCompOCPSelection;
  COMP_TypeDef* COMP_OVPx = pHandle->pParams_str->wCompOVPSelection;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef* ADCx = pHandle->pParams_str->ADCx;

  R1F30X_1ShuntMotorVarsInit(&pHandle->_Super);

  /* Enable TIM1 - TIM8 clock */
  if(TIMx == TIM1)
  {
    /* DMA Event related to TIM1 Channel 4 */
    /* DMA1 Channel4 configuration ----------------------------------------------*/
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)pHandle->hDmaBuff);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)&(TIM1->CCR1));
    LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_4,2);

    /* ensable DMA1 Channel4 */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
    pHandle->DistortionDMAy_Chx = DMA1_Channel4;

    /* DMA Event related to TIM1 update */
    /* DMA1 Channel5 configuration ----------------------------------------------*/
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&(pHandle->wPreloadDisableActing));
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&TIM1->CCMR1);
    LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_5,1);

    /* enable DMA1 Channel5 */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
    pHandle->PreloadDMAy_Chx = DMA1_Channel5;
  }
#if !defined(STM32F302x8) && !defined(STM32F318xx) && !defined(STM32F303x8) && !defined(STM32F328xx) && !defined(STM32F334x8) && !defined(STM32F301x8)
  else
  {
    /* DMA Event related to TIM8 Channel 4 */
    /* DMA2 Channel2 configuration ----------------------------------------------*/
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_CHANNEL_2, (uint32_t)pHandle->hDmaBuff);
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_CHANNEL_2, (uint32_t)&TIM8->CCR1);
    LL_DMA_SetDataLength(DMA2,LL_DMA_CHANNEL_2,2);
    /* enable DMA2 Channel2 */
    LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_2);
    pHandle->DistortionDMAy_Chx = DMA2_Channel2;

    /* DMA Event related to TIM8 update */
    /* DMA2 Channel1 configuration ----------------------------------------------*/
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_CHANNEL_1, (uint32_t)&pHandle->wPreloadDisableActing);
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_CHANNEL_1, (uint32_t)&TIM8->CCMR1);
    LL_DMA_SetDataLength(DMA2,LL_DMA_CHANNEL_1,1);
    /* enable DMA2 Channel2 */
    LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_1);
    pHandle->PreloadDMAy_Chx = DMA2_Channel1;
  }
#endif

  R1F30X_TIMxInit(TIMx, pHandle);

  if (OPAMPx)
  {
    /* enable OPAMP */
    LL_OPAMP_Enable(OPAMPx);
    LL_OPAMP_Lock(OPAMPx);
  }

  /* Over current protection */
  if (COMP_OCPx)
  {
    /* Inverting input*/
    if (pHandle->pParams_str->bCompOCPInvInput_MODE != EXT_MODE)
    {
      if (LL_COMP_GetInputMinus(COMP_OCPx) == LL_COMP_INPUT_MINUS_DAC1_CH1)
      {
        R1F30X_SetAOReferenceVoltage(LL_DAC_CHANNEL_1, (uint16_t)(pHandle->pParams_str->hDAC_OCP_Threshold));
      }
#if !defined(STM32F302x8) && !defined(STM32F318xx) && !defined(STM32F303x8) && !defined(STM32F328xx) && !defined(STM32F334x8) && !defined(STM32F301x8)	  
      else if (LL_COMP_GetInputMinus(COMP_OCPx) == LL_COMP_INPUT_MINUS_DAC1_CH2)
      {
        R1F30X_SetAOReferenceVoltage(LL_DAC_CHANNEL_2, (uint16_t)(pHandle->pParams_str->hDAC_OCP_Threshold));
      }
#endif	  
      else
      {}
    }

    /* Wait to stabilize DAC voltage */
    volatile uint16_t waittime = 0u;
    for(waittime=0u;waittime<1000u;waittime++)
    {}

    /* enable comparator */
#if defined(COMP_CSR_COMPxHYST)
    LL_COMP_SetInputHysteresis(COMP_OCPx, LL_COMP_HYSTERESIS_LOW);
#endif
    LL_COMP_Enable(COMP_OCPx);
    LL_COMP_Lock(COMP_OCPx);
  }

  /* Over voltage protection */
  if (COMP_OVPx)
  {
    /* Inverting input*/
    if (pHandle->pParams_str->bCompOCPInvInput_MODE != EXT_MODE)
    {
      if (LL_COMP_GetInputMinus(COMP_OVPx) == LL_COMP_INPUT_MINUS_DAC1_CH1)
      {
        R1F30X_SetAOReferenceVoltage(LL_DAC_CHANNEL_1, (uint16_t)(pHandle->pParams_str->hDAC_OVP_Threshold));
      }
#if !defined(STM32F302x8) && !defined(STM32F318xx) && !defined(STM32F303x8) && !defined(STM32F328xx) && !defined(STM32F334x8) && !defined(STM32F301x8)	  
      else if (LL_COMP_GetInputMinus(COMP_OVPx) == LL_COMP_INPUT_MINUS_DAC1_CH2)
      {
        R1F30X_SetAOReferenceVoltage(LL_DAC_CHANNEL_2, (uint16_t)(pHandle->pParams_str->hDAC_OVP_Threshold));
      }
#endif	  
      else
      {}
    }

    /* Wait to stabilize DAC voltage */
    volatile uint16_t waittime = 0u;
    for(waittime=0u;waittime<1000u;waittime++)
    {}

    /* enable comparator */
#if defined(COMP_CSR_COMPxHYST)
    LL_COMP_SetInputHysteresis(COMP_OCPx, LL_COMP_HYSTERESIS_LOW);
#endif
    LL_COMP_Enable(COMP_OVPx);
    LL_COMP_Lock(COMP_OVPx);

  }

  if(pHandle->pParams_str->TIMx == TIM1)
  {   
    /* TIM1 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM1_STOP);
  }
#if !defined(STM32F302x8) && !defined(STM32F318xx) && !defined(STM32F303x8) && !defined(STM32F328xx) && !defined(STM32F334x8) && !defined(STM32F301x8)	  
  else
  {
    /* TIM8 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM8_STOP);
  }  
#endif
  
  LL_ADC_EnableInternalRegulator(ADCx);

  LL_ADC_StartCalibration(ADCx,LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(ADCx))
  {
  }

  if (pHandle->_Super.bMotor == M1)
  {
    if (pHandle->pParams_str->regconvADCx != ADCx)
    {
      LL_ADC_EnableInternalRegulator(pHandle->pParams_str->regconvADCx);
      LL_ADC_StartCalibration(pHandle->pParams_str->regconvADCx,LL_ADC_SINGLE_ENDED);
      while (LL_ADC_IsCalibrationOnGoing(pHandle->pParams_str->regconvADCx))
      {}
    }
  }

  /* ADC registers configuration ---------------------------------*/
  /* Enable ADC*/
  LL_ADC_Enable(ADCx);

  if (pHandle->_Super.bMotor == M1)
  {
    if (pHandle->pParams_str->regconvADCx != ADCx)
    {
      LL_ADC_Enable(pHandle->pParams_str->regconvADCx);
    }
  }

  /* Flushing JSQR queue of context by setting JADSTP = 1 (JQM)=1 */
  LL_ADC_INJ_StopConversion(ADCx);

  if(pHandle->pParams_str->TIMx == TIM1)
  {   
    LL_ADC_INJ_ConfigQueueContext(ADCx,
                                  LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2,
                                  LL_ADC_INJ_TRIG_EXT_RISING,
                                  LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS,
                                  pHandle->pParams_str->bIChannel<<ADC_CFGR_AWD1CH_Pos,
                                  pHandle->pParams_str->bIChannel<<ADC_CFGR_AWD1CH_Pos,
                                  pHandle->pParams_str->bIChannel<<ADC_CFGR_AWD1CH_Pos,
                                  pHandle->pParams_str->bIChannel<<ADC_CFGR_AWD1CH_Pos);
  }
#if !defined(STM32F302x8) && !defined(STM32F318xx) && !defined(STM32F303x8) && !defined(STM32F328xx) && !defined(STM32F334x8) && !defined(STM32F301x8) 
  else
  {
    LL_ADC_INJ_ConfigQueueContext(ADCx,
                                  LL_ADC_INJ_TRIG_EXT_TIM8_TRGO2,
                                  LL_ADC_INJ_TRIG_EXT_RISING,
                                  LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS,
                                  pHandle->pParams_str->bIChannel<<ADC_CFGR_AWD1CH_Pos,
                                  pHandle->pParams_str->bIChannel<<ADC_CFGR_AWD1CH_Pos,
                                  pHandle->pParams_str->bIChannel<<ADC_CFGR_AWD1CH_Pos,
                                  pHandle->pParams_str->bIChannel<<ADC_CFGR_AWD1CH_Pos);
  }  
#endif


  /* store register value in the handle to be used later during SVPWM for re-init */
  pHandle->wADC_JSQR = ADCx->JSQR;

  LL_ADC_INJ_SetQueueMode(ADCx, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY);
  LL_ADC_INJ_SetSequencerDiscont(ADCx, LL_ADC_INJ_SEQ_DISCONT_1RANK);

  /* Clear the flags */
  pHandle->OverVoltageFlag = false;
  pHandle->OverCurrentFlag = false;

  pHandle->_Super.DTTest = 0u;
  pHandle->_Super.DTCompCnt = pHandle->_Super.hDTCompCnt;

}

/**
  * @brief  It initializes TIMx peripheral for PWM generation
  * @param TIMx: Timer to be initialized
  * @param pHandle: handler of the current instance of the PWM component
  * @retval none
  */
static void R1F30X_TIMxInit(TIM_TypeDef* TIMx, PWMC_R1_F3_Handle_t *pHandle)
{

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
  /* Enables the TIMx Preload on CC5 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH5);
  /* Enables the TIMx Preload on CC6 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH6);

  /* Always enable BKIN for safety feature */
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

  pHandle->wPreloadDisableCC1 = TIMx->CCMR1 & CC1_PRELOAD_DISABLE_MASK;
  pHandle->wPreloadDisableCC2 = TIMx->CCMR1 & CC2_PRELOAD_DISABLE_MASK;
  pHandle->wPreloadDisableCC3 = TIMx->CCMR2 & CC3_PRELOAD_DISABLE_MASK;

}

/**
  * @brief  First initialization of the handler
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
static void R1F30X_1ShuntMotorVarsInit(PWMC_Handle_t *pHdl)
{  
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;

  /* Init motor vars */
  pHandle->bInverted_pwm_new=INVERT_NONE;
  pHandle->hFlags &= (~STBD3); /*STBD3 cleared*/
  pHandle->hFlags &= (~DSTEN); /*DSTEN cleared*/

  pHandle->Half_PWMPeriod = ((pHandle->_Super.hPWMperiod)/2u);

  /* After reset, value of DMA buffers for distortion*/
  pHandle->hDmaBuff[0] =  pHandle->Half_PWMPeriod + 1u;
  pHandle->hDmaBuff[1] =  pHandle->Half_PWMPeriod >> 1; /*dummy*/

  /* Default value of sampling points */ 
  pHandle->hCntSmp1 = (pHandle->Half_PWMPeriod >> 1) + (pHandle->pParams_str->hTafter);
  pHandle->hCntSmp2 = pHandle->Half_PWMPeriod - 1u;
}

/**
  * @brief  Re-initialization of of the handler after each motor start
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
static void R1F30X_1ShuntMotorVarsRestart(PWMC_Handle_t *pHdl)
{
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;

  /* Default value of sampling points */
  pHandle->hCntSmp1 = (pHandle->Half_PWMPeriod >> 1) + (pHandle->pParams_str->hTafter);
  pHandle->hCntSmp2 = pHandle->Half_PWMPeriod - 1u;

  pHandle->bInverted_pwm_new=INVERT_NONE;
  pHandle->hFlags &= (~STBD3); /*STBD3 cleared*/

  /* Set the default previous value of Phase A,B,C current */
  pHandle->hCurrAOld=0;
  pHandle->hCurrBOld=0;

  pHandle->hDmaBuff[0] =  pHandle->Half_PWMPeriod + 1u;
  pHandle->hDmaBuff[1] =  pHandle->Half_PWMPeriod >> 1; /*dummy*/

  pHandle->BrakeActionLock = false;
}

/**
  * @brief  It perform the start of all the timers required by the control.
  *         It utilizes TIM2 as temporary timer to achieve synchronization between
  *         PWM signals. When pHandle function is called, TIM1 and/or TIM8 must be
  *         in frozen state with CNT, ARR, REP RATE and trigger correctly set (these
  *         setting are usually performed in the Init method accordingly with the configuration)
 * @retval none
 */
void R1F3XX_StartTimers(void)
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
  * @brief It stores into pHandle the offset voltage read onchannels when no
  * current is flowing into the motor
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R1F30X_CurrentReadingCalibration(PWMC_Handle_t *pHdl)
{
  uint16_t hCalibrationPeriodCounter;
  uint16_t hMaxPeriodsNumber;  

  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  pHandle->wPhaseOffset = 0u;
  pHandle->bIndex=0u;

  /* Disable distortion*/
  pHandle->hFlags &= (~DSTEN); /*DSTEN cleared*/

  /* It forces inactive level on TIMx CHy and CHyN */
  TIMx->CCER &= (~TIMxCCER_MASK_CH123);

  /* Offset calibration  */
  /* Change function to be executed in ADCx_ISR */ 
  pHandle->_Super.pFctGetPhaseCurrents = &R1F30X_HFCurrentsCalibration;

  R1F30X_SwitchOnPWM(&pHandle->_Super);

  /* Wait for NB_CONVERSIONS to be executed */
  hMaxPeriodsNumber=(NB_CONVERSIONS+1u)*((uint16_t)(pHandle->pParams_str->bRepetitionCounter)+1u);

  LL_TIM_ClearFlag_CC1(TIMx);
  hCalibrationPeriodCounter = 0u;
  while (pHandle->bIndex < NB_CONVERSIONS)
  {
    if (LL_TIM_IsActiveFlag_CC1(TIMx))
    {
      LL_TIM_ClearFlag_CC1(TIMx);
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

  R1F30X_SwitchOffPWM(&pHandle->_Super);

  pHandle->wPhaseOffset >>=4;

  /* Change back function to be executed in ADCx_ISR */ 
  pHandle->_Super.pFctGetPhaseCurrents = &R1F30X_GetPhaseCurrents;

  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  TIMx->CCER |= TIMxCCER_MASK_CH123;

  /* Enable distortion*/
  pHandle->hFlags |= DSTEN; /*DSTEN set*/

  R1F30X_1ShuntMotorVarsRestart(&pHandle->_Super);
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
  * @param pHdl: handler of the current instance of the PWM component
  * @retval Ia and Ib current in Curr_Components format
  */
void R1F30X_GetPhaseCurrents(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents)
{
  int32_t wAux;
  int16_t hCurrA = 0;
  int16_t hCurrB = 0;
  int16_t hCurrC = 0;
  uint8_t bCurrASamp = 0u;
  uint8_t bCurrBSamp = 0u;
  uint8_t bCurrCSamp = 0u;

  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef* ADCx = pHandle->pParams_str->ADCx;

  TIMx->CCMR1 |= CC12_PRELOAD_ENABLE_MASK;
  TIMx->CCMR2 |= CC3_PRELOAD_ENABLE_MASK;  

  /* Reset the update flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* First sampling point */
  wAux = (int32_t)(ADCx->JDR1);

  wAux -= (int32_t)(pHandle->wPhaseOffset);

  /* Check saturation */
  if (wAux > -INT16_MAX)
  {
    if (wAux < INT16_MAX)
    {
    }
    else
    {
      wAux = INT16_MAX;
    }
  }
  else
  {
    wAux = -INT16_MAX;
  }   

  switch (pHandle->sampCur1)
  {
    case SAMP_IA:
      hCurrA = (int16_t)(wAux);
      bCurrASamp = 1u;
      break;
    case SAMP_IB:
      hCurrB = (int16_t)(wAux);
      bCurrBSamp = 1u;
      break;
    case SAMP_IC:
      hCurrC = (int16_t)(wAux);
      bCurrCSamp = 1u;
      break;
    case SAMP_NIA:
      wAux = -wAux;
      hCurrA = (int16_t)(wAux);
      bCurrASamp = 1u;
      break;
    case SAMP_NIB:
      wAux = -wAux;
      hCurrB = (int16_t)(wAux);
      bCurrBSamp = 1u;
      break;
    case SAMP_NIC:
      wAux = -wAux;
      hCurrC = (int16_t)(wAux);
      bCurrCSamp = 1u;
      break;
    case SAMP_OLDA:
      hCurrA = pHandle->hCurrAOld;
      bCurrASamp = 1u;
      break;
    case SAMP_OLDB:
      hCurrB = pHandle->hCurrBOld;
      bCurrBSamp = 1u;
      break;
    default:
      break;
  }

  /* Second sampling point */
  wAux = (int32_t)(ADCx->JDR2);

  wAux -= (int32_t)(pHandle->wPhaseOffset);

  /* Check saturation */
  if (wAux > -INT16_MAX)
  {
    if (wAux < INT16_MAX)
    {
    }
    else
    {
      wAux = INT16_MAX;
    }
  }
  else
  {
    wAux = -INT16_MAX;
  }   

  switch (pHandle->sampCur2)
  {
    case SAMP_IA:
      hCurrA = (int16_t)(wAux);
      bCurrASamp = 1u;
      break;
    case SAMP_IB:
      hCurrB = (int16_t)(wAux);
      bCurrBSamp = 1u;
      break;
    case SAMP_IC:
      hCurrC = (int16_t)(wAux);
      bCurrCSamp = 1u;
      break;
    case SAMP_NIA:
      wAux = -wAux;
      hCurrA = (int16_t)(wAux);
      bCurrASamp = 1u;
      break;
    case SAMP_NIB:
      wAux = -wAux;
      hCurrB = (int16_t)(wAux);
      bCurrBSamp = 1u;
      break;
    case SAMP_NIC:
      wAux = -wAux;
      hCurrC = (int16_t)(wAux);
      bCurrCSamp = 1u;
      break;
    default:
      break;
  }

  /* Computation of the third value */
  if (bCurrASamp == 0u)
  {
    wAux = -((int32_t)(hCurrB)) -((int32_t)(hCurrC));

    /* Check saturation */
    if (wAux > -INT16_MAX)
    {
      if (wAux < INT16_MAX)
      {
      }
      else
      {
        wAux = INT16_MAX;
      }
    }
    else
    {
      wAux = -INT16_MAX;
    }  

    hCurrA = (int16_t)wAux;
  }
  if (bCurrBSamp == 0u)
  {
    wAux = -((int32_t)(hCurrA)) -((int32_t)(hCurrC));

    /* Check saturation */
    if (wAux > -INT16_MAX)
    {
      if (wAux < INT16_MAX)
      {
      }
      else
      {
        wAux = INT16_MAX;
      }
    }
    else
    {
      wAux = -INT16_MAX;
    }  

    hCurrB = (int16_t)wAux;
  }
  if (bCurrCSamp == 0u)
  {
    wAux = -((int32_t)(hCurrA)) -((int32_t)(hCurrB));

    /* Check saturation */
    if (wAux > -INT16_MAX)
    {
      if (wAux < INT16_MAX)
      {
      }
      else
      {
        wAux = INT16_MAX;
      }
    }
    else
    {
      wAux = -INT16_MAX;
    }  

    hCurrC = (int16_t)wAux;
  }

  /* hCurrA, hCurrB, hCurrC values are the sampled values */
  pHandle->hCurrAOld = hCurrA;
  pHandle->hCurrBOld = hCurrB;

  pStator_Currents->qI_Component1 = hCurrA;
  pStator_Currents->qI_Component2 = hCurrB;

  pHandle->_Super.hIa = hCurrA;
  pHandle->_Super.hIb = hCurrB;
  pHandle->_Super.hIc = -hCurrA - hCurrB;
}

/**
  * @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during
  *         calibration. It sum up injected conversion data into wPhaseCOffset
  *         to compute the offset introduced in the current feedback
  *         network. It is requied to proper configure ADC input before to enable
  *         the offset computation.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval It always returns {0,0} in Curr_Components format
  */
static void R1F30X_HFCurrentsCalibration(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents)
{
  /* Derived class members container */
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef* ADCx = pHandle->pParams_str->ADCx;

  /* Reset the update flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE(TIMx);

  if (pHandle->bIndex < NB_CONVERSIONS)
  {
    pHandle->wPhaseOffset += ADCx->JDR2;
    pHandle->bIndex++;

    /* fill the queue*/
    /* Preload Disable */
    TIMx->CCMR3 &= TIMxCCR56_PRELOAD_DISABLE_MASK;
    LL_TIM_OC_SetCompareCH5(TIMx,0);
    LL_TIM_OC_SetCompareCH6(TIMx,0);
    /* Preload enable */
    TIMx->CCMR3 |= TIMxCCR56_PRELOAD_ENABLE_MASK; 

    LL_TIM_OC_SetCompareCH5(TIMx,((uint32_t)(pHandle->Half_PWMPeriod) >> 1) + (uint32_t)(pHandle->pParams_str->hTafter));
    LL_TIM_OC_SetCompareCH6(TIMx,((uint32_t)(pHandle->Half_PWMPeriod) - 1u));
  }
}

/**
  * @brief  It turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R1F30X_TurnOnLowSides(PWMC_Handle_t *pHdl)
{  
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = true;
  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1(TIMx,0);
  LL_TIM_OC_SetCompareCH2(TIMx,0);
  LL_TIM_OC_SetCompareCH3(TIMx,0);

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);

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
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R1F30X_SwitchOnPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  LL_TIM_DisableDMAReq_CC4(TIMx);
  LL_TIM_DisableDMAReq_UPDATE(TIMx);
  pHandle->PreloadDMAy_Chx->CCR &= (uint16_t)(~DMA_CCR_EN);
  pHandle->DistortionDMAy_Chx->CCR &= (uint16_t)(~DMA_CCR_EN);
  pHandle->DistortionDMAy_Chx->CNDTR = 2u;

  /* Enables the TIMx Preload on CC1 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  /* Enables the TIMx Preload on CC2 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  /* Enables the TIMx Preload on CC3 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);

  /* TIM output trigger 2 for ADC */
  LL_TIM_SetTriggerOutput2(TIMx, LL_TIM_TRGO2_OC5_RISING_OC6_RISING);

  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx)==RESET)
  {}
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Set all duty to 50% */
  /* Set ch5 ch6 for triggering */
  /* Clear Update Flag */
  /* TIM ch4 DMA request enable */  

  pHandle->hDmaBuff[1] = pHandle->Half_PWMPeriod >> 1;
  LL_TIM_OC_SetCompareCH1(TIMx,(uint32_t)(pHandle->Half_PWMPeriod >> 1));
  LL_TIM_OC_SetCompareCH2(TIMx,(uint32_t)(pHandle->Half_PWMPeriod >> 1));
  LL_TIM_OC_SetCompareCH3(TIMx,(uint32_t)(pHandle->Half_PWMPeriod >> 1));

  while (LL_TIM_IsActiveFlag_UPDATE(TIMx)==RESET)
  {}
  /* Main PWM Output Enable */  
  LL_TIM_EnableAllOutputs(TIMx);

  LL_TIM_OC_SetCompareCH5(TIMx,(((uint32_t)(pHandle->Half_PWMPeriod >> 1)) + (uint32_t)pHandle->pParams_str->hTafter));
  LL_TIM_OC_SetCompareCH6(TIMx,(uint32_t)(pHandle->Half_PWMPeriod - 1u));

  LL_TIM_EnableDMAReq_CC4(TIMx);
  pHandle->DistortionDMAy_Chx->CCR |= DMA_CCR_EN;

  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u)
    {
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
  }
  return; 
}


/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R1F30X_SwitchOffPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef* ADCx = pHandle->pParams_str->ADCx;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  if (pHandle->BrakeActionLock == true)
  {
  }
  else
  {
    TIMx->BDTR &= ~((uint32_t)(LL_TIM_OSSI_ENABLE));  // todo
  }
  LL_TIM_DisableAllOutputs(TIMx);

  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }  

  /* disable injected end of sequence conversions interrupt */
  LL_ADC_DisableIT_JEOS(ADCx);

  /* Flushing JSQR queue of context by setting JADSTP = 1 (JQM)=1 */
  LL_ADC_INJ_StopConversion(ADCx);

  /* channel 5 and 6 Preload Disable */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH5);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH6);


  LL_TIM_OC_SetCompareCH5(TIMx,(uint32_t)(pHandle->Half_PWMPeriod + 1u));
  LL_TIM_OC_SetCompareCH6(TIMx,(uint32_t)(pHandle->Half_PWMPeriod + 1u));

  /* channel 5 and 6 Preload enable */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH5);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH6);

  /* Re-enable ADC triggering ( wADC_JSQRvalue has been computed stored
   * during the init routine ) */
  ADCx->JSQR = pHandle->wADC_JSQR;

  /*start injected conversion */
  LL_ADC_INJ_StartConversion(ADCx);

  /* clear injected end of sequence conversions flag*/
  LL_ADC_ClearFlag_JEOS(ADCx);

  /* enable injected end of sequence conversions interrupt */
  LL_ADC_EnableIT_JEOS(ADCx);

  /* Disable TIMx DMA requests enable */
  LL_TIM_DisableDMAReq_CC4(TIMx);
  LL_TIM_DisableDMAReq_UPDATE(TIMx);

  /* Disable DMA channels*/
  pHandle->PreloadDMAy_Chx->CCR &= (uint16_t)(~DMA_CCR_EN);

  return; 
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
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void *R1F30X_TIMx_UP_IRQHandler(PWMC_Handle_t *pHdl)
{
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
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
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void *R1F30X_BRK2_IRQHandler(PWMC_Handle_t *pHdl)
{
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;

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
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void *R1F30X_BRK_IRQHandler(PWMC_Handle_t *pHdl)
{
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;

  pHandle->pParams_str->TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  pHandle->OverVoltageFlag = true;
  pHandle->BrakeActionLock = true;

  return &(pHandle->_Super.bMotor);
}

/**
  * @brief  Execute a regular conversion using ADCx.
  *         The function is not re-entrant (can't executed twice at the same time)
  * @param pHdl: handler of the current instance of the PWM component
  * @param channel: channel to be sampled (value from 0:12)
  * @retval It returns converted value or oxFFFF for conversion error
  */
uint16_t R1F30X_ExecRegularConv(PWMC_Handle_t *pHdl, uint8_t bChannel)
{
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
  ADC_TypeDef* ADCx = pHandle->pParams_str->regconvADCx;

  LL_ADC_REG_SetSequencerRanks(pHandle->pParams_str->regconvADCx,
                               LL_ADC_REG_RANK_1,
                               (bChannel<<ADC_CFGR_AWD1CH_Pos));
  LL_ADC_REG_ReadConversionData12(ADCx); 
  LL_ADC_REG_StartConversion(ADCx);      

  /* Wait until end of regular conversion */
  while(LL_ADC_IsActiveFlag_EOC(ADCx) == 0u)  
  {}

  pHandle->hRegConv = LL_ADC_REG_ReadConversionData12(ADCx); 
  return (pHandle->hRegConv);
}

/**
  * @brief  It sets the specified sampling time for the specified ADC channel
  *         on ADC1. It must be called once for each channel utilized by user
  * @param  pHdl: handler of the current instance of the PWM component
  * @param  ADC channel, sampling time
  * @retval none
  */
void R1F30X_ADC_SetSamplingTime(PWMC_Handle_t *pHdl, ADConv_t ADConv_struct)
{ 
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
  uint32_t tmpreg2 = 0u;
  uint8_t ADC_Channel = ADConv_struct.Channel;
  uint8_t ADC_SampleTime = ADConv_struct.SamplTime;

  /* Channel sampling configuration */
  /* if ADC_CHANNEL_10 ... ADC_CHANNEL_18 is selected */
  if (ADC_Channel > ADC_CHANNEL_9)
  {
    uint32_t wAux,wAux2;
    /* Get the old register value */
    /* Calculate the mask to clear */
    wAux = ADC_SMPR2_SMP10;
    wAux2 = 3u * ((uint32_t)(ADC_Channel) - 10u);
    tmpreg2 =  wAux << wAux2;
    /* Clear the old channel sample time */
    pHandle->pParams_str->regconvADCx->SMPR2 &= ~tmpreg2;
    /* Calculate the mask to set */
    wAux = (uint32_t)(ADC_SampleTime);
    pHandle->pParams_str->regconvADCx->SMPR2 |=  wAux << wAux2;

  }
  else /* ADC_Channel include in ADC_Channel_[0..9] */
  {
    uint32_t wAux,wAux2;
    /* Get the old register value */
    /* Calculate the mask to clear */
    wAux = ADC_SMPR1_SMP1;
    wAux2 = 3u * ((uint32_t)(ADC_Channel) - 1u);
    tmpreg2 =  wAux << wAux2;
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
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
  *                  detected since last method call, MC_NO_FAULTS otherwise.
  */
uint16_t R1F30X_IsOverCurrentOccurred(PWMC_Handle_t *pHdl)
{
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;

  uint16_t retVal = MC_NO_FAULTS;

  if (pHandle->OverVoltageFlag == true)
  {
    retVal = MC_OVER_VOLT;
    pHandle->OverVoltageFlag = false;
  }

  if (pHandle->OverCurrentFlag == true)
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
static void R1F30X_SetAOReferenceVoltage(uint32_t DAC_Channel, uint16_t hDACVref)
{

  LL_DAC_ConvertData12LeftAligned(DAC1, DAC_Channel, hDACVref);
  LL_DAC_TrigSWConversion(DAC1, DAC_Channel);
  /* Enable DAC Channel */
  LL_DAC_Enable(DAC1, DAC_Channel);

}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif

/**
  * @brief  Implementation of the single shunt algorithm to setup the
  *         TIM1 register and DMA buffers values for the next PWM period.
  * @param  pHandle related object of class CPWMC
  * @retval uint16_t It returns MC_FOC_DURATION if the TIMx update occurs
  *         before the end of FOC algorithm else returns MC_NO_ERROR
  */
uint16_t R1F30X_CalcDutyCycles(PWMC_Handle_t *pHdl)
{
  int16_t hDeltaDuty_0;
  int16_t hDeltaDuty_1;
  uint16_t hDutyV_0 = 0u;
  uint16_t hDutyV_1 = 0u;
  uint16_t hDutyV_2 = 0u;
  uint8_t bSector;
  uint8_t bStatorFluxPos;
  uint16_t hAux;

  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  bSector = (uint8_t)(pHandle->_Super.hSector);

  if ((pHandle->hFlags & DSTEN) != 0u)
  { 
    switch (bSector)
    {
      case SECTOR_1:
        hDutyV_2 = pHandle->_Super.hCntPhA;
        hDutyV_1 = pHandle->_Super.hCntPhB;
        hDutyV_0 = pHandle->_Super.hCntPhC;
        break;
      case SECTOR_2:
        hDutyV_2 = pHandle->_Super.hCntPhB;
        hDutyV_1 = pHandle->_Super.hCntPhA;
        hDutyV_0 = pHandle->_Super.hCntPhC;
        break;
      case SECTOR_3:
        hDutyV_2 = pHandle->_Super.hCntPhB;
        hDutyV_1 = pHandle->_Super.hCntPhC;
        hDutyV_0 = pHandle->_Super.hCntPhA;
        break;
      case SECTOR_4:
        hDutyV_2 = pHandle->_Super.hCntPhC;
        hDutyV_1 = pHandle->_Super.hCntPhB;
        hDutyV_0 = pHandle->_Super.hCntPhA;
        break;
      case SECTOR_5:
        hDutyV_2 = pHandle->_Super.hCntPhC;
        hDutyV_1 = pHandle->_Super.hCntPhA;
        hDutyV_0 = pHandle->_Super.hCntPhB;
        break;
      case SECTOR_6:
        hDutyV_2 = pHandle->_Super.hCntPhA;
        hDutyV_1 = pHandle->_Super.hCntPhC;
        hDutyV_0 = pHandle->_Super.hCntPhB;
        break;
      default:
        break;
    }

    /* Compute delta duty */
    hDeltaDuty_0 = (int16_t)(hDutyV_1) - (int16_t)(hDutyV_0);
    hDeltaDuty_1 = (int16_t)(hDutyV_2) - (int16_t)(hDutyV_1);

    /* Check region */
    if ((uint16_t)hDeltaDuty_0<=pHandle->pParams_str->hTMin)
    {
      if ((uint16_t)hDeltaDuty_1<=pHandle->pParams_str->hTMin)
      {
        bStatorFluxPos = BOUNDARY_3;
      }
      else
      {
        bStatorFluxPos = BOUNDARY_2;
      }
    } 
    else 
    {
      if ((uint16_t)hDeltaDuty_1>pHandle->pParams_str->hTMin)
      {
        bStatorFluxPos = REGULAR;
      }
      else
      {
        bStatorFluxPos = BOUNDARY_1;
      }
    }

    if (bStatorFluxPos == BOUNDARY_1) /* Adjust the lower */
    {
      switch (bSector)
      {
        case SECTOR_5:
        case SECTOR_6:
          if (pHandle->_Super.hCntPhA - pHandle->pParams_str->hCHTMin - hDutyV_0 > pHandle->pParams_str->hTMin)
          {
            pHandle->bInverted_pwm_new = INVERT_A;
            pHandle->_Super.hCntPhA -=pHandle->pParams_str->hCHTMin;
            if (pHandle->_Super.hCntPhA < hDutyV_1)
            {
              hDutyV_1 = pHandle->_Super.hCntPhA;
            }
          }
          else
          {
            bStatorFluxPos = BOUNDARY_3;
            if ((pHandle->hFlags & STBD3) == 0u)
            {
              pHandle->bInverted_pwm_new = INVERT_A;
              pHandle->_Super.hCntPhA -=pHandle->pParams_str->hCHTMin;
              pHandle->hFlags |= STBD3;
            }
            else
            {
              pHandle->bInverted_pwm_new = INVERT_B;
              pHandle->_Super.hCntPhB -=pHandle->pParams_str->hCHTMin;
              pHandle->hFlags &= (~STBD3);
            }
          }
          break;
        case SECTOR_2:
        case SECTOR_1:
          if (pHandle->_Super.hCntPhB - pHandle->pParams_str->hCHTMin - hDutyV_0 > pHandle->pParams_str->hTMin)
          {
            pHandle->bInverted_pwm_new = INVERT_B;
            pHandle->_Super.hCntPhB -=pHandle->pParams_str->hCHTMin;
            if (pHandle->_Super.hCntPhB < hDutyV_1)
            {
              hDutyV_1 = pHandle->_Super.hCntPhB;
            }
          }
          else
          {
            bStatorFluxPos = BOUNDARY_3;
            if ((pHandle->hFlags & STBD3) == 0u)
            {
              pHandle->bInverted_pwm_new = INVERT_A;
              pHandle->_Super.hCntPhA -=pHandle->pParams_str->hCHTMin;
              pHandle->hFlags |= STBD3;
            }
            else
            {
              pHandle->bInverted_pwm_new = INVERT_B;
              pHandle->_Super.hCntPhB -=pHandle->pParams_str->hCHTMin;
              pHandle->hFlags &= (~STBD3);
            }
          }
          break;
        case SECTOR_4:
        case SECTOR_3:
          if (pHandle->_Super.hCntPhC - pHandle->pParams_str->hCHTMin - hDutyV_0 > pHandle->pParams_str->hTMin)
          {
            pHandle->bInverted_pwm_new = INVERT_C;
            pHandle->_Super.hCntPhC -=pHandle->pParams_str->hCHTMin;
            if (pHandle->_Super.hCntPhC < hDutyV_1)
            {
              hDutyV_1 = pHandle->_Super.hCntPhC;
            }
          }
          else
          {
            bStatorFluxPos = BOUNDARY_3;
            if ((pHandle->hFlags & STBD3) == 0u)
            {
              pHandle->bInverted_pwm_new = INVERT_A;
              pHandle->_Super.hCntPhA -=pHandle->pParams_str->hCHTMin;
              pHandle->hFlags |= STBD3;
            }
            else
            {
              pHandle->bInverted_pwm_new = INVERT_B;
              pHandle->_Super.hCntPhB -=pHandle->pParams_str->hCHTMin;
              pHandle->hFlags &= (~STBD3);
            }
          }
          break;
        default:
          break;
      }
    }
    else if (bStatorFluxPos == BOUNDARY_2) /* Adjust the middler */
    {
      switch (bSector)
      {
        case SECTOR_4:
        case SECTOR_5: /* Invert B */
          pHandle->bInverted_pwm_new = INVERT_B;
          pHandle->_Super.hCntPhB -=pHandle->pParams_str->hCHTMin;
          if (pHandle->_Super.hCntPhB > 0xEFFFu)
          {
            pHandle->_Super.hCntPhB = 0u;
          }
          break;
        case SECTOR_2:
        case SECTOR_3: /* Invert A */
          pHandle->bInverted_pwm_new = INVERT_A;
          pHandle->_Super.hCntPhA -=pHandle->pParams_str->hCHTMin;
          if (pHandle->_Super.hCntPhA > 0xEFFFu)
          {
            pHandle->_Super.hCntPhA = 0u;
          }
          break;
        case SECTOR_6:
        case SECTOR_1: /* Invert C */
          pHandle->bInverted_pwm_new = INVERT_C;
          pHandle->_Super.hCntPhC -=pHandle->pParams_str->hCHTMin;
          if (pHandle->_Super.hCntPhC > 0xEFFFu)
          {
            pHandle->_Super.hCntPhC = 0u;
          }
          break;
        default:
          break;
      }
    }
    else if (bStatorFluxPos == BOUNDARY_3)
    {
      if ((pHandle->hFlags & STBD3) == 0u)
      {
        pHandle->bInverted_pwm_new = INVERT_A;
        pHandle->_Super.hCntPhA -=pHandle->pParams_str->hCHTMin;
        pHandle->hFlags |= STBD3;
      } 
      else
      {
        pHandle->bInverted_pwm_new = INVERT_B;
        pHandle->_Super.hCntPhB -=pHandle->pParams_str->hCHTMin;
        pHandle->hFlags &= (~STBD3);
      }
    }
    else
    {
    }

    if (bStatorFluxPos == REGULAR) /* Regular zone */
    {
      /* First point */
      pHandle->hCntSmp1 = hDutyV_1 - pHandle->pParams_str->hTbefore;

      /* Second point */
      pHandle->hCntSmp2 = hDutyV_2 - pHandle->pParams_str->hTbefore;
    }

    if (bStatorFluxPos == BOUNDARY_1) /* Two small, one big */
    {      
      /* First point */
      pHandle->hCntSmp1 = hDutyV_1 - pHandle->pParams_str->hTbefore;

      /* Second point */
      pHandle->hCntSmp2 = pHandle->Half_PWMPeriod - pHandle->pParams_str->hHTMin + pHandle->pParams_str->hTSample;
    }

    if (bStatorFluxPos == BOUNDARY_2) /* Two big, one small */
    {
      /* First point */
      pHandle->hCntSmp1 = hDutyV_2 - pHandle->pParams_str->hTbefore;

      /* Second point */
      pHandle->hCntSmp2 = pHandle->Half_PWMPeriod - pHandle->pParams_str->hHTMin + pHandle->pParams_str->hTSample;
    }

    if (bStatorFluxPos == BOUNDARY_3)  
    {
      /* First point */
      pHandle->hCntSmp1 = hDutyV_0-pHandle->pParams_str->hTbefore; /* Dummy trigger */
      /* Second point */
      pHandle->hCntSmp2 = pHandle->Half_PWMPeriod - pHandle->pParams_str->hHTMin + pHandle->pParams_str->hTSample;
    }
  }
  else
  {
    pHandle->bInverted_pwm_new = INVERT_NONE;
    bStatorFluxPos = REGULAR;
  }

  /* Update Timer Ch4 for active vector*/  
  /* Update Timer Ch 5,6 for ADC triggering and books the queue*/
  TIMx->CCMR3 &= TIMxCCR56_PRELOAD_DISABLE_MASK;
  TIMx->CCR5 = 0x0u;
  TIMx->CCR6 = 0xFFFFu;
  TIMx->CCMR3 |= TIMxCCR56_PRELOAD_ENABLE_MASK; 

  TIMx->CCR5 = pHandle->hCntSmp1;
  TIMx->CCR6 = pHandle->hCntSmp2;

  if (bStatorFluxPos == REGULAR)
  {
    /*LL_TIM_SetTriggerOutput2(TIMx, LL_TIM_TRGO2_OC5_RISING_OC6_RISING); */
    TIMx->CR2 &= ((uint32_t)0xFFEFFFFFu);

    switch (pHandle->bInverted_pwm_new)
    {
      case INVERT_A:
        pHandle->hDmaBuff[1] = pHandle->_Super.hCntPhA;

        break;
      case INVERT_B:
        pHandle->hDmaBuff[1] = pHandle->_Super.hCntPhB;

        break;
      case INVERT_C:
        pHandle->hDmaBuff[1] = pHandle->_Super.hCntPhC;

        break;
      default:
        break;
    }
  }
  else
  {
    /* disable DMA request update interrupt */
    LL_TIM_DisableDMAReq_UPDATE(TIMx);


    /* Set the DMA destinations */
    switch (pHandle->bInverted_pwm_new)
    {
      case INVERT_A:
        pHandle->DistortionDMAy_Chx->CPAR = (uint32_t)(&(TIMx->CCR1));
        pHandle->PreloadDMAy_Chx->CPAR = (uint32_t)(&(TIMx->CCMR1));
        pHandle->hDmaBuff[1] = pHandle->_Super.hCntPhA;
        pHandle->wPreloadDisableActing = pHandle->wPreloadDisableCC1;
        break;

      case INVERT_B:
        pHandle->DistortionDMAy_Chx->CPAR = (uint32_t)(&(TIMx->CCR2));
        pHandle->PreloadDMAy_Chx->CPAR = (uint32_t)(&(TIMx->CCMR1));
        pHandle->hDmaBuff[1] = pHandle->_Super.hCntPhB;
        pHandle->wPreloadDisableActing = pHandle->wPreloadDisableCC2;
        break;

      case INVERT_C:
        pHandle->DistortionDMAy_Chx->CPAR = (uint32_t)(&(TIMx->CCR3));
        pHandle->PreloadDMAy_Chx->CPAR = (uint32_t)(&(TIMx->CCMR2));
        pHandle->hDmaBuff[1] = pHandle->_Super.hCntPhC;
        pHandle->wPreloadDisableActing = pHandle->wPreloadDisableCC3;
        break;

      default:
        break;
    } 

    /*TIM_SelectOutputTrigger2(TIMx, TIM_TRGO2Source_OC5RefRising_OC6RefFalling); */
    TIMx->CR2 |= ((uint32_t)0x100000u);

    /*active vector*/
    pHandle->PreloadDMAy_Chx->CCR &= (uint16_t)(~DMA_CCR_EN);
    pHandle->PreloadDMAy_Chx->CNDTR = 1u;
    pHandle->PreloadDMAy_Chx->CCR |= DMA_CCR_EN;

    /* enable DMA request update interrupt */
    LL_TIM_EnableDMAReq_UPDATE(TIMx);
  }

  pHandle->pParams_str->ADCx->JSQR = pHandle->wADC_JSQR;

  /* Update Timer Ch 1,2,3 (These value are required before update event) */
  TIMx->CCR1 = pHandle->_Super.hCntPhA;
  TIMx->CCR2 = pHandle->_Super.hCntPhB;
  TIMx->CCR3 = pHandle->_Super.hCntPhC;

  /*End of FOC*/
  /*check software error*/
  if (LL_TIM_IsActiveFlag_UPDATE(TIMx))
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

  /* Set the current sampled */
  if (bStatorFluxPos == REGULAR) /* Regual zone */
  {
    pHandle->sampCur1 = REGULAR_SAMP_CUR1[bSector];
    pHandle->sampCur2 = REGULAR_SAMP_CUR2[bSector];
  }

  if (bStatorFluxPos == BOUNDARY_1) /* Two small, one big */
  {
    pHandle->sampCur1 = REGULAR_SAMP_CUR1[bSector];
    pHandle->sampCur2 = BOUNDR1_SAMP_CUR2[bSector];
  }

  if (bStatorFluxPos == BOUNDARY_2) /* Two big, one small */
  {
    pHandle->sampCur1 = BOUNDR2_SAMP_CUR1[bSector];
    pHandle->sampCur2 = BOUNDR2_SAMP_CUR2[bSector];
  }

  if (bStatorFluxPos == BOUNDARY_3)  
  {
    if (pHandle->bInverted_pwm_new == INVERT_A)
    {
      pHandle->sampCur1 = SAMP_OLDB;
      pHandle->sampCur2 = SAMP_IA;
    }
    if (pHandle->bInverted_pwm_new == INVERT_B)
    {
      pHandle->sampCur1 = SAMP_OLDA;
      pHandle->sampCur2 = SAMP_IB;
    }
  }

  /* Limit for the Get Phase current (Second EOC Handler) */

  return (hAux);
}

/**
  * @brief  It is used to set the PWM mode for R/L detection.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R1F30X_RLDetectionModeEnable(PWMC_Handle_t *pHdl)
{
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  if (pHandle->_Super.RLDetectionMode == false)
  {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
    LL_TIM_OC_SetCompareCH1(TIMx, 0u);

    /*  Channel2 configuration */
    if ((pHandle->pParams_str->LowSideOutputs) == LS_PWM_TIMER)
    {
      LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_ACTIVE);
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2);
    }
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_INACTIVE);
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2);
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
    }
    else
    {
    }

    /*  Channel3 configuration */
    LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N);
  }

  pHandle->_Super.pFctGetPhaseCurrents = &R1F30X_RLGetPhaseCurrents;
  pHandle->_Super.pFctTurnOnLowSides = &R1F30X_RLTurnOnLowSides;
  pHandle->_Super.pFctSwitchOnPwm = &R1F30X_RLSwitchOnPWM;
  pHandle->_Super.pFctSwitchOffPwm = &R1F30X_RLSwitchOffPWM;

  pHandle->_Super.RLDetectionMode = true;
}

/**
  * @brief  It is used to disable the PWM mode for R/L detection.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R1F30X_RLDetectionModeDisable(PWMC_Handle_t *pHdl)
{
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  if (pHandle->_Super.RLDetectionMode == true)
  {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);

    if ((pHandle->pParams_str->LowSideOutputs)== LS_PWM_TIMER)
    {
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
    }
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
    }
    else
    {}

    LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);

    /*  Channel2 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2);

    if ((pHandle->pParams_str->LowSideOutputs)== LS_PWM_TIMER)
    {
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
    }
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
    }
    else
    {}

    LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);

    /*  Channel3 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3);

    if ((pHandle->pParams_str->LowSideOutputs)== LS_PWM_TIMER)
    {
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3N);
    }
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3N);
    }
    else
    {}

    LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);

    pHandle->_Super.pFctGetPhaseCurrents = &R1F30X_GetPhaseCurrents;
    pHandle->_Super.pFctTurnOnLowSides = &R1F30X_TurnOnLowSides;
    pHandle->_Super.pFctSwitchOnPwm = &R1F30X_SwitchOnPWM;
    pHandle->_Super.pFctSwitchOffPwm = &R1F30X_SwitchOffPWM;

    pHandle->_Super.RLDetectionMode = false;
  }
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif

/**
  * @brief  It is used to set the PWM dutycycle for R/L detection
  * @param  pHdl: handler of the current instance of the PWM component
  * @param  hDuty to be applied in uint16_t
  * @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR'
  *         otherwise. These error codes are defined in mc_type.h
  */
uint16_t R1F30X_RLDetectionModeSetDuty(PWMC_Handle_t *pHdl, uint16_t hDuty)
{
  uint16_t hAux;
  uint32_t val;
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef* ADCx = pHandle->pParams_str->ADCx;

  val = ((uint32_t)(pHandle->Half_PWMPeriod) * (uint32_t)(hDuty)) >> 16;
  pHandle->_Super.hCntPhA = (uint16_t)(val);

  LL_TIM_OC_SetCompareCH1(TIMx, pHandle->_Super.hCntPhA);

  ADCx->JSQR = pHandle->wADC_JSQR & (uint32_t)(0xFFFFFFFCu); /* Re enable ADC trig 1 trigger*/

  /* Limit for update event */
  /* Check the status flag. If an update event has occurred before to set new
  values of regs the FOC rate is too high */
  if (LL_TIM_IsActiveFlag_UPDATE(TIMx))
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
  * @brief  It computes and return latest converted motor phase currents motor
  *         during RL detection phase
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval Ia and Ib current in Curr_Components format
  */
static void R1F30X_RLGetPhaseCurrents(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents)
{
  int32_t wAux;
  int16_t hCurrA = 0;
  int16_t hCurrB = 0;

  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
  ADC_TypeDef* ADCx = pHandle->pParams_str->ADCx;

  /* Reset the update flag to indicate the start of algorithm*/
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->TIMx);

  wAux = LL_ADC_INJ_ReadConversionData32(ADCx, LL_ADC_INJ_RANK_1);

  wAux -= (int32_t)(pHandle->wPhaseOffset);

  /* Check saturation */
  if (wAux > -INT16_MAX)
  {
    if (wAux < INT16_MAX)
    {
    }
    else
    {
      wAux = INT16_MAX;
    }
  }
  else
  {
    wAux = -INT16_MAX;
  }

  hCurrA = (int16_t)(wAux);
  hCurrB = -hCurrA;

  pStator_Currents->qI_Component1 = -hCurrA;
  pStator_Currents->qI_Component2 = hCurrB;
}

/**
  * @brief  It turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers.
  *         This function is specific for RL detection phase.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
static void R1F30X_RLTurnOnLowSides(PWMC_Handle_t *pHdl)
{  
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  /*Turn on the phase A low side switch */
  LL_TIM_OC_SetCompareCH1(TIMx, 0u);

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Wait until next update */
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == RESET)
  {}

  /* Main PWM Output Enable */
  TIMx->BDTR |= TIM_BDTR_MOE;

  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  return;
}

/**
  * @brief  It enables PWM generation on the proper Timer peripheral
  *         This function is specific for RL detection phase.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
static void R1F30X_RLSwitchOnPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  /* Enables the TIMx Preload on CC1 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  /* Enables the TIMx Preload on CC2 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);

  /* TIM output trigger 2 for ADC */
  LL_TIM_SetTriggerOutput2(TIMx, LL_TIM_TRGO2_UPDATE);

  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while ((LL_TIM_IsActiveFlag_UPDATE(TIMx)) == RESET)
  {}
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Set duty to 0% */
  LL_TIM_OC_SetCompareCH1(TIMx, 0u);

  while ((LL_TIM_IsActiveFlag_UPDATE(TIMx)) == RESET)
  {}

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  LL_TIM_EnableAllOutputs(TIMx);

  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  return;
}


/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit
  *         This function is specific for RL detection phase.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
static void R1F30X_RLSwitchOffPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R1_F3_Handle_t *pHandle = (PWMC_R1_F3_Handle_t *)pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef* ADCx = pHandle->pParams_str->ADCx;

  /* Main PWM Output Disable */
  if (pHandle->BrakeActionLock == true)
  {
  }
  else
  {
    TIMx->BDTR &= ~((uint32_t)(LL_TIM_OSSI_ENABLE));
  }
  LL_TIM_DisableAllOutputs(TIMx);

  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }

  /* TIM output trigger 2 for ADC */
  LL_TIM_SetTriggerOutput2(TIMx, LL_TIM_TRGO2_RESET);

  LL_ADC_DisableIT_JEOS(ADCx);

  /* Flushing JSQR queue of context by setting JADSTP = 1 (JQM)=1 */
  LL_ADC_INJ_StopConversion(ADCx);

  /* Re-enable ADC triggering*/
  ADCx->JSQR = pHandle->wADC_JSQR;
  /*ADC_StartInjectedConversion(ADCx);*/
  LL_ADC_INJ_StartConversion(ADCx);
  /*ADC_ClearFlag(ADCx, ADC_FLAG_JEOS);*/
  LL_ADC_ClearFlag_JEOS(ADCx);

  LL_ADC_EnableIT_JEOS(ADCx);

  return;
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
