/**
  ******************************************************************************
  * @file    r3_1_f30x_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement current sensor
  *          class to be stantiated when the three shunts current sensing
  *          topology is used. It is specifically designed for STM32F302x8
  *          microcontrollers.
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
#include "r3_1_f30x_pwm_curr_fdbk.h"
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
 * @defgroup r3_1_f30X_pwm_curr_fdbk R3 1 ADC F30x PWM & Current Feedback
 *
 * @brief STM32F3, 1 ADC, 3-Shunt PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F3 MCU, using a three
 * shunt resistors current sensing topology and only one ADC to acquire the current
 * values. This is typically the implementation to use with STM32F301xx and STM32F302xx
 * MCUs that only have one ADC peripheral.
 *
 * It computes the PWM duty cycles on each PWM period, applies them to the Motor phases and reads
 * the current flowing through the Motor phases. It is built on the @ref pwm_curr_fdbk component.
 *
 * Instances of this component are managed by a PWMC_R3_1_F3_Handle_t Handle structure that needs
 * to be initialized with the R3_1_F30X_Init() prior to being used.
 *
 * Usually, the R3_1_F30X_Init() function i the only function of this component that needs to be
 * called directly. Its other functions are usually invoked by functions of the @ref pwm_curr_fdbk
 * base component.
 *
 * @section r3_1_f30x_periph_usage Peripheral usage
 * The PWMC_R3_1_F3 uses the following IPs of the STM32 MCU....
 *
 * * 1 Advanced Timer: 3 PWM channels, their output pins and optionally their complemented output
 * * 1 ADC: Three channels of the ADC are used. The ADC to choose can be triggered by the Timer....
 * * ...
 *
 * @todo: TODO: complete documentation.
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/
#define TIMxCCER_MASK_CH123        ((uint32_t)  0x00000555u)
#define NB_CONVERSIONS 16u
#define CCMR2_CH4_DISABLE 0x8FFFu
#define CCMR2_CH4_PWM1    0x6000u
#define CCMR2_CH4_PWM2    0x7000u
/* DIR bits of TIM1 CR1 register identification for correct check of Counting direction detection*/
#define DIR_MASK 0x0010u       /* binary value: 0000000000010000 */
/* JSQR register Mask */
#define JSQR_CLEAR_Mask             ((uint32_t)0x00000000)
#define JSQR_EDGE_CLEAR_Mask        (~(uint32_t) LL_ADC_INJ_TRIG_EXT_RISINGFALLING)            
        
/* Private typedef -----------------------------------------------------------*/


/** 
  * @brief  ADC Init structure definition  
  */
typedef struct
{

   uint32_t ADC_ExternalTrigInjecConvEvent;     /*!< Defines the external trigger used to start the analog
                                                     to digital conversion of injected channels. This parameter
                                                     can be a value of @ref ADC_external_trigger_sources_for_Injected_channels_conversion */
  uint32_t ADC_ExternalTrigInjecEventEdge;     /*!< Select the external trigger edge and enable the trigger of an injected group. 
                                                    This parameter can be a value of 
                                                    @ref ADC_external_trigger_edge_for_Injected_channels_conversion */
  uint8_t ADC_NbrOfInjecChannel;               /*!< Specifies the number of ADC channels that will be converted
                                                    using the sequencer for injected channel group.
                                                    This parameter must range from 1 to 4. */ 
  uint32_t ADC_InjecSequence1; 
  uint32_t ADC_InjecSequence2;
  uint32_t ADC_InjecSequence3;
  uint32_t ADC_InjecSequence4;                                            
}ADC_InjectedInitTypeDef;
#define ADC_EXTERNALTRIGINJECTEVENT    LL_ADC_INJ_TRIG_EXT_TIM1_TRGO  /*!<  ADC external trigger for injected conversion event 0 */
#define ADC_EXTERNALTRIGINJECTEDGE     LL_ADC_INJ_TRIG_EXT_RISING /*!<  ADC external trigger rising edge for injected conversion */

/* Private function prototypes -----------------------------------------------*/
static void R3_1_F30X_HFCurrentsCalibrationAB(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents);
static void R3_1_F30X_HFCurrentsCalibrationC(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents);
static void R3_1_F3XX_StartTimers(void);
static uint16_t R3_1_F30X_WriteTIMRegisters(PWMC_Handle_t *pHdl);
static uint32_t SingleADC_InjectedConfig(ADC_TypeDef* ADCx,
                                         ADC_InjectedInitTypeDef* ADC_InjectedInitStruct);


/**
  * @brief  It initializes peripherals for current reading and PWM generation
  *         in three shunts configuration using STM32F302x8
  * @param  pHandle: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_F30X_Init(PWMC_R3_1_F3_Handle_t *pHandle)
{
  COMP_TypeDef* COMP_OCPAx = pHandle->pParams_str->wCompOCPASelection;
  COMP_TypeDef* COMP_OCPBx = pHandle->pParams_str->wCompOCPBSelection;
  COMP_TypeDef* COMP_OCPCx = pHandle->pParams_str->wCompOCPCSelection;
  COMP_TypeDef* COMP_OVPx = pHandle->pParams_str->wCompOVPSelection;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef* ADCx  = pHandle->pParams_str->ADCx;
  ADC_InjectedInitTypeDef ADC_InjectedInitStruct;

  if ((uint32_t)pHandle == (uint32_t)&pHandle->_Super)
  {

    /* disable main TIM counter to ensure
     * a synchronous start by TIM2 trigger */
    LL_TIM_DisableCounter(TIMx);

    /* Over current protection phase A */
    if (COMP_OCPAx)
    {
      /* Inverting input*/
      if (pHandle->pParams_str->bCompOCPAInvInput_MODE != EXT_MODE)
      {
        if (LL_COMP_GetInputMinus(COMP_OCPAx) == LL_COMP_INPUT_MINUS_DAC1_CH1)
        {
          LL_DAC_ConvertData12LeftAligned (DAC1, LL_DAC_CHANNEL_1, (uint16_t)(pHandle->pParams_str->hDAC_OCP_Threshold) );
          LL_DAC_TrigSWConversion (DAC1, LL_DAC_CHANNEL_1);
          LL_DAC_Enable (DAC1, LL_DAC_CHANNEL_1 );
        }
#if defined(DAC_CHANNEL2_SUPPORT)
        else if (LL_COMP_GetInputMinus(COMP_OCPAx) == LL_COMP_INPUT_MINUS_DAC1_CH2)
        {
          LL_DAC_ConvertData12LeftAligned (DAC1, LL_DAC_CHANNEL_2, (uint16_t)(pHandle->pParams_str->hDAC_OCP_Threshold) );
          LL_DAC_TrigSWConversion (DAC1, LL_DAC_CHANNEL_2);
          LL_DAC_Enable (DAC1, LL_DAC_CHANNEL_2 );
        }
#endif
        else
        {/* in case of internal reference, nothing to do, Vref is configured during 
	        MX_COMP_Init intialization phase */	
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
          LL_DAC_ConvertData12LeftAligned (DAC1, LL_DAC_CHANNEL_1, (uint16_t)(pHandle->pParams_str->hDAC_OVP_Threshold) );
          LL_DAC_TrigSWConversion (DAC1, LL_DAC_CHANNEL_1);
          LL_DAC_Enable (DAC1, LL_DAC_CHANNEL_1 );
        }
#if defined(DAC_CHANNEL2_SUPPORT)
        else if (LL_COMP_GetInputMinus(COMP_OVPx) == LL_COMP_INPUT_MINUS_DAC1_CH2)
        {
          LL_DAC_ConvertData12LeftAligned (DAC1, LL_DAC_CHANNEL_2, (uint16_t)(pHandle->pParams_str->hDAC_OVP_Threshold) );
          LL_DAC_TrigSWConversion (DAC1, LL_DAC_CHANNEL_2);
          LL_DAC_Enable (DAC1, LL_DAC_CHANNEL_2 );
        }
#endif
        else
        {/* in case of internal reference, nothing to do, Vref is configured during 
	        MX_COMP_Init intialization phase */			
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


    LL_TIM_ClearFlag_BRK(TIMx);

    if ((pHandle->pParams_str->bBKIN2Mode) != NONE)
    {
      LL_TIM_ClearFlag_BRK2(TIMx);
    }
    LL_TIM_EnableIT_BRK(TIMx);

    LL_ADC_EnableInternalRegulator(ADCx);

    /* Wait for Regulator Startup time, once for both */
    {
      volatile uint16_t waittime = 0u;
      for(waittime=0u;waittime<65000u;waittime++)
      {
        waittime=waittime;
      }
    }

    LL_ADC_StartCalibration(ADCx, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADCx))
    {
    }

    /* ADC Enable (must be done after calibration) */
    LL_ADC_Enable(ADCx);
    /* Configuration of ADC sequence of two currents for the future JSQR register setting*/
    ADC_InjectedInitStruct.ADC_ExternalTrigInjecConvEvent = ADC_EXTERNALTRIGINJECTEVENT;
    ADC_InjectedInitStruct.ADC_ExternalTrigInjecEventEdge = ADC_EXTERNALTRIGINJECTEDGE;
    ADC_InjectedInitStruct.ADC_NbrOfInjecChannel = 2u;

    /*AB currents sequence --------------------------------------------------------------------------------*/
    ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIaChannel;
    ADC_InjectedInitStruct.ADC_InjecSequence2 = pHandle->pParams_str->bIbChannel;
    ADC_InjectedInitStruct.ADC_InjecSequence3 = 0u;
    ADC_InjectedInitStruct.ADC_InjecSequence4 = 0u;

    pHandle->wADC_JSQR_phAB= SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);

    /*BA currents sequence --------------------------------------------------------------------------------*/
    ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIbChannel;
    ADC_InjectedInitStruct.ADC_InjecSequence2 = pHandle->pParams_str->bIaChannel;

    pHandle->wADC_JSQR_phBA= SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);

    /*AC currents sequence --------------------------------------------------------------------------------*/
    ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIaChannel;
    ADC_InjectedInitStruct.ADC_InjecSequence2 =pHandle->pParams_str->bIcChannel;

    pHandle->wADC_JSQR_phAC= SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);

    /*CA currents sequence --------------------------------------------------------------------------------*/
    ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIcChannel;
    ADC_InjectedInitStruct.ADC_InjecSequence2 = pHandle->pParams_str->bIaChannel;

    pHandle->wADC_JSQR_phCA= SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);

    /*BC currents sequence --------------------------------------------------------------------------------*/
    ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIbChannel;
    ADC_InjectedInitStruct.ADC_InjecSequence2 = pHandle->pParams_str->bIcChannel;

    pHandle->wADC_JSQR_phBC= SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);

    /*CB currents sequence -------------------------------------------------------------------------------- */
    ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIcChannel;
    ADC_InjectedInitStruct.ADC_InjecSequence2 = pHandle->pParams_str->bIbChannel;

    pHandle->wADC_JSQR_phCB= SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);
    /* ---------------------------------------------------------------------------------------------------- */

    /* Configuration of ADC single sequence of single current for the future JSQR register setting*/

    /* Single Phase A current acquisition configuration ------------------------------------------*/
    ADC_InjectedInitStruct.ADC_NbrOfInjecChannel =1u;
    ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIaChannel;
    ADC_InjectedInitStruct.ADC_InjecSequence2 = 0u;
    ADC_InjectedInitStruct.ADC_InjecSequence3 = 0u;
    ADC_InjectedInitStruct.ADC_InjecSequence4 = 0u;

    pHandle->wADC_JSQR_phA= SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);

    /* Single Phase B current acquisition configuration ------------------------------------------*/
    ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIbChannel;

    pHandle->wADC_JSQR_phB= SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);

    /* Single Phase C current acquisition configuration ------------------------------------------*/
    ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIcChannel;

    pHandle->wADC_JSQR_phC= SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);

    /* Queue Of Context Mode for injected channels Enabling */
    ADC_InjectedInitStruct.ADC_NbrOfInjecChannel =2u;
    ADC_InjectedInitStruct.ADC_InjecSequence1 = 0u;
    ADCx->JSQR = SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);
    ADCx->CR |= ADC_CR_JADSTART;

    LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);

    /* Fictitious ADCx Trigger to start ADCx Conversion. */
    TIMx->CCR4 = 0xFFFFu;
    TIMx->CCR4 = 0x0u;

    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);

    /* ADC_FLAG_JEOS wait if is RESET. Wait the end of ADCx injected conversion sequence.  */
    while (LL_ADC_IsActiveFlag_JEOS(ADCx) == 0)
    {
    }

    /* ADCx Injected conversions end interrupt enabling */
    LL_ADC_ClearFlag_JEOS(ADCx);
    LL_ADC_EnableIT_JEOS(ADCx);


    /* Clear the flags */
    pHandle->OverVoltageFlag = false;
    pHandle->OverCurrentFlag = false;

    pHandle->_Super.DTTest = 0u;
    pHandle->_Super.DTCompCnt = pHandle->_Super.hDTCompCnt;

    /* Enable Counter */
    R3_1_F3XX_StartTimers();

  }
}

/**
  * @brief  It perform the start of all the timers required by the control.
  *          It utilizes TIM2 as temporary timer to achieve synchronization between
  *          PWM signals.
  *          When this function is called, TIM1 and/or TIM8 must be in frozen state
  *          with CNT, ARR, REP RATE and trigger correctly set (these setting are
  *          usually performed in the Init method accordingly with the configuration)
  * @param  none
  * @retval none
  */
 static void R3_1_F3XX_StartTimers(void)
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
  * @brief  It measures and stores into handler component variables the offset voltage on Ia and
  *         Ib current feedback analog channels when no current is flowing into the
  *         motor
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_F30X_CurrentReadingCalibration(PWMC_Handle_t *pHdl)
{
  uint16_t hCalibrationPeriodCounter;
  uint16_t hMaxPeriodsNumber;
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  pHandle->wPhaseAOffset = 0u;
  pHandle->wPhaseBOffset = 0u;
  pHandle->wPhaseCOffset = 0u;
  
  pHandle->bIndex=0u;
  
  /* It forces inactive level on TIMx CHy and CHyN */
  TIMx->CCER &= (~TIMxCCER_MASK_CH123);
   
  /* Offset calibration for A & B phases */
  /* Change function to be executed in ADCx_ISR */ 
  pHandle->_Super.pFctGetPhaseCurrents = &R3_1_F30X_HFCurrentsCalibrationAB;
  pHandle->_Super.pFctSetADCSampPointSect1 = &R3_1_F30X_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect2 = &R3_1_F30X_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect3 = &R3_1_F30X_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect4 = &R3_1_F30X_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect5 = &R3_1_F30X_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect6 = &R3_1_F30X_SetADCSampPointCalibration;
  
  pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB;
  
  R3_1_F30X_SwitchOnPWM(&pHandle->_Super);
  
  /* Wait for NB_CONVERSIONS to be executed */
  hMaxPeriodsNumber=(NB_CONVERSIONS+1u)*(((uint16_t)(pHandle->pParams_str->bRepetitionCounter)+1u)>>1);
  TIMx->SR = (uint16_t)~LL_TIM_SR_CC1IF;
  hCalibrationPeriodCounter = 0u;
  while (pHandle->bIndex < NB_CONVERSIONS)
  {
    if (TIMx->SR & LL_TIM_SR_CC1IF)
    {
      TIMx->SR = (uint16_t)~LL_TIM_SR_CC1IF;
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
  
  R3_1_F30X_SwitchOffPWM(&pHandle->_Super);

  /* Offset calibration for C phase */
  /* Reset bIndex */
  pHandle->bIndex=0u;

  /* Change function to be executed in ADCx_ISR */ 
  pHandle->_Super.pFctGetPhaseCurrents = &R3_1_F30X_HFCurrentsCalibrationC;

/* "Phase C current calibration to verify"    */
  pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phC;
  
  R3_1_F30X_SwitchOnPWM(&pHandle->_Super);
  
  /* Wait for NB_CONVERSIONS to be executed */
  TIMx->SR = (uint16_t)~LL_TIM_SR_CC1IF;
  hCalibrationPeriodCounter = 0u;
  while (pHandle->bIndex < NB_CONVERSIONS)
  {
    if (TIMx->SR & LL_TIM_SR_CC1IF)
    {
      TIMx->SR = (uint16_t)~LL_TIM_SR_CC1IF;
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
  
  R3_1_F30X_SwitchOffPWM(&pHandle->_Super);
  
  /* Shift of N bits to divide for the NB_ CONVERSIONS = 16= 2^N with N = 4 */
  pHandle->wPhaseAOffset >>=4;
  pHandle->wPhaseBOffset >>=4;
  pHandle->wPhaseCOffset >>=4;

  /* Change back function to be executed in ADCx_ISR */ 
  pHandle->_Super.pFctGetPhaseCurrents = &R3_1_F30X_GetPhaseCurrents;
  pHandle->_Super.pFctSetADCSampPointSect1 = &R3_1_F30X_SetADCSampPointSect1;
  pHandle->_Super.pFctSetADCSampPointSect2 = &R3_1_F30X_SetADCSampPointSect2;
  pHandle->_Super.pFctSetADCSampPointSect3 = &R3_1_F30X_SetADCSampPointSect3;
  pHandle->_Super.pFctSetADCSampPointSect4 = &R3_1_F30X_SetADCSampPointSect4;
  pHandle->_Super.pFctSetADCSampPointSect5 = &R3_1_F30X_SetADCSampPointSect5;
  pHandle->_Super.pFctSetADCSampPointSect6 = &R3_1_F30X_SetADCSampPointSect6;
  
  /* To program the first samplig at the next switch on PWM */
  pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB;

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
  TIMx->CCER |= TIMxCCER_MASK_CH123;
  
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
  * @param pHdl: handler of the current instance of the PWM component
  * @retval Ia and Ib current in Curr_Components format
  */
void R3_1_F30X_GetPhaseCurrents(PWMC_Handle_t *pHdl, Curr_Components* pStator_Currents)
{
  uint8_t bSector;
  int32_t wAux;
  uint16_t hReg1;
  uint16_t hReg2;
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;

  /* Clear the flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->TIMx);
  
  hReg1 = (uint16_t)(pHandle->pParams_str->ADCx->JDR1);
  hReg2 = (uint16_t)(pHandle->pParams_str->ADCx->JDR2);
  
  bSector = (uint8_t)(pHandle->_Super.hSector);
  
  switch (bSector)
  {
  case SECTOR_4:
  case SECTOR_5:
    {
      /* Current on Phase C is not accessible     */
      
      /* Ia = PhaseAOffset - ADC converted value) */
      if(bSector == SECTOR_4)
      {
        wAux = (int32_t)(pHandle->wPhaseAOffset)-(int32_t)(hReg2);
      }
      else
      {
        wAux = (int32_t)(pHandle->wPhaseAOffset)-(int32_t)(hReg1);
      }
      
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
      if(bSector == SECTOR_4)
      {
        wAux = (int32_t)(pHandle->wPhaseBOffset)-(int32_t)(hReg1);
      }
      else
      {
        wAux = (int32_t)(pHandle->wPhaseBOffset)-(int32_t)(hReg2);
      }
      
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
    }
    break;
    
  case SECTOR_6:
  case SECTOR_1:
    {
      /* Current on Phase A is not accessible     */
      
      /* Ib = PhaseBOffset - ADC converted value) */
      if(bSector == SECTOR_6)
      {
        wAux = (int32_t)(pHandle->wPhaseBOffset)-(int32_t)(hReg2);
      }
      else
      {
        wAux = (int32_t)(pHandle->wPhaseBOffset)-(int32_t)(hReg1);
      }
      
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
      
      /* Ic = PhaseCOffset - ADC converted value) */
      /* Ia = -Ic -Ib */
      if(bSector == SECTOR_6)
      {
        wAux = (int32_t)(pHandle->wPhaseCOffset)-(int32_t)(hReg1);
      }
      else
      {
        wAux = (int32_t)(pHandle->wPhaseCOffset)-(int32_t)(hReg2);
      }
      
      wAux = -wAux - (int32_t)pStator_Currents->qI_Component2;
      
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
    }
    break;
    
  case SECTOR_2:
  case SECTOR_3:
    {
      /* Current on Phase B is not accessible     */
      
      /* Ia = PhaseAOffset - ADC converted value) */
      if(bSector == SECTOR_3)
      {
        wAux = (int32_t)(pHandle->wPhaseAOffset)-(int32_t)(hReg2);
      }
      else
      {
        wAux = (int32_t)(pHandle->wPhaseAOffset)-(int32_t)(hReg1);
      }
      
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
      
      /* Ic = PhaseCOffset - ADC converted value) */
      /* Ib = -Ic -Ia */
      if(bSector == SECTOR_3)
      {
        wAux = (int32_t)(pHandle->wPhaseCOffset)-(int32_t)(hReg1);
      }
      else
      {
        wAux = (int32_t)(pHandle->wPhaseCOffset)-(int32_t)(hReg2);
      }
      
      wAux = -wAux -  (int32_t)pStator_Currents->qI_Component1;
      
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
    }
    break;
    
  default:
    {
    }
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
  * @param pHdl: handler of the current instance of the PWM component
  * @retval It always returns {0,0} in Curr_Components format
  */
static void R3_1_F30X_HFCurrentsCalibrationAB(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents)
{  
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  /* Clear the flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->TIMx);
  
  if (pHandle->bIndex < NB_CONVERSIONS)
  {
    pHandle->wPhaseAOffset += pHandle->pParams_str->ADCx->JDR1;
    pHandle->wPhaseBOffset += pHandle->pParams_str->ADCx->JDR2;
    pHandle->bIndex++;
  }
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
static void R3_1_F30X_HFCurrentsCalibrationC(PWMC_Handle_t *pHdl, Curr_Components* pStator_Currents)
{
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  /* Clear the flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->TIMx);
  
  if (pHandle->bIndex < NB_CONVERSIONS)
  {
    pHandle-> wPhaseCOffset += pHandle->pParams_str->ADCx->JDR1;
    pHandle->bIndex++;
  }
}

/**
  * @brief  It turns on low sides switches. This function is intended to be 
  *         used for charging boot capacitors of driving section. It has to be 
  *         called each motor start-up when using high voltage drivers
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_F30X_TurnOnLowSides(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = true;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  
  /*Turn on the three low side switches */
  TIMx->CCR1 = 0u;
  TIMx->CCR2 = 0u;
  TIMx->CCR3 = 0u;
  
  /* Wait until next update */
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0)
  {}
  
  /* Main PWM Output Enable */
  TIMx->BDTR |= TIM_BDTR_MOE;
  
  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    /* Enable signals activation */
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
void R3_1_F30X_SwitchOnPWM(PWMC_Handle_t *pHdl)
{  
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0)
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  
  /* Set all duty to 50% */
  if (pHandle->_Super.RLDetectionMode == true)
  {
    TIMx->CCR1 = 1u;
    pHandle->pParams_str->ADCx->JSQR = pHandle->wADC1_JSQR;
  }
  else
  {
    TIMx->CCR1 = (uint32_t)(pHandle->Half_PWMPeriod) >> 1;
  }
  TIMx->CCR2 = (uint32_t)(pHandle->Half_PWMPeriod) >> 1;
  TIMx->CCR3 = (uint32_t)(pHandle->Half_PWMPeriod) >> 1;
  TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 5u;
  
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0)
  {}
  
  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  TIMx->BDTR |= TIM_BDTR_MOE;
  
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
  pHandle->pParams_str->ADCx->JSQR = pHandle->wADC1_JSQR;

  return; 
}

/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_F30X_SwitchOffPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef* ADCx = pHandle->pParams_str->ADCx;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  if (pHandle->BrakeActionLock == true)
  {
  }
  else
  {
    TIMx->BDTR &= ~((uint32_t)(LL_TIM_OSSI_ENABLE));
    
    if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
  }
  TIMx->BDTR &= (uint32_t)~TIM_BDTR_MOE;
  
  LL_ADC_DisableIT_JEOS(ADCx);
  
  /* Flushing JSQR queue of context by setting JADSTP = 1 (JQM)=1 */
  ADCx->CR |= ADC_CR_JADSTP;
  ADC_InjectedInitTypeDef ADC_InjectedInitStruct;
  ADC_InjectedInitStruct.ADC_NbrOfInjecChannel = 2u;
  ADC_InjectedInitStruct.ADC_InjecSequence1 = 0u;
  ADC_InjectedInitStruct.ADC_ExternalTrigInjecEventEdge = ADC_EXTERNALTRIGINJECTEDGE;
  ADC_InjectedInitStruct.ADC_ExternalTrigInjecConvEvent = ADC_EXTERNALTRIGINJECTEVENT;
  ADCx->JSQR = SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);
  ADCx->CR |= ADC_CR_JADSTART;
  
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;
  /* Imposing of a change of state from 1 to 0 logic state*/
  TIMx->CCR4 = 0xFFFFu;
  TIMx->CCR4 = 0x0u;
	
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  
  while (LL_ADC_IsActiveFlag_JEOS(ADCx) == 0)
  {}
  /* ADCx Injected conversions end interrupt enabling */
  LL_ADC_ClearFlag_JEOS(ADCx);
  LL_ADC_EnableIT_JEOS(ADCx);
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
  * @brief  writes into peripheral registers the new duty cycles and
  *        sampling point
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
static uint16_t R3_1_F30X_WriteTIMRegisters(PWMC_Handle_t *pHdl)
{
  uint32_t wCCR4Aux;
  uint16_t hAux;
      
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  TIMx->CCR1 = pHandle->_Super.hCntPhA;
  TIMx->CCR2 = pHandle->_Super.hCntPhB;
  TIMx->CCR3 = pHandle->_Super.hCntPhC;
  wCCR4Aux = (uint16_t)(TIMx->CCR4);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  TIMx->CCR4 = 0xFFFFu;
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  
  TIMx->CCR4 = wCCR4Aux;
  pHandle->pParams_str->ADCx->JSQR = pHandle->wADC1_JSQR;
    
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
  * @brief  Configure the ADC for the current sampling during calibration.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
static uint16_t R3_1_F30X_SetADCSampPointCalibration(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;

  pHandle->pParams_str->TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
  return R3_1_F30X_WriteTIMRegisters(&pHandle->_Super);
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
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3_1_F30X_SetADCSampPointSect1(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;

  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC) > pHandle->pParams_str->hTafter))
  {
    pHandle->pParams_str->TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_5; /* Dummy just for the GetPhaseCurrent */
    
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB;
  }
  else
  { /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/
    
    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
    duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
    one with minimum complementary duty and one with variable complementary duty. In this case, phases 
    with variable complementary duty and with maximum duty are converted and the first will be always 
    the phase with variable complementary duty cycle */
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    {  /* Crossing Point Searching */
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhB);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhA - pHandle->pParams_str->hTbefore; /* hTbefore = 2*Ts + Tc, where Ts = Sampling time of ADC, Tc = Conversion Time of ADC */
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhA + pHandle->pParams_str->hTafter; /* hTafter = DT + max(Trise, Tnoise) */
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        { 
          /* It must be changed the trigger direction from positive to negative 
             to sample after middle of PWM*/
          pHandle->wADC_JSQR_phBC &= JSQR_EDGE_CLEAR_Mask;
          pHandle->wADC_JSQR_phBC |= (uint32_t )LL_ADC_INJ_TRIG_EXT_FALLING;
          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }   
    /* Set JSQR register */
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phBC;
    
    /* Set TIMx_CH4 value */
    pHandle->pParams_str->TIMx->CCR4 = hCntSmp;
  }
  return R3_1_F30X_WriteTIMRegisters(&pHandle->_Super);
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
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3_1_F30X_SetADCSampPointSect2(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC) > pHandle->pParams_str->hTafter))
  {
    pHandle->pParams_str->TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_5; /* Dummy just for the GetPhaseCurrent */
    
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB;
  }
  else
  {
 /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/
    
    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
    duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
    one with minimum complementary duty and one with variable complementary duty. In this case, phases 
    with variable complementary duty and with maximum duty are converted and the first will be always 
    the phase with variable complementary duty cycle */    
    
    /* Searching of sampling point to avoid noise inducted by a commutation of other phase's switch */
    
    /* Check if sampling of detected current phases in the middle of PWM is possible.
       It depends on if the complementary duty cycle of the phase having a minimum complementary duty cycle 
       is greater than the Tafter time because in this case its commutation is sufficiently distant from the 
       middle of PWM and it doesn't induct any noise to Shunt voltages of the others phases */
    
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    { /* Crossing Point Searching */ 
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhA);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhB - pHandle->pParams_str->hTbefore; /* hTbefore = 2*Ts + Tc, where Ts = Sampling time of ADC, Tc = Conversion Time of ADC  */
      }
      else
      {
        hCntSmp = pHandle->_Super. hCntPhB + pHandle->pParams_str->hTafter;  /* hTafter = DT + max(Trise, Tnoise) */
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* It must be changed the trigger direction from positive to negative 
             to sample after middle of PWM*/
          pHandle->wADC_JSQR_phAC &= JSQR_EDGE_CLEAR_Mask;
          pHandle->wADC_JSQR_phAC |= (uint32_t )LL_ADC_INJ_TRIG_EXT_FALLING;

          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }
    
    /* Set JSQR register */
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAC;
    
    /* Set TIMx_CH4 value */
    pHandle->pParams_str->TIMx->CCR4 = hCntSmp;
  }
  return R3_1_F30X_WriteTIMRegisters(&pHandle->_Super);
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
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3_1_F30X_SetADCSampPointSect3(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC) > pHandle->pParams_str->hTafter))
  {
    pHandle->pParams_str->TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_5; /* Dummy just for the GetPhaseCurrent */

    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB;
  }
  else
  {/* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/
    
    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
    duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
    one with minimum complementary duty and one with variable complementary duty. In this case, phases 
    with variable complementary duty and with maximum duty are converted and the first will be always 
    the phase with variable complementary duty cycle */ 
    
    uint32_t wADC_JSQR_2phase = pHandle->wADC_JSQR_phCA;
    
    /* Searching of sampling point to avoid noise inducted by a commutation of other phase's switch */
    
    /* Check if sampling of detected current phases in the middle of PWM is possible.
    It depends on if the complementary duty cycle of the phase having a minimum complementary duty cycle 
    is greater than the Tafter time because in this case its commutation is sufficiently distant from the 
    middle of PWM and it doesn't induct any noise to Shunt voltages of the others phases */
    
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    {/* Crossing Point Searching */
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhC);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhB - pHandle->pParams_str->hTbefore; /* hTbefore = 2*Ts + Tc, where Ts = Sampling time of ADC, Tc = Conversion Time of ADC  */
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhB + pHandle->pParams_str->hTafter; /* hTafter = DT + max(Trise, Tnoise) */
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* It must be changed the trigger direction from positive to negative 
             to sample after middle of PWM*/
          wADC_JSQR_2phase &= JSQR_EDGE_CLEAR_Mask;
          wADC_JSQR_2phase |= (uint32_t )LL_ADC_INJ_TRIG_EXT_FALLING;
          
         hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }
    /* Set JSQR register */
    pHandle->wADC1_JSQR = wADC_JSQR_2phase;

    /* Set TIMx_CH4 value */
    pHandle->pParams_str->TIMx->CCR4 = hCntSmp;
  }
  return R3_1_F30X_WriteTIMRegisters(&pHandle->_Super);
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
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3_1_F30X_SetADCSampPointSect4(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC) > pHandle->pParams_str->hTafter))
  {
    pHandle->pParams_str->TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_5; /* Dummy just for the GetPhaseCurrent */
    
    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB;
  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/
    
    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
    duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
    one with minimum complementary duty and one with variable complementary duty. In this case, phases 
    with variable complementary duty and with maximum duty are converted and the first will be always 
    the phase with variable complementary duty cycle */ 
    
    uint32_t wADC_JSQR_2phase = pHandle->wADC_JSQR_phBA;
    
    /* Searching of sampling point to avoid noise inducted by a commutation of other phase's switch */
    
    /* Check if sampling of detected current phases in the middle of PWM is possible.
       It depends on if the complementary duty cycle of the phase having a minimum complementary duty cycle 
       is greater than the Tafter time because in this case its commutation is sufficiently distant from the 
       middle of PWM and it doesn't induct any noise to Shunt voltages of the others phases */    
    
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    {/* Crossing Point Searching */
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhB);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhC - pHandle->pParams_str->hTbefore; /* hTbefore = 2*Ts + Tc, where Ts = Sampling time of ADC, Tc = Conversion Time of ADC  */
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhC + pHandle->pParams_str->hTafter; /* hTafter = DT + max(Trise, Tnoise) */
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* It must be changed the trigger direction from positive to negative 
             to sample after middle of PWM*/
          wADC_JSQR_2phase &= JSQR_EDGE_CLEAR_Mask;
          wADC_JSQR_2phase |= (uint32_t )LL_ADC_INJ_TRIG_EXT_FALLING;
          
          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }
    
    /* Set JSQR register */
    pHandle->wADC1_JSQR = wADC_JSQR_2phase;
    
    /* Set TIMx_CH4 value */
    pHandle->pParams_str->TIMx->CCR4 = hCntSmp;
  }
  return R3_1_F30X_WriteTIMRegisters(&pHandle->_Super);
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
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
*/
uint16_t R3_1_F30X_SetADCSampPointSect5(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC) > pHandle->pParams_str->hTafter))
  {
    pHandle->pParams_str->TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_5; /* Dummy just for the GetPhaseCurrent */

    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB;
  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/
    
    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
    duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
    one with minimum complementary duty and one with variable complementary duty. In this case, phases 
    with variable complementary duty and with maximum duty are converted and the first will be always 
    the phase with variable complementary duty cycle */ 

     uint32_t wADC_JSQR_2phase = pHandle->wADC_JSQR_phAB;
    /* Searching of sampling point to avoid noise inducted by a commutation of other phase's switch */
    
    /* Check if sampling of detected current phases in the middle of PWM is possible.
       It depends on if the complementary duty cycle of the phase having a minimum complementary duty cycle 
       is greater than the Tafter time because in this case its commutation is sufficiently distant from the 
       middle of PWM and it doesn't induct any noise to Shunt voltages of the others phases */      
    
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    { /* Crossing Point Searching */
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhA);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhC - pHandle->pParams_str->hTbefore; /* hTbefore = 2*Ts + Tc, where Ts = Sampling time of ADC, Tc = Conversion Time of ADC  */
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhC + pHandle->pParams_str->hTafter; /* hTafter = DT + max(Trise, Tnoise) */
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* It must be changed the trigger direction from positive to negative 
             to sample after middle of PWM*/
          wADC_JSQR_2phase &= JSQR_EDGE_CLEAR_Mask;
          wADC_JSQR_2phase |= (uint32_t )LL_ADC_INJ_TRIG_EXT_FALLING;         
          
          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }
    
  /* Set JSQR register */
  pHandle->wADC1_JSQR = wADC_JSQR_2phase;
    
  /* Set TIMx_CH4 value */
  pHandle->pParams_str->TIMx->CCR4 = hCntSmp;
  }
  return R3_1_F30X_WriteTIMRegisters(&pHandle->_Super);
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
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3_1_F30X_SetADCSampPointSect6(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp, hDeltaDuty;
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  /* Check if sampling AB in the middle of PWM is possible */
  if (((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter) &&
      ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhC) > pHandle->pParams_str->hTafter))
  {
    pHandle->pParams_str->TIMx->CCR4 = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
    pHandle->_Super.hSector = SECTOR_5; /* Dummy just for the GetPhaseCurrent */

    pHandle->wADC1_JSQR = pHandle->wADC_JSQR_phAB;
  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/
    
    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
    duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
    one with minimum complementary duty and one with variable complementary duty. In this case, phases 
    with variable complementary duty and with maximum duty are converted and the first will be always 
    the phase with variable complementary duty cycle */ 

    uint32_t wADC_JSQR_2phase = pHandle->wADC_JSQR_phCB;
    /* Searching of sampling point to avoid noise inducted by a commutation of other phase's switch */
    
    /* Check if sampling of detected current phases in the middle of PWM is possible.
       It depends on if the complementary duty cycle of the phase having a minimum complementary duty cycle 
       is greater than the Tafter time because in this case its commutation is sufficiently distant from the 
       middle of PWM and it doesn't induct any noise to Shunt voltages of the others phases */    
    
    if ((uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter)
    {
      hCntSmp = pHandle->Half_PWMPeriod - 1u;
    }
    else
    {/* Crossing Point Searching */
      hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhC);
      
      /* Definition of crossing point */
      if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod-pHandle->_Super.hCntPhA)*2u)
      {
        hCntSmp = pHandle->_Super.hCntPhA - pHandle->pParams_str->hTbefore; /* hTbefore = 2*Ts + Tc, where Ts = Sampling time of ADC, Tc = Conversion Time of ADC  */
      }
      else
      {
        hCntSmp = pHandle->_Super.hCntPhA + pHandle->pParams_str->hTafter; /* hTafter = DT + max(Trise, Tnoise) */
        
        if (hCntSmp >= pHandle->Half_PWMPeriod)
        {
          /* It must be changed the trigger direction from positive to negative 
             to sample after middle of PWM*/
          wADC_JSQR_2phase &= JSQR_EDGE_CLEAR_Mask;
          wADC_JSQR_2phase |= (uint32_t )LL_ADC_INJ_TRIG_EXT_FALLING;
          
          hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
        }
      }
    }

    /* Set JSQR register */
    pHandle->wADC1_JSQR = wADC_JSQR_2phase;
    
    /* Set TIMx_CH4 value */
    pHandle->pParams_str->TIMx->CCR4 = hCntSmp;
  }
  return R3_1_F30X_WriteTIMRegisters(&pHandle->_Super);
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
void *R3_1_F30X_TIMx_UP_IRQHandler(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t*)pHdl;
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
void *R3_1_F30X_BRK2_IRQHandler(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t*)pHdl;

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
void *R3_1_F30X_BRK_IRQHandler(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t*)pHdl;

  pHandle->pParams_str->TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  pHandle->OverVoltageFlag = true;
  pHandle->BrakeActionLock = true;

  return &(pHandle->_Super.bMotor);
}

/**
  * @brief  Execute a regular conversion using ADCx.
  *         The function is not re-entrant (can't executed twice at the same time)
  * @param pHdl: handler of the current instance of the PWM component
  * @retval It returns converted value or 0xFFFF for conversion error
  */
uint16_t R3_1_F30X_ExecRegularConv(PWMC_Handle_t *pHdl, uint8_t bChannel)
{
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;

  pHandle->pParams_str->regconvADCx->SQR1 = (uint32_t)(bChannel) << 6;
  pHandle->pParams_str->regconvADCx->DR;
  pHandle->pParams_str->regconvADCx->CR = ADC_CR_ADSTART;
  
  /* Wait until end of regular conversion */
  while ((pHandle->pParams_str->regconvADCx->ISR & ADC_ISR_EOC) == 0u)
  {
  }
  
  pHandle->hRegConv = (uint16_t)(pHandle->pParams_str->regconvADCx->DR);
  return (pHandle->hRegConv);
}

/**
  * @brief  It sets the specified sampling time for the specified ADC channel
  *         on ADCx. It must be called once for each channel utilized by user
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_F30X_ADC_SetSamplingTime(PWMC_Handle_t *pHdl, ADConv_t ADConv_struct)
{ 
  uint32_t tmpreg2 = 0u;
  uint8_t ADC_Channel = ADConv_struct.Channel;
  uint8_t ADC_SampleTime = ADConv_struct.SamplTime;
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
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
  else /* ADC_Channel include in ADC_CHANNEL_[0..9] */
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
  * @param pHdl: handler of the current instance of the PWM component
  * @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
  *                  detected since last method call, MC_NO_FAULTS otherwise.
  */
uint16_t R3_1_F30X_IsOverCurrentOccurred(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
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
  * @brief  It is used to disable the PWM mode during RL Detection Mode.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_F30X_RLDetectionModeEnable(PWMC_Handle_t *pHdl)
{
  ADC_InjectedInitTypeDef ADC_InjectedInitStruct;
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef* ADCx = pHandle->pParams_str->ADCx;
  
  if (pHandle->_Super.RLDetectionMode == false)
  {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
    
    LL_TIM_OC_SetCompareCH1(TIMx, 0u);
    
    /*  Channel2 configuration */
    if ((pHandle->pParams_str->LowSideOutputs)== LS_PWM_TIMER)
    {
      LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_ACTIVE);
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2);
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
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
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM2);
    LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3N);
    
    
    /* Set Update as TRGO of TIM1 */
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_UPDATE);
		
   /* Configuration of ADC sequence of two Phase B current values for during RL Detection Mode*/     
   ADC_InjectedInitStruct.ADC_ExternalTrigInjecConvEvent = ADC_EXTERNALTRIGINJECTEVENT;
   ADC_InjectedInitStruct.ADC_ExternalTrigInjecEventEdge = ADC_EXTERNALTRIGINJECTEDGE;
   ADC_InjectedInitStruct.ADC_NbrOfInjecChannel =2u;
   
   /*Phase B currents sequence -----------------------------------------------*/
   ADC_InjectedInitStruct.ADC_InjecSequence1 = pHandle->pParams_str->bIbChannel;
   ADC_InjectedInitStruct.ADC_InjecSequence2 = pHandle->pParams_str->bIbChannel;
   ADC_InjectedInitStruct.ADC_InjecSequence3 = 0u;
   ADC_InjectedInitStruct.ADC_InjecSequence4 = 0u;
	 
  /* ADCx Injected discontinuous mode activation.
   * This is important because permits to convert first current value of ADCx Injected Sequence at
   * the first Update-Trigger event and wait until the second Update-Trigger event happens to start 
   * the second ADCx Injected conversion, then only at the end of the second conversion JEOS Interrupt
   * event is generated.  
   */
   LL_ADC_INJ_SetSequencerDiscont(ADCx, LL_ADC_INJ_SEQ_DISCONT_1RANK);
	 
   /*NB: the following istruction doesn't write the JSQR register of ADCx but
   * writes into the variable of the pDVars_str structure Class the JSQR value that
   * will be used in the future functions */
   pHandle->wADC_JSQR_RL_Detection_phB = SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);
  }
  
  pHandle->_Super.pFctGetPhaseCurrents = &R3_1_F30X_RLGetPhaseCurrents;
  pHandle->_Super.pFctTurnOnLowSides = &R3_1_F30X_RLTurnOnLowSides;
  pHandle->_Super.pFctSwitchOnPwm = &R3_1_F30X_RLSwitchOnPWM;
  pHandle->_Super.pFctSwitchOffPwm = &R3_1_F30X_RLSwitchOffPWM;
  
  pHandle->_Super.RLDetectionMode = true;
}

/**
  * @brief  It is used to disable the PWM mode during RL Detection Mode.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3_1_F30X_RLDetectionModeDisable(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  if (pHandle->_Super.RLDetectionMode == true)
  {
    /* Repetition Counter of TIM1 User value reactivation BEGIN*/
    
    /* The folowing while cycles ensure the identification of the positive counting mode of TIM1 
     * for correct reactivation of Repetition Counter value of TIM1.*/
    
    /* Wait the change of Counter Direction of TIM1 from Up-Direction to Down-Direction*/
    while ((TIMx->CR1 & DIR_MASK) == 0u)
    {
    }
    /* Wait the change of Counter Direction of TIM1 from Down-Direction to Up-Direction.*/
    while ((TIMx->CR1 & DIR_MASK) == DIR_MASK)
    {
    }
    
    /* TIM1 Repetition Counter reactivation to the User Value */
    TIMx->RCR = pHandle->pParams_str->bRepetitionCounter;
    /* Repetition Counter of TIM1 User value reactivation END*/
    
    
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
    
    if ((pHandle->pParams_str-> LowSideOutputs)== LS_PWM_TIMER)
    {
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
    }
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
    }
    else
    {
    }
    
    LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);
    
    /*  Channel2 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2);
    
    if ((pHandle->pParams_str-> LowSideOutputs)== LS_PWM_TIMER)
    {
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
    }
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
    }
    else
    {
    }
    
    LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);
    
    /*  Channel3 configuration */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3);
    
    if ((pHandle->pParams_str-> LowSideOutputs)== LS_PWM_TIMER)
    {
      LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3N);
    }
    else if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3N);
    }
    else
    {
    }
    
    LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);

    /* Set channel 4 as TRGO (Center TRIGGER - Overflow of TIM1)*/
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_OC4REF);
		
    /* ADCx Injected discontinuous mode disable */
    LL_ADC_INJ_SetSequencerDiscont(pHandle->pParams_str->ADCx,
                                   LL_ADC_INJ_SEQ_DISCONT_DISABLE);
       
    pHandle->_Super.pFctGetPhaseCurrents = &R3_1_F30X_GetPhaseCurrents;
    pHandle->_Super.pFctTurnOnLowSides = &R3_1_F30X_TurnOnLowSides;
    pHandle->_Super.pFctSwitchOnPwm = &R3_1_F30X_SwitchOnPWM;
    pHandle->_Super.pFctSwitchOffPwm = &R3_1_F30X_SwitchOffPWM;
    
    pHandle->_Super.RLDetectionMode = false;
  }
}

/**
  * @brief  It is used to set the PWM dutycycle during RL Detection Mode.
  * @param pHdl: handler of the current instance of the PWM component
  * @param  hDuty: duty cycle to be applied in uint16_t
  * @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR'
  *         otherwise. These error codes are defined in mc_type.h
  */
uint16_t R3_1_F30X_RLDetectionModeSetDuty(PWMC_Handle_t *pHdl, uint16_t hDuty)
{
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  uint16_t hAux;
  
  uint32_t val = ((uint32_t)(pHandle->Half_PWMPeriod) * (uint32_t)(hDuty)) >> 16;
  pHandle->_Super.hCntPhA = (uint16_t)(val);
  
  /* JSQR ADCx resgister writing. The sequence configuration values are set into
   * the R3_1_F30X_RLDetectionModeEnable function*/
  pHandle->pParams_str->ADCx->JSQR = pHandle->wADC_JSQR_RL_Detection_phB;

  /* TIM1 Channel 1 Duty Cycle configuration. 
   * In RL Detection mode only the Up-side device of Phase A are controlled 
   * while the Phase B up-side device is always open.*/
  pHandle->pParams_str->TIMx->CCR1 = pHandle->_Super.hCntPhA;
  
  /* Limit for update event */
  /* Check the status flag. If an update event has occurred before to set new
  values of regs the FOC rate is too high */
  if (LL_TIM_IsActiveFlag_UPDATE(pHandle->pParams_str->TIMx))
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
  * @param pHdl: handler of the current instance of the PWM component
  * @retval Ia and Ib current in Curr_Components format
  */
static void R3_1_F30X_RLGetPhaseCurrents(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents)
{
  int32_t wAux;
  int16_t hCurrA = 0, hCurrB = 0;
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  
  /* Clear the flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->TIMx);
  
  wAux = (int32_t)(pHandle->wPhaseBOffset);
  wAux -= (int32_t)(pHandle->pParams_str->ADCx->JDR1);
  
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
  /* First value read of Phase B*/
  hCurrA = (int16_t)(wAux);                     
  
  wAux = (int32_t)(pHandle->wPhaseBOffset);
  wAux -= (int32_t)(pHandle->pParams_str->ADCx->JDR2);
  
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
  /* Second value read of Phase B*/  
  hCurrB = (int16_t)(wAux);                   

  
  pStator_Currents->qI_Component1 = hCurrA;
  pStator_Currents->qI_Component2 = hCurrB;
}

/**
  * @brief  It turns on low sides switches. This function is intended to be 
  *         used for charging boot capacitors of driving section. It has to be 
  *         called each motor start-up when using high voltage drivers.
  *         This function is specific for RL detection phase.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
static void R3_1_F30X_RLTurnOnLowSides(PWMC_Handle_t *pHdl)
{  
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  /*Turn on the phase A low side switch */
  TIMx->CCR1 = 0u;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  
  /* Wait until next update */
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0)
  {}
  
  /* Main PWM Output Enable */
  TIMx->BDTR |= TIM_BDTR_MOE;
  
  if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
  {
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
    LL_GPIO_SetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
  }
  return; 
}


/**
  * @brief  It enables PWM generation on the proper Timer peripheral
  *         This function is specific for RL detection phase.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
static void R3_1_F30X_RLSwitchOnPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = false;
  /* The folowing while cycles ensure the identification of the nergative counting mode of TIM1 
   * for correct modification of Repetition Counter value of TIM1.*/
  
  /* Wait the change of Counter Direction of TIM1 from Down-Direction to Up-Direction*/
  
  while ((TIMx->CR1 & DIR_MASK) == DIR_MASK)
  {
  }
  /* Wait the change of Counter Direction of TIM1 from Up-Direction to Down-Direction*/
  while ((TIMx->CR1 & DIR_MASK) ==0u)
  {
  }
  /* Set Repetition counter to zero */
  TIMx->RCR = 0u;
  
  
  TIMx->CCR1 = 1u;
    
  /* JSQR ADCx resgister writing. The sequence configuration values are set into
   * the R3_1_F30X_RLDetectionModeEnable function*/
  pHandle->pParams_str->ADCx->JSQR = pHandle->wADC_JSQR_RL_Detection_phB;
  
  LL_TIM_ClearFlag_UPDATE(TIMx); /* Clear flag to wait next update */
  
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0)
  {}
  
  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  TIMx->BDTR |= TIM_BDTR_MOE;
  
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
  *         This function is specific for RL detection phase.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval none
  */
static void R3_1_F30X_RLSwitchOffPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_F3_Handle_t *pHandle = (PWMC_R3_1_F3_Handle_t *)pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef* ADCx = pHandle->pParams_str->ADCx;
    
  /* Main PWM Output Disable */
  if (pHandle->BrakeActionLock == true)
  {
  }
  else
  {
    TIMx->BDTR &= ~((uint32_t)(LL_TIM_OSSI_ENABLE));
    
    if ((pHandle->pParams_str->LowSideOutputs)== ES_GPIO)
    {
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin);
    }
  }
  TIMx->BDTR &= (uint32_t)~TIM_BDTR_MOE;
  
  LL_ADC_DisableIT_JEOS(pHandle->pParams_str->ADCx);
  
  /* Flushing JSQR queue of context by setting JADSTP = 1 (JQM)=1 */
  ADCx->CR |= ADC_CR_JADSTP;
 
  ADC_InjectedInitTypeDef ADC_InjectedInitStruct;
  ADC_InjectedInitStruct.ADC_NbrOfInjecChannel = 2u;
  ADC_InjectedInitStruct.ADC_InjecSequence1 = 0u;
  ADC_InjectedInitStruct.ADC_ExternalTrigInjecEventEdge = ADC_EXTERNALTRIGINJECTEDGE;
  ADC_InjectedInitStruct.ADC_ExternalTrigInjecConvEvent = ADC_EXTERNALTRIGINJECTEVENT;
  ADCx->JSQR = SingleADC_InjectedConfig(ADCx, &ADC_InjectedInitStruct);
  ADCx->CR |= ADC_CR_JADSTART;
  
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  /* Set CC4 as PWM mode 2 (default) */
  TIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  TIMx->CCMR2 |= CCMR2_CH4_PWM2;
  TIMx->CCR4 = 0xFFFFu;
  TIMx->CCR4 = 0x0u;
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  
  while (LL_ADC_IsActiveFlag_JEOS(ADCx) == 0)
  {}
  
  /* ADCx Injected conversions end interrupt enabling */
  LL_ADC_ClearFlag_JEOS(pHandle->pParams_str->ADCx);
  LL_ADC_EnableIT_JEOS(pHandle->pParams_str->ADCx);
  return;
}

/**
  * @brief  Initializes the ADCx peripheral according to the specified parameters
  *         in the ADC_InitStruct.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_InjectInitStruct: pointer to an ADC_InjecInitTypeDef structure that contains
  *         the configuration information for the specified ADC injected channel.
  * @retval None
  */
static uint32_t SingleADC_InjectedConfig(ADC_TypeDef* ADCx, ADC_InjectedInitTypeDef* ADC_InjectedInitStruct)
{
  uint32_t tmpreg1 = 0u;
  
  /*---------------------------- ADCx JSQR Configuration -----------------*/
  /* Get the ADCx JSQR value */
  tmpreg1 = ADCx->JSQR;
  
  /* Clear L bits */
  tmpreg1 &= JSQR_CLEAR_Mask;

  /* Configure ADCx: Injected channel sequence length, external trigger, 
     external trigger edge and sequences
  */
  tmpreg1 = (uint32_t) (((uint32_t)(ADC_InjectedInitStruct->ADC_NbrOfInjecChannel) - 1u) |
                         ADC_InjectedInitStruct->ADC_ExternalTrigInjecConvEvent |         
                         ADC_InjectedInitStruct->ADC_ExternalTrigInjecEventEdge |
                         (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence1) << 8) |
                         (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence2) << 14) |
                         (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence3) << 20) |
                         (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence4) << 26));
   
  return tmpreg1;  
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

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
