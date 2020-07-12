/**
  ******************************************************************************
  * @file    r1_dd_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains common definitions for Single Shunt, Dual Drives
  *          PWM and Current Feedback components.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __R1_DD_PWM_CURR_FDBK_H
#define __R1_DD_PWM_CURR_FDBK_H

#define GPIO_NoRemap_TIM1 ((uint32_t)(0))

#define SAME_FREQ   0u

/** @addtogroup MCSDK
  * @{
  */
  
/** @addtogroup pwm_curr_fdbk
  * @{
  */


/** @defgroup PWMnCurrFdbk_R1_DD Single Shunt Dual Drive Parameters
 *
 * @brief Common definitions for Single Shunt, Dual Drives PWM and Current Feedback components
 *
  * @{
  */

/** 
  * @brief  R1 DD parameters definition
  */
typedef const struct
{
  uint32_t wADC_Clock_Divider;    /*!< APB2 clock prescaling factor for 
                                       ADC peripheral. It must be RCC_PCLK2_DivX
                                       x = 2, 4, 6, 8 */  
  uint8_t bTim_Clock_Divider;      /*!< APB2 clock prescaling factor for 
                                          TIM peripheral. It must be equal to 1, 
                                          2 or 4*/    
  uint8_t IRQnb;                  /*!< MC IRQ number used for TIM1 Update event 
                                       and for DMA TC event.*/

  /* Dual MC parameters --------------------------------------------------------*/ 
  uint8_t bInstanceNbr;           /*!< Instance number with reference to PWMC  
                                       base class. It is necessary to properly
                                       synchronize TIM8 with TIM1 at peripheral
                                       initializations */ 
  bool IsFirstR1DDInstance;      /*!< Specifies whether this object is the first
                                        R1DD instance or not.*/
  uint8_t  bFreqRatio;            /*!< It is used in case of dual MC to 
                                       synchronize times. It must be equal 
                                       to the ratio between the two PWM 
                                       frequencies (higher/lower). 
                                       Supported values are 1, 2 or 3 */
  uint8_t  bIsHigherFreqTim;      /*!< When bFreqRatio is greather than 1
                                       this param is used to indicate if this 
                                       instance is the one with the highest 
                                       frequency. Allowed value are: HIGHER_FREQ
                                       or LOWER_FREQ */
  TIM_TypeDef* TIMx;              /*!< Timer used for PWM generation. It should be
                                       TIM1 or TIM8*/

  /* Current reading A/D Conversions initialization --------------------------*/ 
  uint8_t hIChannel;              /*!< ADC channel used for conversion of 
                                       current. It must be equal to  
                                       ADC_CHANNEL_x x= 0, ..., 15*/
  GPIO_TypeDef* hIPort;           /*!< GPIO port used by hIChannel. It must 
                                       be equal to GPIOx x= A, B, ...*/
  uint16_t hIPin;                 /*!< GPIO pin used by hIChannel. It must be 
                                       equal to GPIO_Pin_x x= 0, 1, ...*/ 
  uint8_t b_ISamplingTime;        /*!< Sampling time used to convert hIChannel. 
                                       It must be equal to ADC_SampleTime_xCycles5 
                                       x= 1, 7, ...*/ 
  
/* PWM generation parameters --------------------------------------------------*/   
  uint16_t hDeadTime;             /*!< Dead time in number of TIM clock
                                       cycles. If CHxN are enabled, it must
                                       contain the dead time to be generated
                                       by the microcontroller, otherwise it 
                                       expresses the maximum dead time 
                                       generated by driving network*/
  uint8_t  bRepetitionCounter;    /*!< It expresses the number of PWM 
                                       periods to be elapsed before compare 
                                       registers are updated again. In 
                                       particular: 
                                       RepetitionCounter= (2* PWM periods) -1*/   
  uint16_t hTafter;               /*!< It is the sum of dead time plus rise time
                                       express in number of TIM clocks.*/
  uint16_t hTbefore;              /*!< It is the value of sampling time 
                                       expressed in numbers of TIM clocks.*/ 
  uint16_t hTMin;                 /*!< It is the sum of dead time plus rise time
                                       plus sampling time express in numbers of
                                       TIM clocks.*/
  uint16_t hHTMin;                /*!< It is the half of hTMin value.*/
  uint16_t hTSample;              /*!< It is the sampling time express in 
                                       numbers of TIM clocks.*/
  uint16_t hMaxTrTs;              /*!< It is the maximum between twice of rise
                                       time express in number of TIM clocks and
                                       twice of sampling time express in numbers
                                       of TIM clocks.*/
  
/* PWM Driving signals initialization ----------------------------------------*/  
  
  uint32_t wTIM1Remapping;        /*!< Used only for instance #1, it 
                                       remaps TIM1 outputs. It must equal to 
                                       GPIO_PartialRemap_TIM1 or 
                                       GPIO_FullRemap_TIM1 or
                                       GPIO_NoRemap_TIM1 */
  uint16_t hCh1Polarity;          /*!< Channel 1 (high side) output polarity, 
                                       it must be TIM_OCPolarity_High or 
                                       TIM_OCPolarity_Low */
  GPIO_TypeDef* hCh1Port;         /*!< Channel 1 (high side) GPIO output 
                                       port (if used, after re-mapping). 
                                       It must be GPIOx x= A, B, ...*/
  uint32_t hCh1Pin;               /*!< Channel 1 (high side) GPIO output pin 
                                       (if used, after re-mapping). It must be 
                                       GPIO_Pin_x x= 0, 1, ...*/  
  uint16_t hCh1IdleState;         /*!< Channel 1 (high side) state (low or high)
                                       when TIM peripheral is in Idle state.
                                       It must be TIM_OCIdleState_Set or
                                       TIM_OCIdleState_Reset*/  
  
  
  uint16_t hCh2Polarity;          /*!< Channel 2 (high side) output polarity, 
                                       it must be TIM_OCPolarity_High or 
                                       TIM_OCPolarity_Low */ 
  GPIO_TypeDef* hCh2Port;         /*!< Channel 2 (high side) GPIO output 
                                       port (if used, after re-mapping). 
                                       It must be GPIOx x= A, B, ...*/
  uint32_t hCh2Pin;               /*!< Channel 2 (high side) GPIO output pin 
                                       (if used, after re-mapping). It must be 
                                       GPIO_Pin_x x= 0, 1, ...*/
  uint16_t hCh2IdleState;         /*!< Channel 2 (high side) state (low or high)
                                       when TIM peripheral is in Idle state.
                                       It must be TIM_OCIdleState_Set or
                                       TIM_OCIdleState_Reset*/    
 
  uint16_t hCh3Polarity;          /*!< Channel 3 (high side) output polarity, 
                                       it must be TIM_OCPolarity_High or 
                                       TIM_OCPolarity_Low */  
  GPIO_TypeDef* hCh3Port;         /*!< Channel 3 (high side) GPIO output 
                                       port (if used, after re-mapping). 
                                       It must be GPIOx x= A, B, ...*/
  uint32_t hCh3Pin;               /*!< Channel 3 (high side) GPIO output pin 
                                       (if used, after re-mapping). It must be 
                                       GPIO_Pin_x x= 0, 1, ...*/  
  uint16_t hCh3IdleState;         /*!< Channel 3 (high side) state (low or high)
                                       when TIM peripheral is in Idle state.
                                       It must be TIM_OCIdleState_Set or
                                       TIM_OCIdleState_Reset*/  
  
  LowSideOutputsFunction_t LowSideOutputs; /*!< Low side or enabling signals 
                                                generation method are defined 
                                                here.*/ 
  
  uint16_t hCh1NPolarity;         /*!< Channel 1N (low side) output polarity, 
                                       it must be TIM_OCNPolarity_High or 
                                       TIM_OCNPolarity_Low */
  GPIO_TypeDef* hCh1NPort;        /*!< Channel 1N (low side) GPIO output 
                                       port (if used, after re-mapping). 
                                       It must be GPIOx x= A, B, ...*/
  uint32_t hCh1NPin;              /*!< Channel 1N (low side) GPIO output pin 
                                       (if used, after re-mapping). It must be 
                                       GPIO_Pin_x x= 0, 1, ...*/ 
  uint16_t hCh1NIdleState;        /*!< Channel 1N (low side) state (low or high)
                                       when TIM peripheral is in Idle state.
                                       It must be TIM_OCNIdleState_Set or
                                       TIM_OCNIdleState_Reset*/ 

  
  uint16_t hCh2NPolarity;         /*!< Channel 2N (low side) output polarity, 
                                       it must be TIM_OCNPolarity_High or 
                                       TIM_OCNPolarity_Low */
  GPIO_TypeDef* hCh2NPort;        /*!< Channel 2N (low side) GPIO output 
                                       port (if used, after re-mapping). 
                                       It must be GPIOx x= A, B, ...*/
  uint32_t hCh2NPin;              /*!< Channel 2N (low side) GPIO output pin 
                                       (if used, after re-mapping). It must be 
                                       GPIO_Pin_x x= 0, 1, ...*/ 
  uint16_t hCh2NIdleState;        /*!< Channel 2N (low side) state (low or high)
                                       when TIM peripheral is in Idle state.
                                       It must be TIM_OCNIdleState_Set or
                                       TIM_OCNIdleState_Reset*/ 
  
  uint16_t hCh3NPolarity;         /*!< Channel 3N (low side) output polarity, 
                                       it must be TIM_OCNPolarity_High or 
                                       TIM_OCNPolarity_Low */
  GPIO_TypeDef* hCh3NPort;        /*!< Channel 3N (low side)  GPIO output 
                                       port (if used, after re-mapping). 
                                       It must be GPIOx x= A, B, ...*/
  uint32_t hCh3NPin;              /*!< Channel 3N (low side)  GPIO output pin 
                                       (if used, after re-mapping). It must be 
                                       GPIO_Pin_x x= 0, 1, ...*/   
  uint16_t hCh3NIdleState;        /*!< Channel 3N (low side) state (low or high)
                                       when TIM peripheral is in Idle state.
                                       It must be TIM_OCNIdleState_Set or
                                       TIM_OCNIdleState_Reset*/ 
/* PWM Driving signals initialization ----------------------------------------*/  
  FunctionalState EmergencyStop;  /*!< It enable/disable the management of 
                                       an emergency input instantaneously 
                                       stopping PWM generation. It must be 
                                       either equal to ENABLE or DISABLE */  
  uint16_t hBKINPolarity;         /*!< Emergency Stop (BKIN) input polarity, 
                                       it must be TIM_BreakPolarity_Low or 
                                       TIM_BreakPolarity_High */
  GPIO_TypeDef* hBKINPort;        /*!< Emergency Stop (BKIN) GPIO input 
                                       port (if used, after re-mapping). 
                                       It must be GPIOx x= A, B, ...*/
  uint16_t hBKINPin;              /*!< Emergency Stop (BKIN) GPIO input pin 
                                       (if used, after re-mapping). It must be 
                                       GPIO_Pin_x x= 0, 1, ...*/
} R1_DDParams_t, *pR1_DDParams_t;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__R1_DD_PWM_CURR_FDBK_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
