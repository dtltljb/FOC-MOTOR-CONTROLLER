/**
  ******************************************************************************
  * @file    ics_dd_pwmncurrfdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          ICS DD PWM current feedback component of the Motor Control SDK.
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
#ifndef __ICS_DD_PWMNCURRFDBK_H
#define __ICS_DD_PWMNCURRFDBK_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @defgroup PWMnCurrFdbk_ICS_DD Insulated Current Sensing Parameters
 *
 *  @brief Common definitions for Insulated Current Sensors based PWM & Current Feedback components
  * @{
  */

#define GPIO_NoRemap_TIM1 ((uint32_t)(0))
#define SHIFTED_TIMs      ((uint8_t) 1)
#define NO_SHIFTED_TIMs   ((uint8_t) 0)


/** 
  * @brief  ICS_DD parameters definition
  */
typedef struct
{
/* Dual MC parameters --------------------------------------------------------*/ 
  uint8_t bInstanceNbr;              /**< Instance number with reference to PWMC  
                                          base class. It is necessary to properly
                                          synchronize TIM8 with TIM1 at 
                                          peripheral initializations */ 
  uint16_t Tw;                       /**< It is used for switching the context 
                                           in dual MC. It contains biggest delay
                                           (expressed in counter ticks) between 
                                           the counter crest and ADC latest 
                                           trigger  */
  uint8_t  bFreqRatio;               /**< It is used in case of dual MC to 
                                           synchronize TIM1 and TIM8. It has 
                                           effect only on the second instanced 
                                           object and must be equal to the 
                                           ratio between the two PWM frequencies
                                           (higher/lower). Supported values are 
                                           1, 2 or 3 */  
  uint8_t  bIsHigherFreqTim;         /**< When bFreqRatio is greather than 1
                                          this param is used to indicate if this 
                                          instance is the one with the highest 
                                          frequency. Allowed value are: HIGHER_FREQ
                                          or LOWER_FREQ */
/* Current reading A/D Conversions initialization -----------------------------*/ 
  uint8_t bIaChannel;                  /**< ADC channel used for conversion of 
                                           current Ia. It must be equal to  
                                           ADC_CHANNEL_x x= 0, ..., 15*/
  uint8_t bIbChannel;                  /**< ADC channel used for conversion of 
                                           current Ib. It must be equal to 
                                           ADC_CHANNEL_x x= 0, ..., 15*/
  
/* PWM generation parameters --------------------------------------------------*/   
  uint8_t  bRepetitionCounter;         /**< It expresses the number of PWM 
                                           periods to be elapsed before compare 
                                           registers are updated again. In 
                                           particular: 
                                           RepetitionCounter= (2* #PWM periods)-1
                                         */   
  TIM_TypeDef*  TIMx;                   /**< It contains the pointer to the timer 
                                           used for PWM generation. It must 
                                           equal to TIM1 if bInstanceNbr is 
                                           equal to 1, to TIM8 otherwise */
/* PWM Driving signals initialization ----------------------------------------*/  
  LowSideOutputsFunction_t LowSideOutputs; /**< Low side or enabling signals 
                                                generation method are defined 
                                                here.*/ 
  GPIO_TypeDef* pwm_en_u_port;            /**< enable signal phase U port.*/
  uint32_t      pwm_en_u_pin;             /**< enable signal phase U pin.*/
  GPIO_TypeDef* pwm_en_v_port;            /**< enable signal phase V port.*/
  uint32_t      pwm_en_v_pin;             /**< enable signal phase V pin.*/
  GPIO_TypeDef* pwm_en_w_port;            /**< enable signal phase W port.*/
  uint32_t      pwm_en_w_pin;             /**< enable signal phase W pin.*/

/* Emergency input signal initialization -------------------------------------*/  
  FunctionalState EmergencyStop;        /**< It enable/disable the management of 
                                            an emergency input instantaneously 
                                            stopping PWM generation. It must be 
                                            either equal to ENABLE or DISABLE */  
} ICS_DDParams_t;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__ICS_DD_PWMNCURRFDBK_H*/
/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
