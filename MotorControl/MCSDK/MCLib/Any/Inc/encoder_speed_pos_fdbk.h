/**
  ******************************************************************************
  * @file    encoder_speed_pos_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Encoder Speed & Position Feedback component of the Motor Control SDK.
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
#ifndef __ENCODER_SPEEDNPOSFDBK_H
#define __ENCODER_SPEEDNPOSFDBK_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
   * @{
   */

 /** @addtogroup Encoder
  * @{
  */
   
/* Exported constants --------------------------------------------------------*/

#define GPIO_NoRemap_TIMx ((uint32_t)(0))
#define ENC_DMA_PRIORITY DMA_Priority_High
#define ENC_SPEED_ARRAY_SIZE 	((uint8_t)16)    /* 2^4 */

/** 
  * @brief  ENCODER class parameters definition
  */
typedef struct
{
  SpeednPosFdbk_Handle_t _Super;
  /* SW Settings */
  uint8_t IRQnb; /*!< MC IRQ number used for TIMx capture update event.*/
  
  uint16_t PulseNumber; /*!< Number of pulses per revolution, provided by each
                              of the two encoder signals, multiplied by 4 */
  
  FunctionalState RevertSignal; /*!< To be enabled if measured speed is opposite
                                     to real one (ENABLE/DISABLE)*/
  
  uint16_t SpeedSamplingFreq01Hz; /*!< Frequency (01Hz) at which motor speed is to be
                                   computed. It must be equal to the frequency
                                   at which function SPD_CalcAvrgMecSpeed01Hz
                                   is called.*/
  
  uint8_t SpeedBufferSize; /*!< Size of the buffer used to calculate the average 
                             speed. It must be <= 16.*/
  
  uint16_t InpCaptFilter; /*!< Time filter applied to validate ENCODER sensor
                                capture. This value defines the frequency 
                                used to sample ENCODER sensors input and the 
                                length of the digital filter applied. 
                                The digital filter is made of an event 
                                counter in which N events are needed to 
                                validate a transition on the input.
                                0000: No filter, sampling is done at fCK_INT.
                                0001: fSAMPLING=fCK_INT , N=2. 
                                0010: fSAMPLING=fCK_INT , N=4.
                                0011: fSAMPLING=fCK_INT , N=8. 
                                0100: fSAMPLING=fCK_INT/2, N=6.
                                0101: fSAMPLING=fCK_INT/2, N=8.
                                0110: fSAMPLING=fCK_INT/4, N=6.
                                0111: fSAMPLING=fCK_INT/4, N=8.
                                1000: fSAMPLING=fCK_INT/8, N=6.
                                1001: fSAMPLING=fCK_INT/8, N=8.
                                1010: fSAMPLING=fCK_INT/16, N=5.
                                1011: fSAMPLING=fCK_INT/16, N=6.
                                1100: fSAMPLING=fCK_INT/16, N=8.
                                1101: fSAMPLING=fCK_INT/32, N=5.
                                1110: fSAMPLING=fCK_INT/32, N=6.
                                1111: fSAMPLING=fCK_INT/32, N=8 */
  /* HW Settings */
  TIM_TypeDef* TIMx;    /*!< Timer used for ENCODER sensor management.*/
  
  volatile uint16_t TimerOverflowNb;    /*!< Number of overflows occurred since
                                        last speed measurement event*/
  
  bool SensorIsReliable;            /*!< Flag to indicate sensor/decoding is not
                                         properly working.*/  
  
  
  uint16_t PreviousCapture;            /*!< Timer counter value captured during
                                        previous speed measurement event*/
  
  int32_t DeltaCapturesBuffer[ENC_SPEED_ARRAY_SIZE]; /*!< Buffer used to store
                                        captured variations of timer counter*/
  
  volatile uint8_t DeltaCapturesIndex; /*! <Buffer index*/
  
  uint32_t U32MAXdivPulseNumber;       /*! <It stores U32MAX/hPulseNumber*/
  
  uint16_t SpeedSamplingFreqHz;        /*! <Frequency (Hz) at which motor speed
                                        is to be computed. */
  
  bool TimerOverflowError;              /*!< true if the number of overflow
                                        occurred is greater than 'define'
                                        ENC_MAX_OVERFLOW_NB*/
  
}ENCODER_Handle_t;


void * ENC_IRQHandler(void *pHandleVoid);
void ENC_Init(ENCODER_Handle_t *pHandle);
void ENC_Clear(ENCODER_Handle_t *pHandle);
int16_t ENC_CalcAngle(ENCODER_Handle_t *pHandle);
bool ENC_CalcAvrgMecSpeed01Hz(ENCODER_Handle_t *pHandle, int16_t *pMecSpeed01Hz);
void ENC_SetMecAngle(ENCODER_Handle_t *pHandle, int16_t hMecAngle);

/**
  * @}
  */
  
/**
  * @}
  */

/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__ENCODER_SPEEDNPOSFDBK_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
