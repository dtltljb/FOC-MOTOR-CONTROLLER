/**
  ******************************************************************************
  * @file    r1_f30x_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          r1_f30x_pwm_curr_fdbk component of the Motor Control SDK.
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
#ifndef __R1_F30X_PWMCURRFDBK_H
#define __R1_F30X_PWMCURRFDBK_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup r1_f30x_pwm_curr_fdbk
  * @{
  */

/* Exported constants --------------------------------------------------------*/

#define GPIO_NoRemap_TIM1 ((uint32_t)(0))
#define SHIFTED_TIMs      ((uint8_t) 1)
#define NO_SHIFTED_TIMs   ((uint8_t) 0)

#define NONE		((uint8_t)(0x00))
#define EXT_MODE 	((uint8_t)(0x01))
#define INT_MODE 	((uint8_t)(0x02))
#define STBD3 0x0002u /*!< Flag to indicate which phase has been distorted
                           in boudary 3 zone (A or B)*/
#define DSTEN 0x0004u /*!< Flag to indicate if the distortion must be performed
                           or not (in case of charge of bootstrap capacitor phase
                           is not required)*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief  Paamters structure of the r1_f30x_pwm_curr_fdbk Component.
 *
 */
typedef const struct
{
  /* Dual MC parameters --------------------------------------------------------*/
  uint8_t  bFreqRatio;             /*!< It is used in case of dual MC to
                                          synchronize TIM1 and TIM8. It has
                                          effect only on the second instanced
                                          object and must be equal to the
                                          ratio between the two PWM frequencies
                                          (higher/lower). Supported values are
                                          1, 2 or 3 */
  uint8_t  bIsHigherFreqTim;       /*!< When bFreqRatio is greather than 1
                                          this param is used to indicate if this
                                          instance is the one with the highest
                                          frequency. Allowed value are: HIGHER_FREQ
                                          or LOWER_FREQ */
  /* Current reading A/D Conversions initialization -----------------------------*/
  ADC_TypeDef* ADCx;                  /*!< ADC peripheral to be used*/
  uint8_t bIChannel;                  /*!< ADC channel used for conversion of
                                             current. It must be equal to
                                             ADC_Channel_x x= 0, ..., 15*/
  /* PWM generation parameters --------------------------------------------------*/
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
  uint16_t hHTMin;                /*!< It is the half of hTMin value*/
  uint16_t hCHTMin;                /*!< It is the half of hTMin value, considering FOC rate*/
  uint16_t hTSample;              /*!< It is the sampling time express in
                                         numbers of TIM clocks.*/
  uint16_t hMaxTrTs;              /*!< It is the maximum between twice of rise
                                       time express in number of TIM clocks and
                                       twice of sampling time express in numbers
                                       of TIM clocks.*/
  TIM_TypeDef* TIMx;                   /*!< It contains the pointer to the timer
                                              used for PWM generation. It must
                                              equal to TIM1 if bInstanceNbr is
                                              equal to 1, to TIM8 otherwise */
  /* PWM Driving signals initialization ----------------------------------------*/


  LowSideOutputsFunction_t LowSideOutputs; /*!< Low side or enabling signals
                                                  generation method are defined
                                                  here.*/
  GPIO_TypeDef* pwm_en_u_port;               /*!< Channel 1N (low side) GPIO output
                                           port (if used, after re-mapping). 
                                           It must be GPIOx x= A, B, ...*/
  uint16_t pwm_en_u_pin;                    /*!< Channel 1N (low side) GPIO output pin
                                           (if used, after re-mapping). It must be 
                                           GPIO_Pin_x x= 0, 1, ...*/ 

  GPIO_TypeDef* pwm_en_v_port;               /*!< Channel 2N (low side) GPIO output
                                           port (if used, after re-mapping). 
                                           It must be GPIOx x= A, B, ...*/
  uint16_t pwm_en_v_pin;                     /*!< Channel 2N (low side) GPIO output pin
                                           (if used, after re-mapping). It must be 
                                           GPIO_Pin_x x= 0, 1, ...*/ 
  GPIO_TypeDef* pwm_en_w_port;               /*!< Channel 3N (low side)  GPIO output
                                           port (if used, after re-mapping). 
                                           It must be GPIOx x= A, B, ...*/
  uint16_t pwm_en_w_pin;                    /*!< Channel 3N (low side)  GPIO output pin
                                           (if used, after re-mapping). It must be 
                                           GPIO_Pin_x x= 0, 1, ...*/   
  /* Emergency input (BKIN2) signal initialization -----------------------------*/
  uint8_t bBKIN2Mode;                 /*!< It defines the modality of emergency
                                             input 2. It must be any of the
                                             the following:
                                             NONE - feature disabled.
                                             INT_MODE - Internal comparator used
                                             as source of emergency event.
                                             EXT_MODE - External comparator used
                                             as source of emergency event.*/

  /* Internal OPAMP common settings --------------------------------------------*/
  OPAMP_TypeDef * wOPAMP_Selection;          /*!< First OPAMP selection. It must be
                                                  either equal to
                                                  OPAMP_Selection_OPAMP1 or
                                                  OPAMP_Selection_OPAMP3.*/
  /* Internal COMP settings ----------------------------------------------------*/
  COMP_TypeDef* wCompOCPSelection;        /*!< Internal comparator used for protection.
                                                It must be COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
  uint8_t       bCompOCPInvInput_MODE;    /*!< COMPx inverting input mode. It must be either
                                                equal to EXT_MODE or INT_MODE. */
  COMP_TypeDef* wCompOVPSelection;         /*!< Internal comparator used for protection.
                                                It must be COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
  uint8_t       bCompOVPInvInput_MODE;     /*!< COMPx inverting input mode. It must be either
                                                equal to EXT_MODE or INT_MODE. */
  /* DAC settings --------------------------------------------------------------*/
  uint16_t hDAC_OCP_Threshold;             /*!< Value of analog reference expressed
                                            as 16bit unsigned integer.
                                            Ex. 0 = 0V 65536 = VDD_DAC.*/
  uint16_t hDAC_OVP_Threshold;              /*!< Value of analog reference expressed
                                            as 16bit unsigned integer.
                                            Ex. 0 = 0V 65536 = VDD_DAC.*/
  /* Regular conversion --------------------------------------------------------*/
  ADC_TypeDef * regconvADCx;         /*!< ADC peripheral used for regular
                                            conversion.*/
} R1_F30XParams_t, *pR1_F30XParams_t;

/**
  * @brief  Handle structure of the r1_f30x_pwm_curr_fdbk Component
  */
typedef struct
{
  PWMC_Handle_t _Super;       /*!< Offset of current sensing network  */
  uint16_t hDmaBuff[2];       /*!< Buffer used for PWM distortion points*/
  uint16_t hCntSmp1;          /*!< First sampling point express in timer counts*/
  uint16_t hCntSmp2;          /*!< Second sampling point express in timer counts*/
  uint8_t sampCur1;           /*!< Current sampled in the first sampling point*/
  uint8_t sampCur2;           /*!< Current sampled in the second sampling point*/
  int16_t hCurrAOld;          /*!< Previous measured value of phase A current*/
  int16_t hCurrBOld;          /*!< Previous measured value of phase B current*/
  uint8_t bInverted_pwm_new;  /*!< This value indicates the type of the current
                                   PWM period (Regular, Distort PHA, PHB or PHC)*/
  uint16_t hFlags;            /*!< Flags
                                   STBD3: Flag to indicate which phase has been distorted
                                          in boudary 3 zone (A or B)
                                   DSTEN: Flag to indicate if the distortion must be
                                          performed or not (charge of bootstrap
                                          capacitor phase) */
  uint16_t hRegConv;          /*!< Temporary variables used to store regular conversions*/
  uint32_t wPhaseOffset;      /*!< Offset of Phase current sensing network  */
  volatile uint8_t  bIndex;   /*!< Number of conversions performed during the
                                   calibration phase*/
  uint16_t Half_PWMPeriod;  /* Half PWM Period in timer clock counts */
  uint32_t wADC_JSQR;   /*!< Stores the value for JSQR register to select
                                 phase A motor current.*/
  uint32_t wPreloadDisableActing; /*!< Preload disable to be applied.*/
  uint32_t wPreloadDisableCC1; /*!< CCMR1 that disables the preload register
                                     of the channel to be distorted.*/
  uint32_t wPreloadDisableCC2; /*!< CCMR1 that disables the preload register
                                     of the channel to be distorted.*/
  uint32_t wPreloadDisableCC3; /*!< CCMR2 that disables the preload register
                                     of the channel to be distorted.*/
  DMA_Channel_TypeDef* PreloadDMAy_Chx;  /*!< DMA resource used for disabling the preload register*/
  DMA_Channel_TypeDef* DistortionDMAy_Chx; /*!< DMA resource used for doing the distortion*/
  bool OverCurrentFlag;     /*!< This flag is set when an overcurrent occurs.*/
  bool OverVoltageFlag;     /*!< This flag is set when an overvoltage occurs.*/
  bool BrakeActionLock;     /*!< This flag is set to avoid that brake action is interrupted.*/

  pR1_F30XParams_t pParams_str;

}PWMC_R1_F3_Handle_t;

/**
  * It performs the start of all the timers required by the control.
  * It utilizes TIM2 as temporary timer to achieve synchronization between
  * PWM signals.
  * When this function is called, TIM1 and/or TIM8 must be in frozen state
  * with CNT, ARR, REP RATE and trigger correctly set (these setting are
  * usually performed in the Init method accordingly with the configuration)
  */
void R1F3XX_StartTimers(void);

/**
  * It performs the initialization of the MCU peripherals required for
  * the PWM generation and current sensing. this initialization is dedicated
  * to one shunt topology and F3 family
  */
void R1F30X_Init(PWMC_R1_F3_Handle_t *pHandle);

/**
  * It disables PWM generation on the proper Timer peripheral acting on
  * MOE bit
  */
void R1F30X_SwitchOffPWM(PWMC_Handle_t *pHdl);

/**
  * It enables PWM generation on the proper Timer peripheral acting on MOE
  * bit
  */
void R1F30X_SwitchOnPWM(PWMC_Handle_t *pHdl);

/**
  * It turns on low sides switches. This function is intended to be
  * used for charging boot capacitors of driving section. It has to be
  * called each motor start-up when using high voltage drivers
  */
void R1F30X_TurnOnLowSides(PWMC_Handle_t *pHdl);

/**
  * It computes and return latest converted motor phase currents motor
  */
void R1F30X_GetPhaseCurrents(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents);

/**
  * It contains the TIMx Update event interrupt
  */
void *R1F30X_TIMx_UP_IRQHandler(PWMC_Handle_t *pHdl);

/**
  * It contains the TIMx Break2 event interrupt
  */
void *R1F30X_BRK2_IRQHandler(PWMC_Handle_t *pHdl);

/**
  * It contains the TIMx Break1 event interrupt
  */
void *R1F30X_BRK_IRQHandler(PWMC_Handle_t *pHdl);

/**
  * It stores into pHandle the offset voltage read onchannels when no
  * current is flowing into the motor
  */
void R1F30X_CurrentReadingCalibration(PWMC_Handle_t *pHdl);

/**
  * Implementation of the single shunt algorithm to setup the
  * TIM1 register and DMA buffers values for the next PWM period.
  */
uint16_t R1F30X_CalcDutyCycles(PWMC_Handle_t *pHdl);

/**
  * Execute a regular conversion using ADCx.
  * The function is not re-entrant (can't executed twice at the same time)
  */
uint16_t R1F30X_ExecRegularConv(PWMC_Handle_t *pHdl, uint8_t bChannel);

/**
  * It sets the specified sampling time for the specified ADC channel
  * on ADC1. It must be called once for each channel utilized by user
  */
void R1F30X_ADC_SetSamplingTime(PWMC_Handle_t *pHdl, ADConv_t ADConv_struct);

/**
  * It sets the specified sampling time for the specified ADC channel
  * on ADC1. It must be called once for each channel utilized by user
  */
void R1F30X_HFCurrentsCalibration(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents);

/**
  * It is used to check if an overcurrent occurred since last call.
  */
uint16_t R1F30X_IsOverCurrentOccurred(PWMC_Handle_t *pHdl);

/**
  * It is used to set the PWM mode for R/L detection.
  */
void R1F30X_RLDetectionModeEnable(PWMC_Handle_t *pHdl);

/**
  * @brief  It is used to disable the PWM mode for R/L detection.
  */
void R1F30X_RLDetectionModeDisable(PWMC_Handle_t *pHdl);

/**
  * @brief  It is used to set the PWM dutycycle for R/L detection.
  */
uint16_t R1F30X_RLDetectionModeSetDuty(PWMC_Handle_t *pHdl, uint16_t hDuty);

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

#endif /*__R1_F30X_PWMCURRFDBK_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
