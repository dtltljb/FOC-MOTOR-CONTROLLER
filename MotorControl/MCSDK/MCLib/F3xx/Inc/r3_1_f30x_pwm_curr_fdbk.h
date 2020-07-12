/**
  ******************************************************************************
  * @file    r3_1_f30X_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          r3_1_f30X_pwm_curr_fdbk component of the Motor Control SDK.
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
#ifndef __R3_1_F30X_PWMCURRFDBK_H
#define __R3_1_F30X_PWMCURRFDBK_H


#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup r3_1_f30X_pwm_curr_fdbk
  * @{
  */

#define GPIO_NoRemap_TIM1 ((uint32_t)(0))
#define SHIFTED_TIMs      ((uint8_t) 1)
#define NO_SHIFTED_TIMs   ((uint8_t) 0)

#define NONE    ((uint8_t)(0x00))
#define EXT_MODE  ((uint8_t)(0x01))
#define INT_MODE  ((uint8_t)(0x02))

/* Exported types ------------------------------------------------------- */

/**
  * @brief  r3_1_f30X_pwm_curr_fdbk component parameters definition
  */
typedef const struct
{
/* Current reading A/D Conversions initialization -----------------------------*/ 
  ADC_TypeDef* ADCx;      /*!< First ADC used for motor phases current
                                 measurement.*/
  uint8_t bIaChannel;              /*!< ADC channel used for conversion of 
                                        current Ia. It must be equal to  
                                        ADC_CHANNEL_x x= 0, ..., 15*/
  uint8_t bIbChannel;              /*!< ADC channel used for conversion of 
                                        current Ib. It must be equal to 
                                        ADC_CHANNEL_x x= 0, ..., 15*/
  uint8_t bIcChannel;              /*!< ADC channel used for conversion of 
                                        current Ia. It must be equal to  
                                        ADC_CHANNEL_x x= 0, ..., 15*/
/* PWM generation parameters --------------------------------------------------*/   
  uint8_t  bRepetitionCounter;         /*!< It expresses the number of PWM 
                                            periods to be elapsed before compare 
                                            registers are updated again. In 
                                            particular: 
                                            RepetitionCounter= (2* #PWM periods)-1*/
  uint16_t hTafter;                    /*!< It is the sum of dead time plus max 
                                            value between rise time and noise time
                                            express in number of TIM clocks.*/
  uint16_t hTbefore;                   /*!< It is the sampling time express in
                                            number of TIM clocks.*/
  TIM_TypeDef*  TIMx;                   /*!< It contains the pointer to the timer 
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
/* Internal COMP settings ----------------------------------------------------*/
  COMP_TypeDef* wCompOCPASelection;        /*!< Internal comparator used for protection.
                                                It must be COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
  uint8_t       bCompOCPAInvInput_MODE;    /*!< COMPx inverting input mode. It must be either
                                                equal to EXT_MODE or INT_MODE. */
  COMP_TypeDef* wCompOCPBSelection;        /*!< Internal comparator used for protection.
                                                It must be COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
  uint8_t       bCompOCPBInvInput_MODE;    /*!< COMPx inverting input mode. It must be either
                                                equal to EXT_MODE or INT_MODE. */
  COMP_TypeDef* wCompOCPCSelection;        /*!< Internal comparator used for protection.
                                                It must be COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
  uint8_t       bCompOCPCInvInput_MODE;    /*!< COMPx inverting input mode. It must be either
                                                equal to EXT_MODE or INT_MODE. */
  COMP_TypeDef* wCompOVPSelection;         /*!< Internal comparator used for protection.
                                                It must be COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
  uint8_t       bCompOVPInvInput_MODE;     /*!< COMPx inverting input mode. It must be either
                                                equal to EXT_MODE or INT_MODE. */
/* DAC settings --------------------------------------------------------------*/
  uint16_t hDAC_OCP_Threshold;        /*!< Value of analog reference expressed 
                                           as 16bit unsigned integer.
                                           Ex. 0 = 0V 65536 = VDD_DAC.*/
  uint16_t hDAC_OVP_Threshold;        /*!< Value of analog reference expressed 
                                           as 16bit unsigned integer.
                                           Ex. 0 = 0V 65536 = VDD_DAC.*/
/* Regular conversion --------------------------------------------------------*/
  ADC_TypeDef * regconvADCx;          /*!< ADC peripheral used for regular 
                                           conversion.*/
} R3_1_F30XParams_t, *pR3_1_F30XParams_t;


/**
  * @brief  This structure is used to handle an instance of the
  *         r3_1_f30X_pwm_curr_fdbk component.
  */
typedef struct
{
  PWMC_Handle_t _Super;     /*!< base component handler  */
  uint32_t wPhaseAOffset;   /*!< Offset of Phase A current sensing network  */
  uint32_t wPhaseBOffset;   /*!< Offset of Phase B current sensing network  */
  uint32_t wPhaseCOffset;   /*!< Offset of Phase C current sensing network  */
  uint16_t Half_PWMPeriod;  /*!< Half PWM Period in timer clock counts */
  uint16_t hRegConv;        /*!< Variable used to store regular conversions
                                 result*/
  uint32_t wADC1_JSQR;      /*!< Stores the value for JSQR register to select
                                 first acquisiton.*/
  uint32_t wADC_JSQR_phA;    /*!< Stores the value for JSQR register to select
                                 phase A motor current.*/
  uint32_t wADC_JSQR_phB;    /*!< Stores the value for JSQR register to select
                                 phase B motor current.*/

  uint32_t wADC_JSQR_RL_Detection_phB;  /*!< Stores the value for JSQR register to select
                                           phase B motor current during the RLDetection Mode.*/

  uint32_t wADC_JSQR_phC;    /*!< Stores the value for JSQR register to select
                                 phase C motor current.*/

  uint32_t wADC_JSQR_phAB;   /*!< Stores the value for JSQR register to select
                                 phase AB motor currents.*/
  uint32_t wADC_JSQR_phBA;   /*!< Stores the value for JSQR register to select
                                 phase BA motor currents.*/
  uint32_t wADC_JSQR_phAC;   /*!< Stores the value for JSQR register to select
                                 phase AC motor currents.*/
  uint32_t wADC_JSQR_phCA;   /*!< Stores the value for JSQR register to select
                                 phase CA motor currents.*/
  uint32_t wADC_JSQR_phBC;   /*!< Stores the value for JSQR register to select
                                 phase BC motor currents.*/
  uint32_t wADC_JSQR_phCB;   /*!< Stores the value for JSQR register to select
                                 phase CB motor currents.*/
  volatile uint8_t  bIndex;
  uint16_t ADC_ExternalTriggerInjected;
                            /*!< Trigger selection for ADC peripheral.*/
  bool OverCurrentFlag;     /*!< This flag is set when an overcurrent occurs.*/
  bool OverVoltageFlag;     /*!< This flag is set when an overvoltage occurs.*/
  bool BrakeActionLock;     /*!< This flag is set to avoid that brake action is
                                 interrupted.*/
  pR3_1_F30XParams_t pParams_str;

}PWMC_R3_1_F3_Handle_t;

/* Exported functions ------------------------------------------------------- */

/*  It initializes peripherals for current reading and PWM generation
 *  in three shunts configuration using STM32F302x8 *****/
void R3_1_F30X_Init(PWMC_R3_1_F3_Handle_t *pHandle);

/* It disables PWM generation on the proper Timer peripheral acting on
 * MOE bit
 */
void R3_1_F30X_SwitchOffPWM(PWMC_Handle_t *pHdl);

/**
  * It enables PWM generation on the proper Timer peripheral acting on MOE
  * bit
  */
void R3_1_F30X_SwitchOnPWM(PWMC_Handle_t *pHdl);

/**
  * It turns on low sides switches. This function is intended to be
  * used for charging boot capacitors of driving section. It has to be
  * called each motor start-up when using high voltage drivers
  */
void R3_1_F30X_TurnOnLowSides(PWMC_Handle_t *pHdl);

/**
  * It computes and return latest converted motor phase currents motor
  */
void R3_1_F30X_GetPhaseCurrents(PWMC_Handle_t *pHdl, Curr_Components* pStator_Currents);

/**
  * It measures and stores into handler component variables the offset voltage on Ia and
  * Ib current feedback analog channels when no current is flowing into the
  * motor
  */
void R3_1_F30X_CurrentReadingCalibration(PWMC_Handle_t *pHdl);

/**
  * Configure the ADC for the current sampling during calibration.
  * It means set the sampling point via TIMx_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3_1_F30X_SetADCSampPointCalibration(PWMC_Handle_t *pHdl);

/**
  * Configure the ADC for the current sampling related to sector 1.
  * It means set the sampling point via TIMx_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3_1_F30X_SetADCSampPointSect1(PWMC_Handle_t *pHdl);

/**
  * Configure the ADC for the current sampling related to sector 2.
  * It means set the sampling point via TIMx_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3_1_F30X_SetADCSampPointSect2(PWMC_Handle_t *pHdl);

/**
  * Configure the ADC for the current sampling related to sector 3.
  * It means set the sampling point via TIMx_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3_1_F30X_SetADCSampPointSect3(PWMC_Handle_t *pHdl);

/**
  * Configure the ADC for the current sampling related to sector 4.
  * It means set the sampling point via TIMx_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3_1_F30X_SetADCSampPointSect4(PWMC_Handle_t *pHdl);

/**
  * Configure the ADC for the current sampling related to sector 5.
  * It means set the sampling point via TIMx_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3_1_F30X_SetADCSampPointSect5(PWMC_Handle_t *pHdl);

/**
  * Configure the ADC for the current sampling related to sector 6.
  * It means set the sampling point via TIMx_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3_1_F30X_SetADCSampPointSect6(PWMC_Handle_t *pHdl);

/**
  * It contains the TIMx Update event interrupt
  */
void *R3_1_F30X_TIMx_UP_IRQHandler(PWMC_Handle_t *pHdl);

/**
  * It contains the TIMx break2 event interrupt
  */
void *R3_1_F30X_BRK_IRQHandler(PWMC_Handle_t *pHdl);

/**
  * It contains the TIMx break1 event interrupt
  */
void *R3_1_F30X_BRK2_IRQHandler(PWMC_Handle_t *pHdl);
/**
  * Execute a regular conversion using ADCx.
  * The function is not re-entrant (can't executed twice at the same time)
  */

uint16_t R3_1_F30X_ExecRegularConv(PWMC_Handle_t *pHdl, uint8_t bChannel);

/**
  * It sets the specified sampling time for the specified ADC channel
  * on ADCx. It must be called once for each channel utilized by user
  */
void R3_1_F30X_ADC_SetSamplingTime(PWMC_Handle_t *pHdl, ADConv_t ADConv_struct);

/**
  * It is used to check if an overcurrent occurred since last call.
  */
uint16_t R3_1_F30X_IsOverCurrentOccurred(PWMC_Handle_t *pHdl);

/**
  * It is used to enable the PWM mode during RL Detection Mode.
  */
void R3_1_F30X_RLDetectionModeEnable(PWMC_Handle_t *pHdl);

/**
  * It is used to disable the PWM mode during RL Detection Mode.
  */
void R3_1_F30X_RLDetectionModeDisable(PWMC_Handle_t *pHdl);

/**
  * It is used to set the PWM dutycycle during RL Detection Mode.
  */
uint16_t R3_1_F30X_RLDetectionModeSetDuty(PWMC_Handle_t *pHdl, uint16_t hDuty);

/**
  * It computes and return latest converted motor phase currents motor
  * during RL detection phase
  */
void R3_1_F30X_RLGetPhaseCurrents(PWMC_Handle_t *pHdl,Curr_Components* pStator_Currents);

/**
  * It turns on low sides switches. This function is intended to be
  * used for charging boot capacitors of driving section. It has to be
  * called each motor start-up when using high voltage drivers.
  * This function is specific for RL detection phase.
  */
void R3_1_F30X_RLTurnOnLowSides(PWMC_Handle_t *pHdl);

/**
  * It enables PWM generation on the proper Timer peripheral
  * This function is specific for RL detection phase.
  */
void R3_1_F30X_RLSwitchOnPWM(PWMC_Handle_t *pHdl);

/**
  * It disables PWM generation on the proper Timer peripheral acting on
  * MOE bit
  */
void R3_1_F30X_RLSwitchOffPWM(PWMC_Handle_t *pHdl);
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

#endif /*__R3_1_F30X_PWMNCURRFDBK_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
