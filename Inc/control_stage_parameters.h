/**
  ******************************************************************************
  * @file    control_stage_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure the control stage of a motor application.
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
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONTROL_STAGE_PARAMETERS_H
#define __CONTROL_STAGE_PARAMETERS_H

#define CTRBDID 0

/***************************** MCU supply voltage *****************************/
#define MCU_SUPPLY_VOLTAGE    3.30

/***************************** CLOCK SETTINGS SECTION *************************/
#define CLOCK_SOURCE          EXTERNAL  /* EXTERNAL or INTERNAL */

#define CPU_CLK_72_MHZ

/* ext. clock frequency */
#define EXT_CLK_8_MHZ

/************************
 *** Motor Parameters ***
 ************************/

/* Maximum modulation index */
#define MAX_MODULATION_94_PER_CENT
                         
/************************ DIGITAL I/O DEFINITION SECTION  *********************/
/* PWM Timer section */
#define PWM_TIMER_SELECTION               PWM_TIM1
#define PWM_TIMER_REMAPPING               PWM_NO_REMAP
#define PHASE_UH_GPIO_PORT                GPIOA
#define PHASE_UH_GPIO_PIN                 LL_GPIO_PIN_8
#define PHASE_VH_GPIO_PORT                GPIOA
#define PHASE_VH_GPIO_PIN                 LL_GPIO_PIN_9
#define PHASE_WH_GPIO_PORT                GPIOA
#define PHASE_WH_GPIO_PIN                 LL_GPIO_PIN_10
#define PHASE_UL_GPIO_PORT                GPIOB
#define PHASE_UL_GPIO_PIN                 LL_GPIO_PIN_13
#define PHASE_VL_GPIO_PORT                GPIOB
#define PHASE_VL_GPIO_PIN                 LL_GPIO_PIN_14
#define PHASE_WL_GPIO_PORT                GPIOB
#define PHASE_WL_GPIO_PIN                 LL_GPIO_PIN_15
#define EMERGENCY_STOP_GPIO_PORT          GPIOA
#define EMERGENCY_STOP_GPIO_PIN           LL_GPIO_PIN_11

#define BKIN_MODE                         NONE
#define BKIN2_MODE                        EXT_MODE
#define EMERGENCY2_STOP_GPIO_PORT         GPIOA
#define EMERGENCY2_STOP_GPIO_PIN          LL_GPIO_PIN_11
#define PHASE_UH_GPIO_AF                  GPIO_AF_6
#define PHASE_VH_GPIO_AF                  GPIO_AF_6
#define PHASE_WH_GPIO_AF                  GPIO_AF_6
#define PHASE_UL_GPIO_AF                  GPIO_AF_6
#define PHASE_VL_GPIO_AF                  GPIO_AF_6
#define PHASE_WL_GPIO_AF                  GPIO_AF_4
#define BRKIN_GPIO_AF                     GPIO_AF_2
#define BRKIN2_GPIO_AF                    GPIO_AF_12

/* Hall timer section */
#define HALL_TIMER_SELECTION              HALL_TIM2
#define HALL_TIMER_REMAPPING              NO_REMAP_TIM2
#define H1_GPIO_PORT                      GPIOA
#define H2_GPIO_PORT                      GPIOA
#define H3_GPIO_PORT                      GPIOA

#define H1_GPIO_PIN                       LL_GPIO_PIN_0
#define H2_GPIO_PIN                       LL_GPIO_PIN_1
#define H3_GPIO_PIN                       LL_GPIO_PIN_2

/* Encoder timer selection */
#define ENC_TIMER_SELECTION               ENC_TIM2
#define ENC_TIMER_REMAPPING               NO_REMAP_TIM2
#define ENC_A_GPIO_PORT                   GPIOA
#define ENC_B_GPIO_PORT                   GPIOB
#define ENC_A_GPIO_PIN                    LL_GPIO_PIN_15
#define ENC_B_GPIO_PIN                    LL_GPIO_PIN_3

/* Digital Outputs */
#define R_BRAKE_GPIO_PORT                 GPIOE
#define R_BRAKE_GPIO_PIN                  LL_GPIO_PIN_5
#define OV_CURR_BYPASS_GPIO_PORT          GPIOE
#define OV_CURR_BYPASS_GPIO_PIN           LL_GPIO_PIN_5
#define INRUSH_CURRLIMIT_GPIO_PORT        GPIOE
#define INRUSH_CURRLIMIT_GPIO_PIN         LL_GPIO_PIN_4

/************************ ANALOG I/O DEFINITION SECTION  *********************/
/** Currents reading  **/
/* Only for three shunt resistors and ICS cases */
#define ADC_1_PERIPH                    ADC1
#define ADC_2_PERIPH                    ADC2
#define PHASE_U_CURR_ADC                ADC1
#define PHASE_U_CURR_CHANNEL            ADC_CHANNEL_6
#define PHASE_U_GPIO_PORT               GPIOC
#define PHASE_U_GPIO_PIN                LL_GPIO_PIN_0
#define PHASE_V_CURR_ADC                ADC1
#define PHASE_V_CURR_CHANNEL            ADC_CHANNEL_7
#define PHASE_V_GPIO_PORT               GPIOC
#define PHASE_V_GPIO_PIN                LL_GPIO_PIN_1
/* Only for three shunts case */
#define PHASE_W_CURR_ADC                ADC1
#define PHASE_W_CURR_CHANNEL            ADC_CHANNEL_8
#define PHASE_W_GPIO_PORT               GPIOC
#define PHASE_W_GPIO_PIN                LL_GPIO_PIN_2       
/* Only for 1 shunt resistor case */
#define ADC_PERIPH                      ADC1
#define PHASE_CURRENTS_CHANNEL          ADC_CHANNEL_9
#define PHASE_CURRENTS_GPIO_PORT        GPIOA
#define PHASE_CURRENTS_GPIO_PIN         LL_GPIO_PIN_0   

/* Common */
#define ADC_AHBPERIPH                   RCC_AHBPeriph_ADC12
#define ADC_CLOCK_WB_FREQ               72
#define ADC_CLOCK_WB_DIV                1
#define CURR_SAMPLING_TIME              19

/** Bus and temperature readings **/
#define REGCONVADC                      ADC1

#define VBUS_ADC                        ADC1
#define VBUS_CHANNEL                    ADC_CHANNEL_9
#define VBUS_GPIO_PORT                  GPIOC
#define VBUS_GPIO_PIN                   LL_GPIO_PIN_3
#define VBUS_ADC_SAMPLING_TIME          19

#define TEMP_FDBK_ADC                   ADC1
#define TEMP_FDBK_CHANNEL               ADC_CHANNEL_4
#define TEMP_FDBK_GPIO_PORT             GPIOA
#define TEMP_FDBK_GPIO_PIN              LL_GPIO_PIN_3
#define TEMP_ADC_SAMPLING_TIME          19

/* OPAMP Settings */

#define USE_INTERNAL_OPAMP                     DISABLE

/*
 * WB generates only OPAMPx (x=1,2..,4) syntax but when a mcu has only one opamp
 * the syntax is OPAMP (ex: STM32F302R8). Then to handle the following compilation 
 * flag has been introduced 
 */
#if ((defined STM32F301x8)||(defined STM32F302x8))
#define OPAMP1_SELECTION                       OPAMP     
#else
#define OPAMP1_SELECTION                       OPAMP1
#endif
#define OPAMP1_INVERTINGINPUT_MODE             INT_MODE
#define OPAMP1_INVERTINGINPUT                  OPAMP1_InvertingInput_PGA
#define OPAMP1_INVERTINGINPUT_GPIO_PORT        GPIOA
#define OPAMP1_INVERTINGINPUT_GPIO_PIN         LL_GPIO_PIN_3
#define OPAMP1_NONINVERTINGINPUT_PHA           OPAMP1_NonInvertingInput_PA1
#define OPAMP1_NONINVERTINGINPUT_PHA_GPIO_PORT GPIOA
#define OPAMP1_NONINVERTINGINPUT_PHA_GPIO_PIN  LL_GPIO_PIN_1
#define OPAMP1_NONINVERTINGINPUT_PHB           OPAMP1_NonInvertingInput_PA7
#define OPAMP1_NONINVERTINGINPUT_PHB_GPIO_PORT GPIOA
#define OPAMP1_NONINVERTINGINPUT_PHB_GPIO_PIN  LL_GPIO_PIN_7
#define OPAMP1_OUT_GPIO_PORT                   GPIOA
#define OPAMP1_OUT_GPIO_PIN                    LL_GPIO_PIN_2

#define OPAMP2_SELECTION                       OPAMP2
#define OPAMP2_INVERTINGINPUT_MODE             INT_MODE
#define OPAMP2_INVERTINGINPUT                  OPAMP2_InvertingInput_PGA
#define OPAMP2_INVERTINGINPUT_GPIO_PORT        GPIOC
#define OPAMP2_INVERTINGINPUT_GPIO_PIN         LL_GPIO_PIN_5
#define OPAMP2_NONINVERTINGINPUT_PHA           OPAMP2_NonInvertingInput_PB14
#define OPAMP2_NONINVERTINGINPUT_PHA_GPIO_PORT GPIOB
#define OPAMP2_NONINVERTINGINPUT_PHA_GPIO_PIN  LL_GPIO_PIN_14
#define OPAMP2_NONINVERTINGINPUT_PHB           OPAMP2_NonInvertingInput_PA7
#define OPAMP2_NONINVERTINGINPUT_PHB_GPIO_PORT GPIOA
#define OPAMP2_NONINVERTINGINPUT_PHB_GPIO_PIN  LL_GPIO_PIN_7
#define OPAMP2_NONINVERTINGINPUT_PHC           OPAMP2_NonInvertingInput_PB14
#define OPAMP2_NONINVERTINGINPUT_PHC_GPIO_PORT GPIOB
#define OPAMP2_NONINVERTINGINPUT_PHC_GPIO_PIN  LL_GPIO_PIN_14
#define OPAMP2_OUT_GPIO_PORT                   GPIOA
#define OPAMP2_OUT_GPIO_PIN                    LL_GPIO_PIN_6

/* Only for 1 shunt resistor case */
/*
 * WB generates only OPAMPx (x=1,2..,4) syntax but when a mcu has only one opamp
 * the syntax is OPAMP (ex: STM32F302R8). Then to handle the following compilation 
 * flag has been introduced 
 */
#if ((defined STM32F301x8)||(defined STM32F302x8))
#define OPAMP_SELECTION                        OPAMP     
#else
#define OPAMP_SELECTION                        OPAMP1
#endif
#define OPAMP_INVERTINGINPUT_MODE              INT_MODE
#define OPAMP_INVERTINGINPUT                   OPAMP1_InvertingInput_PGA
#define OPAMP_INVERTINGINPUT_GPIO_PORT         GPIOA
#define OPAMP_INVERTINGINPUT_GPIO_PIN          LL_GPIO_PIN_3
#define OPAMP_NONINVERTINGINPUT                OPAMP1_NonInvertingInput_PA7
#define OPAMP_NONINVERTINGINPUT_GPIO_PORT      GPIOA
#define OPAMP_NONINVERTINGINPUT_GPIO_PIN       LL_GPIO_PIN_7
#define OPAMP_OUT_GPIO_PORT                    GPIOA
#define OPAMP_OUT_GPIO_PIN                     LL_GPIO_PIN_2

/* OPAMP common settings*/
#define OPAMP_PGAGAIN                          OPAMP_OPAMP_PGAGain_2
#define OPAMP_PGACONNECT                       OPAMP_PGAConnect_No

/* COMP Settings */

#define INTERNAL_OVERCURRENTPROTECTION    DISABLE
#define OCPREF                            23830

#define INTERNAL_OVERVOLTAGEPROTECTION    DISABLE
#define OVPREF                            23830

/* Only for 1 shunt resistor case */
#define OCP_SELECTION                     COMP2
#define OCP_INVERTINGINPUT_MODE           INT_MODE
#define OCP_INVERTINGINPUT                COMPX_InvertingInput_VREF
#define OCP_INVERTINGINPUT_GPIO_PORT      GPIOA
#define OCP_INVERTINGINPUT_GPIO_PIN       LL_GPIO_PIN_4
#define OCP_NONINVERTINGINPUT             COMP2_NonInvertingInput_PA7
#define OCP_NONINVERTINGINPUT_GPIO_PORT   GPIOA
#define OCP_NONINVERTINGINPUT_GPIO_PIN    LL_GPIO_PIN_7
#define OCP_OUTPUT_MODE                   EXT_MODE
#define OCP_OUTPUT                        COMP_Output_TIM1BKIN2
#define OCP_OUTPUT_GPIO_PORT              GPIOA
#define OCP_OUTPUT_GPIO_PIN               LL_GPIO_PIN_2
#define OCP_OUTPUT_GPIO_AF                GPIO_AF_8
#define OCP_OUTPUTPOL                     COMP_OutputPol_NonInverted

#define OCPA_SELECTION                    COMP2
#define OCPA_INVERTINGINPUT_MODE          INT_MODE
#define OCPA_INVERTINGINPUT               COMPX_InvertingInput_VREF
#define OCPA_INVERTINGINPUT_GPIO_PORT     GPIOA
#define OCPA_INVERTINGINPUT_GPIO_PIN      LL_GPIO_PIN_4
#define OCPA_NONINVERTINGINPUT            COMP2_NonInvertingInput_PA7
#define OCPA_NONINVERTINGINPUT_GPIO_PORT  GPIOA
#define OCPA_NONINVERTINGINPUT_GPIO_PIN   LL_GPIO_PIN_7
#define OCPA_OUTPUT_MODE                  EXT_MODE
#define OCPA_OUTPUT                       COMP_Output_TIM1BKIN2
#define OCPA_OUTPUT_GPIO_PORT             GPIOA
#define OCPA_OUTPUT_GPIO_PIN              LL_GPIO_PIN_12
#define OCPA_OUTPUT_GPIO_AF               GPIO_AF_8
#define OCPA_OUTPUTPOL                    COMP_OutputPol_NonInverted

#define OCPB_SELECTION                    COMP4
#define OCPB_INVERTINGINPUT_MODE          INT_MODE
#define OCPB_INVERTINGINPUT               COMPX_InvertingInput_VREF
#define OCPB_INVERTINGINPUT_GPIO_PORT     GPIOA
#define OCPB_INVERTINGINPUT_GPIO_PIN      LL_GPIO_PIN_4
#define OCPB_NONINVERTINGINPUT            COMP4_NonInvertingInput_PB0
#define OCPB_NONINVERTINGINPUT_GPIO_PORT  GPIOB
#define OCPB_NONINVERTINGINPUT_GPIO_PIN   LL_GPIO_PIN_0
#define OCPB_OUTPUT_MODE                  EXT_MODE
#define OCPB_OUTPUT                       COMP_Output_TIM1BKIN2
#define OCPB_OUTPUT_GPIO_PORT             GPIOB
#define OCPB_OUTPUT_GPIO_PIN              LL_GPIO_PIN_1
#define OCPB_OUTPUT_GPIO_AF               GPIO_AF_8
#define OCPB_OUTPUTPOL                    COMP_OutputPol_NonInverted

#define OCPC_SELECTION                    COMP6
#define OCPC_INVERTINGINPUT_MODE          INT_MODE
#define OCPC_INVERTINGINPUT               COMPX_InvertingInput_VREF
#define OCPC_INVERTINGINPUT_GPIO_PORT     GPIOA
#define OCPC_INVERTINGINPUT_GPIO_PIN      LL_GPIO_PIN_4
#define OCPC_NONINVERTINGINPUT            COMP6_NonInvertingInput_PB11
#define OCPC_NONINVERTINGINPUT_GPIO_PORT  GPIOB
#define OCPC_NONINVERTINGINPUT_GPIO_PIN   LL_GPIO_PIN_11
#define OCPC_OUTPUT_MODE                  EXT_MODE
#define OCPC_OUTPUT                       COMP_Output_TIM1BKIN2
#define OCPC_OUTPUT_GPIO_PORT             GPIOA
#define OCPC_OUTPUT_GPIO_PIN              LL_GPIO_PIN_10
#define OCPC_OUTPUT_GPIO_AF               GPIO_AF_8
#define OCPC_OUTPUTPOL                    COMP_OutputPol_NonInverted
                                                              
#define OVP_SELECTION                     COMP2
#define OVP_INVERTINGINPUT_MODE           INT_MODE
#define OVP_INVERTINGINPUT                COMPX_InvertingInput_VREF
#define OVP_INVERTINGINPUT_GPIO_PORT      GPIOA
#define OVP_INVERTINGINPUT_GPIO_PIN       LL_GPIO_PIN_4
#define OVP_NONINVERTINGINPUT             COMP2_NonInvertingInput_PA7
#define OVP_NONINVERTINGINPUT_GPIO_PORT   GPIOA
#define OVP_NONINVERTINGINPUT_GPIO_PIN    LL_GPIO_PIN_7
#define OVP_OUTPUT_MODE                   INT_MODE
#define OVP_OUTPUT                        COMP_Output_TIM1BKIN
#define OVP_OUTPUT_GPIO_PORT              GPIOA
#define OVP_OUTPUT_GPIO_PIN               LL_GPIO_PIN_2
#define OVP_OUTPUT_GPIO_AF                GPIO_AF_8
#define OVP_OUTPUTPOL                     COMP_OutputPol_NonInverted

#define HIGH_SIDE_BRAKE_STATE             TURN_OFF
#define LOW_SIDE_BRAKE_STATE              TURN_OFF

#define BKIN1_FILTER                      0
#define BKIN2_FILTER                      5

#define OCP_FILTER                        COMP_Mode_HighSpeed
#define OVP_FILTER                        COMP_Mode_HighSpeed

/**************************
 *** Control Parameters ***
 **************************/

/* Debug Setting */
#define DAC_FUNCTIONALITY                ENABLE
#define DEBUG_DAC_CH1                    ENABLE
#define DEBUG_DAC_CH2                    DISABLE
#define DEFAULT_DAC_CHANNEL_1            MC_PROTOCOL_REG_I_Q_REF
#define DEFAULT_DAC_CHANNEL_2            MC_PROTOCOL_REG_I_B
#define DEFAULT_DAC_MOTOR                0

#define DAC_TIMER_SELECTION              TIM3
#define DAC_TIMER_REMAPPING              NO_REMAP
#define DAC_TIMER_CH1_GPIO_PORT          GPIOA
#define DAC_TIMER_CH1_GPIO_PIN           LL_GPIO_PIN_4
#define DAC_TIMER_CH2_GPIO_PORT          GPIOA
#define DAC_TIMER_CH2_GPIO_PIN           LL_GPIO_PIN_0

#define SW_OV_CURRENT_PROT_ENABLING      ENABLE

#define START_STOP_GPIO_PORT             GPIOE
#define START_STOP_GPIO_PIN              LL_GPIO_PIN_6
#define START_STOP_POLARITY              DIN_ACTIVE_LOW

/* Serial communication */
#define USART_SELECTION                 USE_USART3
#define USART_REMAPPING                 NO_REMAP_USART3
#define USART_TX_GPIO_PORT              GPIOB
#define USART_TX_GPIO_PIN               LL_GPIO_PIN_10
#define USART_RX_GPIO_PORT              GPIOB
#define USART_RX_GPIO_PIN               LL_GPIO_PIN_11
#define USART_SPEED                     115200

#endif /*__CONTROL_STAGE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
