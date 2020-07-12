/**
  ******************************************************************************
  * @file    parameters_conversion_f30x.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions needed to convert MC SDK parameters
  *          so as to target the STM32F3 Family.
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
#ifndef __PARAMETERS_CONVERSION_F30X_H
#define __PARAMETERS_CONVERSION_F30X_H

#include "Definitions.h"
#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"
#include "power_stage_parameters.h"
#include "control_stage_parameters.h"
#include "mc_math.h"

#define SQRT_2  1.4142
#define SQRT_3  1.732

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/

#define ADC_CLOCK_DIVIDER  ADC_Clock_SynClkModeDiv1
#define ADC_CLOCK_DIV      ADC_CLOCK_WB_DIV

#define SYSCLK_FREQ_72MHz  72000000uL
#define TIM_CLOCK_DIVIDER  1
#define ADV_TIM_CLK_MHz    (72uL/TIM_CLOCK_DIVIDER)
#define ADC_CLK_MHz        (72uL/ADC_CLOCK_DIV)
#define HALL_TIM_CLK       SYSCLK_FREQ_72MHz

/************************* CONTROL FREQUENCIES & DELAIES **********************/
#define TF_REGULATION_RATE 	(uint16_t) ((uint16_t)(PWM_FREQUENCY)/REGULATION_EXECUTION_RATE)
#define REP_COUNTER 			(uint16_t) ((REGULATION_EXECUTION_RATE *2u)-1u)

#define MEDIUM_FREQUENCY_TASK_RATE	(uint16_t)SPEED_LOOP_FREQUENCY_HZ

#define INRUSH_CURRLIMIT_DELAY_COUNTS  (uint16_t)(INRUSH_CURRLIMIT_DELAY_MS * \
                                  ((uint16_t)SPEED_LOOP_FREQUENCY_HZ)/1000u -1u)
#define SYS_TICK_FREQUENCY          2000
#define UI_TASK_FREQUENCY_HZ        10
#define SERIAL_COM_TIMEOUT_INVERSE  25
#define SERIAL_COM_ATR_TIME_MS 20

#define MF_TASK_OCCURENCE_TICKS  (SYS_TICK_FREQUENCY/SPEED_LOOP_FREQUENCY_HZ)-1u
#define UI_TASK_OCCURENCE_TICKS  (SYS_TICK_FREQUENCY/UI_TASK_FREQUENCY_HZ)-1u
#define SERIALCOM_TIMEOUT_OCCURENCE_TICKS (SYS_TICK_FREQUENCY/SERIAL_COM_TIMEOUT_INVERSE)-1u
#define SERIALCOM_ATR_TIME_TICKS (uint16_t)(((SYS_TICK_FREQUENCY * SERIAL_COM_ATR_TIME_MS) / 1000u) - 1u)

/*********************** SENSORLESS REV-UP PARAMETERS *************************/
#define FIRST_SLESS_ALGO_PHASE (ENABLE_SL_ALGO_FROM_PHASE-1u)  

/************************* OBSERVER + PLL PARAMETERS **************************/
#define MAX_BEMF_VOLTAGE  (uint16_t)((MAX_APPLICATION_SPEED * 1.2 *\
                           MOTOR_VOLTAGE_CONSTANT*SQRT_2)/(1000u*SQRT_3))

/*max phase voltage, 0-peak Volts*/
#define MAX_VOLTAGE (int16_t)((MCU_SUPPLY_VOLTAGE/2)/BUS_ADC_CONV_RATIO) 

#define MAX_CURRENT (MCU_SUPPLY_VOLTAGE/(2*RSHUNT*AMPLIFICATION_GAIN))

#define C1 (int32_t)((((int16_t)F1)*RS)/(LS*TF_REGULATION_RATE))
#define C2 (int32_t) GAIN1
#define C3 (int32_t)((((int16_t)F1)*MAX_BEMF_VOLTAGE)/(LS*MAX_CURRENT*TF_REGULATION_RATE))
#define C4 (int32_t) GAIN2
#define C5 (int32_t)((((int16_t)F1)*MAX_VOLTAGE)/(LS*MAX_CURRENT*TF_REGULATION_RATE))

#define PERCENTAGE_FACTOR    (uint16_t)(VARIANCE_THRESHOLD*128u)      
#define OBS_MINIMUM_SPEED        (uint16_t) (OBS_MINIMUM_SPEED_RPM/6u)
/*********************** OBSERVER + CORDIC PARAMETERS *************************/
#define CORD_C1 (int32_t)((((int16_t)CORD_F1)*RS)/(LS*TF_REGULATION_RATE))
#define CORD_C2 (int32_t) CORD_GAIN1
#define CORD_C3 (int32_t)((((int16_t)CORD_F1)*MAX_BEMF_VOLTAGE)/(LS*MAX_CURRENT\
                                                           *TF_REGULATION_RATE))
#define CORD_C4 (int32_t) CORD_GAIN2
#define CORD_C5 (int32_t)((((int16_t)CORD_F1)*MAX_VOLTAGE)/(LS*MAX_CURRENT*\
                                                          TF_REGULATION_RATE))
#define CORD_PERCENTAGE_FACTOR    (uint16_t)(CORD_VARIANCE_THRESHOLD*128u)      
#define CORD_MINIMUM_SPEED        (uint16_t) (CORD_MINIMUM_SPEED_RPM/6u)
/**************************   VOLTAGE CONVERSIONS  ****************************/
#define BUS_ADC_CONV_RATIO       VBUS_PARTITIONING_FACTOR

#define OVERVOLTAGE_THRESHOLD_d   (uint16_t)(OV_VOLTAGE_THRESHOLD_V*65535/\
                                  (MCU_SUPPLY_VOLTAGE/VBUS_PARTITIONING_FACTOR))
#define UNDERVOLTAGE_THRESHOLD_d  (uint16_t)((UD_VOLTAGE_THRESHOLD_V*65535)/\
                                  ((uint16_t)(MCU_SUPPLY_VOLTAGE/\
                                                           BUS_ADC_CONV_RATIO)))
#define INT_SUPPLY_VOLTAGE          (uint16_t)(65536/MCU_SUPPLY_VOLTAGE)

#define DELTA_TEMP_THRESHOLD        (OV_TEMPERATURE_THRESHOLD_C- T0_C)
#define DELTA_V_THRESHOLD           (dV_dT * DELTA_TEMP_THRESHOLD)
#define OV_TEMPERATURE_THRESHOLD_d  ((V0_V + DELTA_V_THRESHOLD)*INT_SUPPLY_VOLTAGE)

#define DELTA_TEMP_HYSTERESIS        (OV_TEMPERATURE_HYSTERESIS_C)
#define DELTA_V_HYSTERESIS           (dV_dT * DELTA_TEMP_HYSTERESIS)
#define OV_TEMPERATURE_HYSTERESIS_d  (DELTA_V_HYSTERESIS*INT_SUPPLY_VOLTAGE)
/*************************  PWM IDLE STATES AND POLARITY  *********************/														  
#define H_ACTIVE_HIGH    0x0000u //TIM_OCPolarity_High
#define H_ACTIVE_LOW     0x0002u //TIM_OCPolarity_Low
#define L_ACTIVE_HIGH    0x0000u  //TIM_OCNPolarity_High
#define L_ACTIVE_LOW     0x0008u  //TIM_OCNPolarity_Low 

#define DOUT_ACTIVE_HIGH   DOutputActiveHigh
#define DOUT_ACTIVE_LOW    DOutputActiveLow

#define EMSTOP_ACTIVE_HIGH    TIM_Break1Polarity_High
#define EMSTOP_ACTIVE_LOW     TIM_Break1Polarity_Low
#define EMSTOP2_ACTIVE_HIGH   TIM_Break2Polarity_High
#define EMSTOP2_ACTIVE_LOW    TIM_Break2Polarity_Low

#define IDLE_UH_POLARITY  TIM_OCIDLESTATE_RESET

#define IDLE_VH_POLARITY  TIM_OCIDLESTATE_RESET

#define IDLE_WH_POLARITY  TIM_OCIDLESTATE_RESET

#define BRAKE_UH_POLARITY  TIM_OCIDLESTATE_RESET

#define BRAKE_VH_POLARITY  TIM_OCIDLESTATE_RESET

#define BRAKE_WH_POLARITY  TIM_OCIDLESTATE_RESET

#define IDLE_UL_POLARITY  TIM_OCNIDLESTATE_RESET

#define IDLE_VL_POLARITY  TIM_OCNIDLESTATE_RESET

#define IDLE_WL_POLARITY  TIM_OCNIDLESTATE_RESET

#define BRAKE_UL_POLARITY  TIM_OCNIDLESTATE_RESET

#define BRAKE_VL_POLARITY  TIM_OCNIDLESTATE_RESET

#define BRAKE_WL_POLARITY  TIM_OCNIDLESTATE_RESET

#define PWM_TIM1 0x1 /* Dummy value for comparison */
#define PWM_TIM8 0x8 /* Dummy value for comparison */
#define TIMx_UP_M1_IRQHandler TIM1_UP_TIM16_IRQHandler
#define TIMx_UP_M1_IRQFlag 0
#define TIMx_BRK_M1_IRQHandler TIM1_BRK_TIM15_IRQHandler
#undef PWM_TIM1
#undef PWM_TIM8

#define PWM_TIM1			TIM1
#define PWM_TIM8                        TIM8
#define PWM_NO_REMAP		0 //Dummy
#define PWM_FULL_REMAP 		0 //Dummy
#define PWM_PARTIAL_REMAP 	0 //

/**********  AUXILIARY TIMER (SINGLE SHUNT) *************/
#define R1_PWM_AUX_TIM                  TIM4

/**********  SPEED FEEDBACK TIMERS SETTING *************/
  #define HALL_TIMER 				TIM2  
  #define HALL_RCC_PERIPHERAL		RCC_APB1Periph_TIM2  
  #define HALL_IRQ_CHANNEL  		TIM2_IRQn

  /* Dummy value to avoid compiler error */
  #undef  ENC_TIMER
  #undef  ENC_TIMER_REMAPPING
  #undef  ENC_RCC_PERIPHERAL
  #undef  ENC_IRQ_CHANNEL
  #undef  ENC_A_GPIO_PORT
  #undef  ENC_B_GPIO_PORT
  #undef  ENC_A_GPIO_PIN
  #undef  ENC_B_GPIO_PIN

  #define ENC_TIMER 				    TIM2
  #define ENC_TIMER_REMAPPING   NO_REMAP_TIM2
  #define ENC_RCC_PERIPHERAL		RCC_APB1Periph_TIM2
  #define ENC_IRQ_CHANNEL  		  TIM2_IRQn
  #define ENC_A_GPIO_PORT       GPIOA
  #define ENC_B_GPIO_PORT       GPIOA
  #define ENC_A_GPIO_PIN        LL_GPIO_PIN_0
  #define ENC_B_GPIO_PIN        LL_GPIO_PIN_1

/* Resistive Brake */
  /* Dummy value to avoid compiler error */
  #undef R_BRAKE_GPIO_PORT
  #undef R_BRAKE_GPIO_PIN
  
  #define R_BRAKE_GPIO_PORT     GPIOD
  #define R_BRAKE_GPIO_PIN      LL_GPIO_PIN_13

/* Hardware over current protection bypass */
  /* Dummy value to avoid compiler error */
  #undef OV_CURR_BYPASS_GPIO_PORT
  #undef OV_CURR_BYPASS_GPIO_PIN
  
  #define OV_CURR_BYPASS_GPIO_PORT  GPIOD        
  #define OV_CURR_BYPASS_GPIO_PIN   LL_GPIO_PIN_13

/* Inrush current limiter */
  /* Dummy value to avoid compiler error */
  #undef INRUSH_CURRLIMIT_GPIO_PORT
  #undef INRUSH_CURRLIMIT_GPIO_PIN
  
  #define INRUSH_CURRLIMIT_GPIO_PORT  GPIOD
  #define INRUSH_CURRLIMIT_GPIO_PIN   LL_GPIO_PIN_10

/* USART */

/* Current sensing topology */
  /* Dummy value to avoid compiler error */
  #undef PHASE_CURRENTS_ADC
  #undef PHASE_CURRENTS_CHANNEL
  #undef PHASE_CURRENTS_GPIO_PORT
  #undef PHASE_CURRENTS_GPIO_PIN
  
  #define PHASE_CURRENTS_ADC              ADC3
  #define PHASE_CURRENTS_CHANNEL          ADC_CHANNEL_12
  #define PHASE_CURRENTS_GPIO_PORT        GPIOC
  #define PHASE_CURRENTS_GPIO_PIN         LL_GPIO_PIN_2

#define SPD_TIM_M1_IRQHandler TIM2_IRQHandler

#define HALL_IC_FILTER   14

#define ENC_IC_FILTER   8

/*************** Encoder Alignemnt ************************/
    
/* Encoder alignment */
#define T_ALIGNMENT              ALIGNMENT_DURATION
#define ALIGNMENT_ANGLE_S16      (int16_t)  (ALIGNMENT_ANGLE_DEG*65536u/360u)

/*************** Timer for speed/position feedback   ******/
#define FULL_REMAP_TIM2			GPIO_FullRemap_TIM2
#define PARTIAL_REMAP_01_TIM2		GPIO_PartialRemap1_TIM2
#define PARTIAL_REMAP_10_TIM2		GPIO_PartialRemap2_TIM2
#define NO_REMAP_TIM2                   GPIO_NoRemap_TIMx

#define FULL_REMAP_TIM3			GPIO_FullRemap_TIM3
#define PARTIAL_REMAP_TIM3		GPIO_PartialRemap_TIM3
#define NO_REMAP_TIM3                   GPIO_NoRemap_TIMx

#define REMAP_TIM4			GPIO_Remap_TIM4
#define NO_REMAP_TIM4                   GPIO_NoRemap_TIMx

#define NO_REMAP_TIM5                   GPIO_NoRemap_TIMx

/*************** Timer for PWM generation & currenst sensing parameters  ******/
#define PWM_PERIOD_CYCLES (uint16_t)(ADV_TIM_CLK_MHz*\
                                      (unsigned long long)1000000u/((uint16_t)(PWM_FREQUENCY)))
#define DEADTIME_NS  SW_DEADTIME_NS

#define DEAD_TIME_ADV_TIM_CLK_MHz (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER)
#define DEAD_TIME_COUNTS_1  (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/1000uL)

#if (DEAD_TIME_COUNTS_1 <= 255)
#define DEAD_TIME_COUNTS (uint16_t) DEAD_TIME_COUNTS_1
#elif (DEAD_TIME_COUNTS_1 <= 508)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/2) /1000uL) + 128)
#elif (DEAD_TIME_COUNTS_1 <= 1008)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/8) /1000uL) + 320)
#elif (DEAD_TIME_COUNTS_1 <= 2015)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/16) /1000uL) + 384)
#else
#define DEAD_TIME_COUNTS 510
#endif

#define DTCOMPCNT (uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz) / 2000)
#define TON_NS  500
#define TOFF_NS 500
#define TON  (uint16_t)((TON_NS * ADV_TIM_CLK_MHz)  / 2000)
#define TOFF (uint16_t)((TOFF_NS * ADV_TIM_CLK_MHz) / 2000)

#define MAX_TNTR_NS TRISE_NS

#define SAMPLING_TIME_NS (((CURR_SAMPLING_TIME) * 1000uL/ADC_CLK_MHz)+(7000uL/(2*ADC_CLK_MHz)))
#define SAMPLING_TIME_SEL  LL_ADC_SAMPLINGTIME_19CYCLES_5

#define SAMPLING_TIME (uint16_t)(((uint16_t)(SAMPLING_TIME_NS) * ADV_TIM_CLK_MHz)/1000uL) 
#define TRISE (uint16_t)((((uint16_t)(TRISE_NS)) * ADV_TIM_CLK_MHz)/1000uL)
#define TDEAD (uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz)/1000uL)

#define TMIN (((uint16_t)(((DEADTIME_NS+((uint16_t)(TRISE_NS))+\
			 ((uint16_t)(SAMPLING_TIME_NS)))*ADV_TIM_CLK_MHz)/1000ul))+1)
#define HTMIN (uint16_t)(TMIN >> 1)
#define CHTMIN (uint16_t)(TMIN/(REGULATION_EXECUTION_RATE*2))
#define TSAMPLE SAMPLING_TIME
#define TAFTER ((uint16_t)(((DEADTIME_NS+((uint16_t)(TRISE_NS)))\
												   *ADV_TIM_CLK_MHz)/1000ul))
#define TBEFORE (((uint16_t)(((((uint16_t)(SAMPLING_TIME_NS)))\
											*ADV_TIM_CLK_MHz)/1000ul))+1)

#if (TRISE_NS > SAMPLING_TIME_NS)
#define MAX_TRTS (2 * TRISE)
#else
#define MAX_TRTS (2 * SAMPLING_TIME)
#endif

#define TNOISE (uint16_t)((((uint16_t)(TNOISE_NS)) * ADV_TIM_CLK_MHz)/1000uL)

#define MAX_TNTR_NS TRISE_NS

#define TW_AFTER ((uint16_t)(((DEADTIME_NS+MAX_TNTR_NS)*ADV_TIM_CLK_MHz)/1000ul))
#define TW_BEFORE (((uint16_t)(((((uint16_t)(SAMPLING_TIME_NS)))*ADV_TIM_CLK_MHz)/1000ul))+1u)

#define ADC_CONV_NB_CK 13u
#define TW_BEFORE_R3_1 (((uint16_t)(((((uint16_t)(SAMPLING_TIME_NS * 2)))*ADV_TIM_CLK_MHz)/1000ul))+ 1u + ADC_CONV_NB_CK)

#define START_INDEX     56
#define MAX_MODULE      30800   //root(Vd^2+Vq^2) <= MAX_MODULE = 32767*94%
#define MMITABLE {\
32607,32293,31988,31691,31546,31261,30984,30714,30451,30322,\
30069,29822,29581,29346,29231,29004,28782,28565,28353,28249,\
28044,27843,27647,27455,27360,27174,26991,26812,26724,26550,\
26380,26213,26049,25968,25808,25652,25498,25347,25272,25125,\
24981,24839,24699,24630,24494,24360,24228,24098,24034,23908,\
23783,23660,23600,23480,23361,23245,23131,23074,22962,22851,\
22742,22635,22582,22477,22373,22271,22170,22120,22021,21924,\
21827,21732\
}

/*************** PI divisor  ***************/
#define SP_KPDIV_LOG LOG2(1)
#define SP_KIDIV_LOG LOG2(1)
#define SP_KDDIV_LOG LOG2(16)
#define TF_KPDIV_LOG LOG2(2048)
#define TF_KIDIV_LOG LOG2(2048)
#define TF_KDDIV_LOG LOG2(8192)
#define FW_KPDIV_LOG LOG2(32768)
#define FW_KIDIV_LOG LOG2(32768)
#define PLL_KPDIV     16384
#define PLL_KPDIV_LOG LOG2(PLL_KPDIV)
#define PLL_KIDIV     65535
#define PLL_KIDIV_LOG LOG2(PLL_KIDIV)
#define F1_LOG LOG2(8192)
#define F2_LOG LOG2(16384)
#define STO_FIFO_DEPTH_DPP_LOG LOG2(64)
#define CORD_FIFO_DEPTH_DPP_LOG LOG2(64)
#define HFI_PID_KPDIV_LOG LOG2(16384)
#define HFI_PID_KIDIV_LOG LOG2(32768)

#define VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE   25u

#define TEMP_SW_FILTER_BW_FACTOR      250u
#define VQD_SW_FILTER_BW_FACTOR       128u
#define VBUS_SW_FILTER_BW_FACTOR      6u

#define VQD_SW_FILTER_BW_FACTOR_LOG LOG2(VQD_SW_FILTER_BW_FACTOR)

#define VBUS_SAMPLING_TIME  LL_ADC_SAMPLINGTIME_19CYCLES_5

#define TEMP_SAMPLING_TIME  LL_ADC_SAMPLINGTIME_19CYCLES_5

/* Number of MC object */
#define MC_NUM 1

#define PQD_CONVERSION_FACTOR (int32_t)(( 1000 * 3 * MCU_SUPPLY_VOLTAGE ) /\
             ( 1.732 * RSHUNT * AMPLIFICATION_GAIN ))

/*********************** Usart remapping  ***********************/
#define USE_USART1 0
#define USE_USART2 1
#define USE_USART3 2
#define USE_USART4 3
#define USE_USART5 4

/* Dummy */
#define NO_REMAP_USART1       0
#define REMAP_USART1          1
#define NO_REMAP_USART2       2
#define REMAP_USART2          3
#define NO_REMAP_USART3       4
#define FULL_REMAP_USART3     5
#define PARTIAL_REMAP_USART3  6

#define GPIO_NoRemap_USARTx 0

  #define USART                   USART3
  #define USART_CLK               RCC_APB1Periph_USART3
  #define USART_IRQ               USART3_IRQn
  #define USART_IRQHandler        USART3_IRQHandler
  /* Dummy */
  #define USART_GPIO_REMAP      GPIO_NoRemap_USARTx

#define NVIC_USART_INIT_STR \
const NVIC_InitTypeDef NVICInitHW_str =\
{\
  USART_IRQ,  /* NVIC_IRQChannel */\
  USART_PRE_EMPTION_PRIORITY,            /* NVIC_IRQChannelPreemptionPriority */\
  USART_SUB_PRIORITY,            /* NVIC_IRQChannelSubPriority */\
  (FunctionalState)(ENABLE)        /* NVIC_IRQChannelCmd */\
};

/****** Prepares the UI configurations according the MCconfxx settings ********/
#define LCD_ENABLE 

#define COM_ENABLE | OPT_COM

#define DAC_ENABLE | OPT_DAC
#define DAC_OPTION | OPT_DACF3
#define DAC_OP_ENABLE | UI_CFGOPT_DAC

/* Motor 1 settings */
#define FW_ENABLE

#define DIFFTERM_ENABLE

/* Sensors setting */

#define MAIN_SCFG UI_SCODE_HALL

#define AUX_SCFG 0x0

#define PLLTUNING_ENABLE

#define UI_CFGOPT_PFC_ENABLE

/******************************************************************************* 
  * UI configurations settings. It can be manually overwritten if special 
  * configuartion is required. 
*******************************************************************************/

/* Base configuration of UI */
#define UI_INIT_CFG ( OPT_NONE LCD_ENABLE COM_ENABLE DAC_ENABLE )

/* Specific options of UI */
#define UI_CONFIG_M1 ( UI_CFGOPT_NONE DAC_OP_ENABLE FW_ENABLE DIFFTERM_ENABLE \
  | (MAIN_SCFG << MAIN_SCFG_POS) | (AUX_SCFG << AUX_SCFG_POS) | UI_CFGOPT_SETIDINSPDMODE PLLTUNING_ENABLE UI_CFGOPT_PFC_ENABLE | UI_CFGOPT_PLLTUNING)

#define UI_CONFIG_M2

#define OCP_OFFSET (((float)(OCP_V)) * (OCP_R1/(OCP_R1 + OCP_R2)))
#define OCP_TH_V (OCP_OFFSET + ((float)(RSHUNT)*OCP_THRESHOLD)*(OCP_R2/(OCP_R1+OCP_R2)))
#define OCP_DAC_VREF (uint16_t)((OCP_TH_V * DAC_MAX_DIGIT)/DAC_VMAX)
#define DAC_VMAX ((float)(MCU_SUPPLY_VOLTAGE))
#define DAC_MAX_DIGIT 65536.0f

#define OVP_DAC_VREF (uint16_t)(((float)(OV_VOLTAGE_THRESHOLD_V)*(float)(VBUS_PARTITIONING_FACTOR)*DAC_MAX_DIGIT)/DAC_VMAX)

#define OPAMP1_InvertingInput_PC5         LL_OPAMP_INPUT_INVERT_IO0
#define OPAMP1_InvertingInput_PA3         LL_OPAMP_INPUT_INVERT_IO1 
#define OPAMP1_InvertingInput_PGA         LL_OPAMP_INPUT_INVERT_CONNECT_NO
#define OPAMP1_InvertingInput_FOLLOWER    LL_OPAMP_MODE_FOLLOWER
#define OPAMP2_InvertingInput_PC5         LL_OPAMP_INPUT_INVERT_IO0
#define OPAMP2_InvertingInput_PA5         LL_OPAMP_INPUT_INVERT_IO1 
#define OPAMP2_InvertingInput_PGA         LL_OPAMP_INPUT_INVERT_CONNECT_NO
#define OPAMP2_InvertingInput_FOLLOWER    LL_OPAMP_MODE_FOLLOWER
#define OPAMP3_InvertingInput_PB10        LL_OPAMP_INPUT_INVERT_IO0
#define OPAMP3_InvertingInput_PB2         LL_OPAMP_INPUT_INVERT_IO1 
#define OPAMP3_InvertingInput_PGA         LL_OPAMP_INPUT_INVERT_CONNECT_NO
#define OPAMP3_InvertingInput_FOLLOWER    LL_OPAMP_MODE_FOLLOWER
#define OPAMP4_InvertingInput_PB10        LL_OPAMP_INPUT_INVERT_IO0
#define OPAMP4_InvertingInput_PD8         LL_OPAMP_INPUT_INVERT_IO1 
#define OPAMP4_InvertingInput_PGA         LL_OPAMP_INPUT_INVERT_CONNECT_NO
#define OPAMP4_InvertingInput_FOLLOWER    LL_OPAMP_MODE_FOLLOWER

#define OPAMP1_NonInvertingInput_PA7      LL_OPAMP_INPUT_NONINVERT_IO1
#define OPAMP1_NonInvertingInput_PA5      LL_OPAMP_INPUT_NONINVERT_IO3
#define OPAMP1_NonInvertingInput_PA3      LL_OPAMP_INPUT_NONINVERT_IO2
#define OPAMP1_NonInvertingInput_PA1      LL_OPAMP_INPUT_NONINVERT_IO0
#define OPAMP2_NonInvertingInput_PD14     LL_OPAMP_INPUT_NONINVERT_IO1
#define OPAMP2_NonInvertingInput_PB14     LL_OPAMP_INPUT_NONINVERT_IO3
#define OPAMP2_NonInvertingInput_PB0      LL_OPAMP_INPUT_NONINVERT_IO2
#define OPAMP2_NonInvertingInput_PA7      LL_OPAMP_INPUT_NONINVERT_IO0
#define OPAMP3_NonInvertingInput_PB13     LL_OPAMP_INPUT_NONINVERT_IO1
#define OPAMP3_NonInvertingInput_PA5      LL_OPAMP_INPUT_NONINVERT_IO3
#define OPAMP3_NonInvertingInput_PA1      LL_OPAMP_INPUT_NONINVERT_IO2
#define OPAMP3_NonInvertingInput_PB0      LL_OPAMP_INPUT_NONINVERT_IO0
#define OPAMP4_NonInvertingInput_PD11     LL_OPAMP_INPUT_NONINVERT_IO1
#define OPAMP4_NonInvertingInput_PB11     LL_OPAMP_INPUT_NONINVERT_IO3
#define OPAMP4_NonInvertingInput_PA4      LL_OPAMP_INPUT_NONINVERT_IO2
#define OPAMP4_NonInvertingInput_PB13     LL_OPAMP_INPUT_NONINVERT_IO0

#define OPAMP1_PGAConnect_PC5             OPAMP_CSR_PGGAIN_3
#define OPAMP1_PGAConnect_PA3             (OPAMP_CSR_PGGAIN_3|OPAMP_CSR_PGGAIN_2)
#define OPAMP2_PGAConnect_PC5             OPAMP_CSR_PGGAIN_3
#define OPAMP2_PGAConnect_PA5             (OPAMP_CSR_PGGAIN_3|OPAMP_CSR_PGGAIN_2)
#define OPAMP3_PGAConnect_PB10            OPAMP_CSR_PGGAIN_3
#define OPAMP3_PGAConnect_PB2             (OPAMP_CSR_PGGAIN_3|OPAMP_CSR_PGGAIN_2)
#define OPAMP4_PGAConnect_PB10            OPAMP_CSR_PGGAIN_3
#define OPAMP4_PGAConnect_PD8             (OPAMP_CSR_PGGAIN_3|OPAMP_CSR_PGGAIN_2)

#define COMP1_InvertingInput_PA0          LL_COMP_INPUT_MINUS_IO1
#define COMP2_InvertingInput_PA2          LL_COMP_INPUT_MINUS_IO1
#define COMP3_InvertingInput_PD15         LL_COMP_INPUT_MINUS_IO1
#define COMP3_InvertingInput_PB12         LL_COMP_INPUT_MINUS_IO2
#define COMP4_InvertingInput_PE8          LL_COMP_INPUT_MINUS_IO1
#define COMP4_InvertingInput_PB2          LL_COMP_INPUT_MINUS_IO2
#define COMP5_InvertingInput_PD13         LL_COMP_INPUT_MINUS_IO1
#define COMP5_InvertingInput_PB10         LL_COMP_INPUT_MINUS_IO2
#define COMP6_InvertingInput_PD10         LL_COMP_INPUT_MINUS_IO1
#define COMP6_InvertingInput_PB15         LL_COMP_INPUT_MINUS_IO2
#define COMP7_InvertingInput_PC0          LL_COMP_INPUT_MINUS_IO1

#define COMPX_InvertingInput_DAC1        LL_COMP_INPUT_MINUS_DAC1_CH1
#define COMPX_InvertingInput_DAC2        LL_COMP_INPUT_MINUS_DAC1_CH2
#define COMPX_InvertingInput_VREF        LL_COMP_INPUT_MINUS_VREFINT
#define COMPX_InvertingInput_VREF_1_4    LL_COMP_INPUT_MINUS_1_4VREFINT
#define COMPX_InvertingInput_VREF_1_2    LL_COMP_INPUT_MINUS_1_2VREFINT
#define COMPX_InvertingInput_VREF_3_4    LL_COMP_INPUT_MINUS_3_4VREFINT

#define COMP1_NonInvertingInput_PA1    LL_COMP_INPUT_PLUS_IO1
#define COMP2_NonInvertingInput_PA3    LL_COMP_INPUT_PLUS_IO2
#define COMP2_NonInvertingInput_PA7    LL_COMP_INPUT_PLUS_IO1
#define COMP3_NonInvertingInput_PB14   LL_COMP_INPUT_PLUS_IO1
#define COMP3_NonInvertingInput_PD14   LL_COMP_INPUT_PLUS_IO2
#define COMP4_NonInvertingInput_PB0    LL_COMP_INPUT_PLUS_IO1
#define COMP4_NonInvertingInput_PE7    LL_COMP_INPUT_PLUS_IO2
#define COMP5_NonInvertingInput_PB13   LL_COMP_INPUT_PLUS_IO2
#define COMP5_NonInvertingInput_PD12   LL_COMP_INPUT_PLUS_IO1
#define COMP6_NonInvertingInput_PB11   LL_COMP_INPUT_PLUS_IO2
#define COMP6_NonInvertingInput_PD11   LL_COMP_INPUT_PLUS_IO1
#define COMP7_NonInvertingInput_PC1    LL_COMP_INPUT_PLUS_IO2
#define COMP7_NonInvertingInput_PA0    LL_COMP_INPUT_PLUS_IO1

#define R3_4_OPAMPPARAMSM1 MC_NULL
#define R3_2_OPAMPPARAMSM1 MC_NULL

#define ADC1_2_DUALDRIVEFIFOUPDATE
#define ADC3_DUALDRIVEFIFOUPDATE

/*******************************************************************************
The following block shall be removed when future version of WB will generate the
proper AF defines according selected pins 
*******************************************************************************/
#undef GPIOA
#undef GPIOB
#undef GPIOC
#undef GPIOD
#undef GPIOE
#undef GPIOF

#define GPIOA 100
#define GPIOB 101
#define GPIOC 102
#define GPIOD 103
#define GPIOE 104
#define GPIOF 105

#undef GPIO_Pin_0
#undef GPIO_Pin_1
#undef GPIO_Pin_2
#undef GPIO_Pin_3
#undef GPIO_Pin_4
#undef GPIO_Pin_5
#undef GPIO_Pin_6
#undef GPIO_Pin_7
#undef GPIO_Pin_8
#undef GPIO_Pin_9
#undef GPIO_Pin_10
#undef GPIO_Pin_11
#undef GPIO_Pin_12
#undef GPIO_Pin_13
#undef GPIO_Pin_14
#undef GPIO_Pin_15

#define GPIO_Pin_0  200
#define GPIO_Pin_1  201
#define GPIO_Pin_2  202
#define GPIO_Pin_3  203
#define GPIO_Pin_4  204
#define GPIO_Pin_5  205
#define GPIO_Pin_6  206
#define GPIO_Pin_7  207
#define GPIO_Pin_8  208
#define GPIO_Pin_9  209
#define GPIO_Pin_10 210
#define GPIO_Pin_11 211
#define GPIO_Pin_12 212
#define GPIO_Pin_13 213
#define GPIO_Pin_14 214
#define GPIO_Pin_15 215

/* TIM 2 - CH1*/

/* TIM 3 - CH1*/

/* TIM 4 - CH1*/

/* TIM 2 - CH2*/

/* TIM 3 - CH2*/

/* TIM 4 - CH2*/

/* TIM 2 - CH3*/

/* TIM 3 - CH3*/

/* TIM 4 - CH3*/

#undef GPIOA
#undef GPIOB
#undef GPIOC
#undef GPIOD
#undef GPIOE
#undef GPIOF
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)

#undef GPIO_Pin_0
#undef GPIO_Pin_1
#undef GPIO_Pin_2
#undef GPIO_Pin_3
#undef GPIO_Pin_4
#undef GPIO_Pin_5
#undef GPIO_Pin_6
#undef GPIO_Pin_7
#undef GPIO_Pin_8
#undef GPIO_Pin_9
#undef GPIO_Pin_10
#undef GPIO_Pin_11
#undef GPIO_Pin_12
#undef GPIO_Pin_13
#undef GPIO_Pin_14
#undef GPIO_Pin_15
#define GPIO_Pin_0                 LL_GPIO_PIN_0  /*!< Pin 0 selected    */
#define GPIO_Pin_1                 LL_GPIO_PIN_1  /*!< Pin 1 selected    */
#define GPIO_Pin_2                 LL_GPIO_PIN_2  /*!< Pin 2 selected    */
#define GPIO_Pin_3                 LL_GPIO_PIN_3  /*!< Pin 3 selected    */
#define GPIO_Pin_4                 LL_GPIO_PIN_4  /*!< Pin 4 selected    */
#define GPIO_Pin_5                 LL_GPIO_PIN_5  /*!< Pin 5 selected    */
#define GPIO_Pin_6                 LL_GPIO_PIN_6  /*!< Pin 6 selected    */
#define GPIO_Pin_7                 LL_GPIO_PIN_7  /*!< Pin 7 selected    */
#define GPIO_Pin_8                 LL_GPIO_PIN_8  /*!< Pin 8 selected    */
#define GPIO_Pin_9                 LL_GPIO_PIN_9  /*!< Pin 9 selected    */
#define GPIO_Pin_10                LL_GPIO_PIN_10  /*!< Pin 10 selected   */
#define GPIO_Pin_11                LL_GPIO_PIN_11 /*!< Pin 11 selected   */
#define GPIO_Pin_12                LL_GPIO_PIN_12  /*!< Pin 12 selected   */
#define GPIO_Pin_13                LL_GPIO_PIN_13  /*!< Pin 13 selected   */
#define GPIO_Pin_14                LL_GPIO_PIN_14  /*!< Pin 14 selected   */
#define GPIO_Pin_15                LL_GPIO_PIN_15  /*!< Pin 15 selected   */
/*******************************************************************************
Block ends here 
*******************************************************************************/

#endif /*__PARAMETERS_CONVERSION_F30X_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
