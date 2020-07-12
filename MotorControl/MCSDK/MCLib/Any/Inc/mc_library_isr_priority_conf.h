/**
  ******************************************************************************
  * @file    mc_library_isr_priority_conf.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the priority configuration for each interrupt 
  *          service routine used by the MC library
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
#ifndef __MCLIBRARYISRPRIORITYCONF_H
#define __MCLIBRARYISRPRIORITYCONF_H

/* Includes ------------------------------------------------------------------*/

/** @addtogroup MCSDK
  * @{
  */
  
/**
 * @defgroup MCLibraryISRPriorityConf_definitions MC ISR Priorities
 *
 * @brief Definitions of the priorities to assign to the Interrupts used by the Motor Control subsystem.
 * @{
 */

/** @brief Current sensor TIMx update ISR priority */
#define TIMx_UP_PRE_EMPTION_PRIORITY 0u
/** @brief Current sensor TIMx update ISR sub-priority */
#define TIMx_UP_SUB_PRIORITY 0u

/** @brief Current sensor DMAx TC ISR priority */ 
#define DMAx_TC_PRE_EMPTION_PRIORITY 0u
/** @brief Current sensor DMAx TC ISR sub-priority */
#define DMAx_TC_SUB_PRIORITY 0u

#if !defined(STM32F0XX)
/** @brief Current sensor ADC1_2 ISR priority */
#define ADC_PRE_EMPTION_PRIORITY 2u
/** @brief Current sensor ADC1_2 ISR sub-priority */
#define ADC_SUB_PRIORITY 0u

/** @brief Speed feedback sensor TIM ISR priority */
#define TIMx_PRE_EMPTION_PRIORITY 3u
/** @brief Speed feedback sensor TIM ISR sub-priority */
#define TIMx_SUB_PRIORITY 0u

/** @brief Serial communication USART ISR priority */
#define USART_PRE_EMPTION_PRIORITY 3u
/** @brief Serial communication USART ISR sub-priority */
#define USART_SUB_PRIORITY 1u

/** @brief Systick ISR priority */
#define SYSTICK_PRE_EMPTION_PRIORITY 4u
/** @brief Systick ISR sub-priority */
#define SYSTICK_SUB_PRIORITY 0u

/** @brief Current sensor TIMx BRK ISR priority */
#define TIMx_BRK_PRE_EMPTION_PRIORITY 4u
/** @brief Current sensor TIMx BRK ISR sub-priority */
#define TIMx_BRK_SUB_PRIORITY 1u

/** @brief PendSV ISR priority */
#define PENDSV_PRE_EMPTION_PRIORITY 5u
/** @brief PendSV ISR sub-priority */
#define PENDSV_SUB_PRIORITY 0u

/* Do not modify (NVIC_PriorityGroup_3 is assumed to be set) */
#define SYSTICK_PRIORITY (((SYSTICK_PRE_EMPTION_PRIORITY & 0x07) << 1) | (SYSTICK_SUB_PRIORITY & 0x01))
#define PENDSV_PRIORITY  (((PENDSV_PRE_EMPTION_PRIORITY & 0x07)  << 1) | (PENDSV_SUB_PRIORITY & 0x01))
#endif

#if defined(STM32F0XX)
/* Current sensor ADC1_2 ISR priority */
#define ADC_PRE_EMPTION_PRIORITY 1u
#define ADC_SUB_PRIORITY 0u

/* Speed feedback sensor TIM ISR priority */
#define TIMx_PRE_EMPTION_PRIORITY 2u
#define TIMx_SUB_PRIORITY 0u

/* Serial communication USART ISR priority */
#define USART_PRE_EMPTION_PRIORITY 3u
#define USART_SUB_PRIORITY 1u

/* Systick ISR priority */
#define SYSTICK_PRE_EMPTION_PRIORITY 2u
#define SYSTICK_SUB_PRIORITY 0u

/* PendSV ISR priority */
#define PENDSV_PRE_EMPTION_PRIORITY 3u
#define PENDSV_SUB_PRIORITY 0u

/* Do not modify (NVIC_PriorityGroup_3 is assumed to be set) */
#define SYSTICK_PRIORITY (SYSTICK_PRE_EMPTION_PRIORITY & 0x03)
#define PENDSV_PRIORITY  (PENDSV_PRE_EMPTION_PRIORITY  & 0x03)
#endif

/**
  * @}
  */

/**
  * @}
  */

#endif /* __MCLIBRARYISRPRIORITYCONF_H */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
