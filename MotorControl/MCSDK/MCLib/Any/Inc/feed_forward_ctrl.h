/**
  ******************************************************************************
  * @file    feed_forward_ctrl.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Feed Forward Control component of the Motor Control SDK.
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
#ifndef __FEEDFORWARDCTRL_H
#define __FEEDFORWARDCTRL_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "bus_voltage_sensor.h"
#include "speed_pos_fdbk.h"
#include "speed_torq_ctrl.h"
#include "sto_speed_pos_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup FeedForwardCtrl
  * @{
  */

/** 
  * @brief Handle structure of the Feed Forward Component
  */
typedef struct
{
  Volt_Components   Vqdff;                 /**< Feed Forward controller @f$I_{qd}@f$ contribution to @f$V_{qd}@f$ */
  Volt_Components   VqdPIout;              /**< @f$V_{qd}@f$ as output by PID controller */
  Volt_Components   VqdAvPIout;            /**< Averaged @f$V_{qd}@f$ as output by PID controller */
  int32_t  wConstant_1D;                   /**< Feed forward default constant for the @f$d@f$ axis */
  int32_t  wConstant_1Q;                   /**< Feed forward default constant for the @f$q@f$ axis */
  int32_t  wConstant_2;                    /**< Default constant value used by Feed-Forward algorithm */
  BusVoltageSensor_Handle_t *pBus_Sensor;  /**< Related bus voltage sensor */
  PID_Handle_t  *pPID_q;                   /*!< Related PI for @f$I_{q}@f$ regulator */
  PID_Handle_t  *pPID_d;                   /*!< Related PI for @f$I_{d}@f$ regulator */
  uint16_t hVqdLowPassFilterBW;            /**< Configures the @f$V_{qd}@f$
                                                first order software filter bandwidth.

                                                If #FULL_MISRA_COMPLIANCY is defined, this field must be
                                                set to
                                                @f[
                                                \frac{FOC_CurrController
                                                call rate [Hz]}{FilterBandwidth[Hz]}
                                                @f]
                                                Otherwise, it must be set to:
                                                @f[
                                                log_2(\frac{FOC_CurrController
                                                call rate [Hz]}{FilterBandwidth[Hz]})
                                                @f]
                                                */
  int32_t  wDefConstant_1D;                /**< Feed forward default constant for d axes */
  int32_t  wDefConstant_1Q;                /**< Feed forward default constant for q axes */
  int32_t  wDefConstant_2;                 /**< Default constant value used by
                                                Feed-Forward algorithm*/
  uint16_t hVqdLowPassFilterBWLOG;         /**< hVqdLowPassFilterBW expressed as power of 2.
                                                E.g. if gain divisor is 512 the value
                                                must be 9 because 2^9 = 512 */
									 
} FF_Handle_t;  


  


/**
  * @brief  Initializes all the component variables
  * @param  pHandle Feed forward init strutcture.
  * @param  pBusSensor VBus Sensor.
  * @param  pPIDId Id PID.
  * @param  pPIDIq Iq PID.                                
  * @retval none
  */
void FF_Init(FF_Handle_t *pHandle,BusVoltageSensor_Handle_t *pBusSensor, PID_Handle_t *pPIDId, PID_Handle_t *pPIDIq );

/**
  * @brief  It should be called before each motor restart and clears the Flux 
  *         weakening internal variables.
  * @param  pHandle Feed forward init strutcture.
  * @retval none
  */
void FF_Clear(FF_Handle_t *pHandle);

/**
  * @brief  It implements feed-forward controller by computing new Vqdff value. 
  *         This will be then summed up to PI output in IMFF_VqdConditioning 
  *         method.
  * @param  pHandle Feed forward  strutcture.
  * @param  Iqdref Idq reference componets used to calcupate the feed forward
  *         action.
  * @param  pSTC  Speed sensor.
  * @retval none
  */
void FF_VqdffComputation(FF_Handle_t *pHandle, Curr_Components Iqdref,SpeednTorqCtrl_Handle_t *pSTC);

/**
  * @brief  It return the Vqd componets fed in input plus the feed forward 
  *         action and store the last Vqd values in the internal variable.
  * @param  pHandle Feed forward  strutcture.
  * @param  Vqd Initial value of Vqd to be manipulated by FF.
  * @retval none
  */
Volt_Components FF_VqdConditioning(FF_Handle_t *pHandle, Volt_Components Vqd);

/**
  * @brief  It low-pass filters the Vqd voltage coming from the speed PI. Filter 
  *         bandwidth depends on hVqdLowPassFilterBW parameter.
  * @param  pHandle Feed forward  strutcture.
  * @retval none
  */
void FF_DataProcess(FF_Handle_t *pHandle);

/**
  * @brief  Use this method to initialize FF vars in START_TO_RUN state.
  * @param  pHandle Feed forward  strutcture.
  * @retval none
  */
void FF_InitFOCAdditionalMethods(FF_Handle_t *pHandle);

/**
  * @brief  Use this method to set new values for the constants utilized by 
  *         feed-forward algorithm.
  * @param  pHandle Feed forward  strutcture.
  * @param  sNewConstants The FF_TuningStruct_t containing constants utilized by 
  *         feed-forward algorithm.
  * @retval none
  */
void FF_SetFFConstants(FF_Handle_t *pHandle, FF_TuningStruct_t sNewConstants);

/**
  * @brief  Use this method to get present values for the constants utilized by 
  *         feed-forward algorithm.
  * @param  pHandle Feed forward  strutcture.
  * @retval FF_TuningStruct_t Values of the constants utilized by 
  *         feed-forward algorithm.
  */
FF_TuningStruct_t FF_GetFFConstants(FF_Handle_t *pHandle);

/**
  * @brief  Use this method to get present values for the Vqd feed-forward 
  *         components.
  * @param  pHandle Feed forward  strutcture.
  * @retval Volt_Components Vqd feed-forward components.
  */
Volt_Components FF_GetVqdff(FF_Handle_t *pHandle);

/**
  * @brief  Use this method to get values of the averaged output of qd axes 
  *         currents PI regulators.
  * @param  pHandle Feed forward  strutcture.
  * @retval Volt_Components Averaged output of qd axes currents PI regulators.
  */
Volt_Components FF_GetVqdAvPIout(FF_Handle_t *pHandle);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __FEEDFORWARDCTRL_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
