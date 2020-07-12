/**
  ******************************************************************************
  * @file    enc_align_ctrl.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Encoder Alignment Control component of the Motor Control SDK.
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
#ifndef __ENCALIGNCTRLCLASS_H
#define __ENCALIGNCTRLCLASS_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
   
/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "speed_torq_ctrl.h"
#include "virtual_speed_sensor.h"
#include "encoder_speed_pos_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup EncAlignCtrl
  * @{
  */

/** 
  * @brief  This structure is used to handle an instance of  EncAlignCtrl component
  */
   
typedef struct
{
  SpeednTorqCtrl_Handle_t *pSTC;        /*!< Speed and torque controller object used by  EAC.*/
  VirtualSpeedSensor_Handle_t *pVSS;    /*!< Virtual speed sensor object used by EAC.*/
  ENCODER_Handle_t *pENC;               /*!< Encoder object used by EAC.*/
  uint16_t hRemainingTicks;             /*!< Number of clock events remaining to complete 
                                             the alignment.*/
  bool EncAligned;   /*!< This flag is true if the encoder has been 
                               aligned at least one time, false if hasn't been
                                never aligned.*/
  bool EncRestart;   /*!< This flag is used to force a restart of the 
                                motorafter the encoder alignment. It is true
                                if a restart is programmed else false*/
  uint16_t hEACFrequencyHz; /*!< Frequency expressed in Hz at which the user 
                                 clocks the EAC calling EAC_Exec method */
  int16_t hFinalTorque;     /*!< Motor torque reference imposed by STC at the 
                                 end of programmed alignment. This value 
                                 represents actually the Iq current expressed in
                                 digit.*/
  int16_t hElAngle;        /*!< Electrical angle of programmed alignment 
                                 expressed in s16degrees.*/
  uint16_t hDurationms;     /*!< Duration of the programmed alignment expressed
                                 in milliseconds.*/
  uint8_t bElToMecRatio;    /*!< Coefficient used to transform electrical to
                                 mechanical quantities and viceversa. It usually
                                 coincides with motor pole pairs number*/
} EncAlign_Handle_t;
  

/* Exported functions ------------------------------------------------------- */

/*  Function used to initialize an instance of the EncAlignCtrl component */
void EAC_Init(EncAlign_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pSTC, VirtualSpeedSensor_Handle_t *pVSS, ENCODER_Handle_t *pENC );

/* Function used to start the encoder alignment procedure.*/
void EAC_StartAlignment(EncAlign_Handle_t *pHandle);

/* Function used to clocks the encoder alignment controller*/
bool EAC_Exec(EncAlign_Handle_t *pHandle);

/* It returns true if the encoder has been aligned at least one time*/
bool EAC_IsAligned(EncAlign_Handle_t *pHandle);

/* It sets a restart after an encoder alignment*/
void EAC_SetRestartState(EncAlign_Handle_t *pHandle, bool restart);

/* Returns true if a restart after an encoder alignment has been requested*/
bool EAC_GetRestartState(EncAlign_Handle_t *pHandle);

/**
  * @}
  */
  
/**
  * @}
  */
#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __ENCALIGNCTRLCLASS_H */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
