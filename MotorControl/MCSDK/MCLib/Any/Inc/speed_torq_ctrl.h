/**
  ******************************************************************************
  * @file    speed_torq_ctrl.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Speed & Torque Control component of the Motor Control SDK.
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
#ifndef __SPEEDNTORQCTRLCLASS_H
#define __SPEEDNTORQCTRLCLASS_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pid_regulator.h"
#include "speed_pos_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednTorqCtrl
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  Speed & Torque Control parameters definition
  */
typedef struct
{
  STC_Modality_t Mode;   /*!< Modality of STC. It can be one of these two
                               settings: STC_TORQUE_MODE to enable the
                               Torque mode or STC_SPEED_MODE to enable the
                               Speed mode.*/
  int16_t TargetFinal;	/*!< Backup of hTargetFinal to be applied in the
                             last step.*/ 
  int32_t SpeedRef01HzExt; /*!< Current mechanical rotor speed reference 
                                     expressed in tenths of HZ multiplied by 
                                     65536.*/
  int32_t TorqueRef;     /*!< Current motor torque reference. This value
                                     represents actually the Iq current 
                                     expressed in digit multiplied by 65536.*/
  uint32_t RampRemainingStep;/*!< Number of steps remaining to complete the
                                     ramp.*/
  PID_Handle_t * PISpeed;		/*!< The regulator used to perform the speed
                                     control loop.*/
  SpeednPosFdbk_Handle_t * SPD;/*!< The speed sensor used to perform the speed 
                                     regulation.*/
  int32_t IncDecAmount;	/*!< Increment/decrement amount to be applied to
                                     the reference value at each
                                     CalcTorqueReference.*/
        
  uint16_t STCFrequencyHz;             /*!< Frequency on which the user updates 
                                             the torque reference calling 
                                             STC_CalcTorqueReference method 
                                             expressed in Hz */
  uint16_t MaxAppPositiveMecSpeed01Hz; /*!< Application maximum positive value
                                             of rotor speed. It's expressed in 
                                             tenth of mechanical Hertz.*/
  uint16_t MinAppPositiveMecSpeed01Hz; /*!< Application minimum positive value
                                             of rotor speed. It's expressed in
                                             tenth of mechanical Hertz.*/
  int16_t MaxAppNegativeMecSpeed01Hz;  /*!< Application maximum negative value
                                             of rotor speed. It's expressed in 
                                             tenth of mechanical Hertz.*/
  int16_t MinAppNegativeMecSpeed01Hz;  /*!< Application minimum negative value
                                             of rotor speed. It's expressed in
                                             tenth of mechanical Hertz.*/
  uint16_t MaxPositiveTorque;          /*!< Maximum positive value of motor 
                                             torque. This value represents 
                                             actually the maximum Iq current 
                                             expressed in digit.*/
  int16_t MinNegativeTorque;           /*!< Minimum negative value of motor 
                                             torque. This value represents 
                                             actually the maximum Iq current 
                                             expressed in digit.*/
  STC_Modality_t ModeDefault;          /*!< Default STC modality.*/
  int16_t MecSpeedRef01HzDefault;      /*!< Default mechanical rotor speed 
                                             reference expressed in tenths of 
                                             HZ.*/
  int16_t TorqueRefDefault;            /*!< Default motor torque reference. 
                                             This value represents actually the
                                             Iq current reference expressed in
                                             digit.*/
  int16_t IdrefDefault;                /*!< Default Id current reference expressed
                                             in digit.*/
}SpeednTorqCtrl_Handle_t;
  


/* It initializes all the object variables */
void STC_Init(SpeednTorqCtrl_Handle_t *pHandle, PID_Handle_t * oPI, SpeednPosFdbk_Handle_t * oSPD);

/* It resets the integral term of speed regulator */
void STC_Clear(SpeednTorqCtrl_Handle_t *pHandle);

/* Get the current mechanical rotor speed reference expressed in tenths of HZ.*/ 
int16_t STC_GetMecSpeedRef01Hz(SpeednTorqCtrl_Handle_t *pHandle);

/*  Get the current motor torque reference. */
int16_t STC_GetTorqueRef(SpeednTorqCtrl_Handle_t *pHandle);

/* Set the mode of the speed and torque controller (Torque mode or Speed mode)*/
void STC_SetControlMode(SpeednTorqCtrl_Handle_t *pHandle, STC_Modality_t bMode);

/* Get the mode of the speed and torque controller. */
STC_Modality_t STC_GetControlMode(SpeednTorqCtrl_Handle_t *pHandle);

/* Starts the execution of a ramp using new target and duration. */
bool STC_ExecRamp(SpeednTorqCtrl_Handle_t *pHandle, int16_t hTargetFinal, uint32_t hDurationms);

/* It interrupts the execution of any previous ramp command.*/
void STC_StopRamp(SpeednTorqCtrl_Handle_t *pHandle);

/* It computes the new value of motor torque reference */
int16_t STC_CalcTorqueReference(SpeednTorqCtrl_Handle_t *pHandle);

/* Get the Default mechanical rotor speed reference expressed in tenths of HZ.*/
int16_t STC_GetMecSpeedRef01HzDefault(SpeednTorqCtrl_Handle_t *pHandle);

/* Get the Application maximum positive rotor speed in tenth of mech. Hertz.*/
uint16_t STC_GetMaxAppPositiveMecSpeed01Hz(SpeednTorqCtrl_Handle_t *pHandle);

/* Get the Application minimum negative rotor speed in tenth of mech. Hertz.*/
int16_t STC_GetMinAppNegativeMecSpeed01Hz(SpeednTorqCtrl_Handle_t *pHandle);

/* Check if the settled speed or torque ramp has been completed.*/
bool STC_RampCompleted(SpeednTorqCtrl_Handle_t *pHandle);

/* Stop the execution of speed ramp. */
bool STC_StopSpeedRamp(SpeednTorqCtrl_Handle_t *pHandle);

/* It sets in real time the speed sensor utilized by the FOC. */
void STC_SetSpeedSensor(SpeednTorqCtrl_Handle_t *pHandle, SpeednPosFdbk_Handle_t *oSPD);

/* It returns the speed sensor utilized by the FOC. */
SpeednPosFdbk_Handle_t* STC_GetSpeedSensor(SpeednTorqCtrl_Handle_t *pHandle);

/* It returns the default values of Iqdref. */
Curr_Components STC_GetDefaultIqdref(SpeednTorqCtrl_Handle_t *pHandle);

/* It sets the nominal current */
void STC_SetNominalCurrent(SpeednTorqCtrl_Handle_t *pHandle, uint16_t hNominalCurrent);

/* Force the speed reference to the current speed */
void STC_ForceSpeedReferenceToCurrentSpeed(SpeednTorqCtrl_Handle_t *pHandle);

/**
  * @}
  */
  
/**
  * @}
  */
#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __SPEEDNTORQCTRLCLASS_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

