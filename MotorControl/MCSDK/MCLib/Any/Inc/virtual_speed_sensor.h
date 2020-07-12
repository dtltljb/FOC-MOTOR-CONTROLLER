/**
  ******************************************************************************
  * @file    virtual_speed_sensor.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Virtual Speed Sensor component of the Motor Control SDK.
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
#ifndef _VIRTUALSPEEDSENSOR_H
#define _VIRTUALSPEEDSENSOR_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"
#ifdef STM32F0XX
#include "fast_div.h"
#endif
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @addtogroup VirtualSpeedSensor
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  This structure is used to handle an instance of the Virtual Speed
  *         sensor component
  */
typedef struct {
  SpeednPosFdbk_Handle_t   _Super;
  int32_t wElAccDppP32;   /*!< Delta electrical speed expressed in dpp per speed 
                               sampling period to be appied each time is called 
                               SPD_CalcAvrgMecSpeed01Hz multiplied by scaling
                               factor of 65536.*/
  int32_t wElSpeedDpp32;  /*!< Electrical speed expressed in dpp multiplied by
                               scaling factor 65536.*/
  uint16_t hRemainingStep;/*!< Number of steps remaining to reach the final
                               speed.*/
  int16_t hFinalMecSpeed01Hz;/*!< Backup of hFinalMecSpeed01Hz to be applied in 
                               the last step.*/
  bool bTransitionStarted;    /*!< Retaining information about Transition status.*/
  bool bTransitionEnded;      /*!< Retaining information about ransition status.*/
  int16_t hTransitionRemainingSteps;  /*!< Number of steps remaining to end
                               transition from CVSS_SPD to other CSPD*/
  int16_t hElAngleAccu;        /*!< Electrical angle accumulator*/
  bool bTransitionLocked;      /*!< Transition acceleration started*/
  bool bCopyObserver;          /*!< Command to set VSPD output same as state observer*/
  
  uint16_t hSpeedSamplingFreqHz; /*!< Frequency (Hz) at which motor speed is to 
                             be computed. It must be equal to the frequency
                             at which function SPD_CalcAvrgMecSpeed01Hz
                             is called.*/
  int16_t hTransitionSteps; /*< Number of steps to perform the transition phase
                             from CVSS_SPD to other CSPD; if the Transition PHase
                             should last TPH milliseconds, and the FOC Execution
                             Frequency is set to FEF kHz, then
                             hTransitionSteps = TPH * FEF*/
#ifdef STM32F0XX
  /* (Fast division optimization for cortex-M0 micros)*/  
  FastDiv_Handle_t fd;                       /*!< Fast division obj.*/
#endif
  
} VirtualSpeedSensor_Handle_t;

/* It initializes the Virtual Speed Sensor component */
void VSS_Init(VirtualSpeedSensor_Handle_t *pHandle);

/* It clears Virtual Speed Sensor by re-initializing private variables*/
void VSS_Clear(VirtualSpeedSensor_Handle_t *pHandle);

/* It compute a theorical speed and update the theorical electrical angle. */
int16_t VSS_CalcElAngle(VirtualSpeedSensor_Handle_t *pHandle, void *pInputVars_str);

/*It computes through pMecSpeed01Hz the rotor theorical average mechanical speed in 01Hz*/
bool VSS_CalcAvrgMecSpeed01Hz(VirtualSpeedSensor_Handle_t *pHandle, int16_t *hMecSpeed01Hz);

/* It set istantaneous information on VSS mechanical and  electrical angle.*/
void VSS_SetMecAngle(VirtualSpeedSensor_Handle_t *pHandle, int16_t hMecAngle);

/* Set the mechanical acceleration of virtual sensor. */
void  VSS_SetMecAcceleration(VirtualSpeedSensor_Handle_t *pHandle, int16_t  hFinalMecSpeed01Hz, 
                              uint16_t hDurationms);
/* Checks if the ramp executed after a VSPD_SetMecAcceleration command has been completed*/
bool VSS_RampCompleted(VirtualSpeedSensor_Handle_t *pHandle);

/* Get the final speed of last setled ramp of virtual sensor expressed in 0.1Hz*/
int16_t  VSS_GetLastRampFinalSpeed(VirtualSpeedSensor_Handle_t *pHandle);

/* Set the command to Start the transition phase from VirtualSpeedSensor to other SpeedSensor.*/
bool VSS_SetStartTransition(VirtualSpeedSensor_Handle_t *pHandle, bool bCommand);

/* Return the status of the transition phase.*/
bool VSS_IsTransitionOngoing(VirtualSpeedSensor_Handle_t *pHandle);

/* It set istantaneous information on rotor electrical angle copied by state observer */
void VSS_SetCopyObserver(VirtualSpeedSensor_Handle_t *pHandle);

/* It  set istantaneous information on rotor electrical angle */
void VSS_SetElAngle(VirtualSpeedSensor_Handle_t *pHandle, int16_t hElAngle);

/** @} */
/** @} */
/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* _VIRTUALSPEEDSENSOR_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
