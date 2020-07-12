/**
  ******************************************************************************
  * @file    revup_ctrl.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          RevUpCtrl component of the Motor Control SDK.
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
#ifndef __REVUP_CTRL_H
#define __REVUP_CTRL_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "speed_torq_ctrl.h"
#include "virtual_speed_sensor.h"
#include "sto_speed_pos_fdbk.h"
#include "pwm_curr_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup RevUpCtrl
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/**
  * @brief Maximum number of phases allowed for RevUp process.
  *
  */
#define RUC_MAX_PHASE_NUMBER 5u

/**
  * @brief RevUpCtrl_PhaseParams_t structure used for phases definition
  *
  */
typedef struct
{
    uint16_t hDurationms;         /**< Duration of the RevUp phase.
                                       This parameter is expressed in millisecond.*/
    int16_t hFinalMecSpeed01Hz;   /**< Mechanical speed expressed in 0.1Hz assumed
                                       by VSS at the end of the RevUp phase.*/
    int16_t hFinalTorque;         /**< Motor torque reference imposed by STC at the
                                       end of RevUp phase. This value represents
                                       actually the Iq current expressed in digit.*/
    void* pNext;                  /**< Pointer on the next phase section to proceed
                                       This parameter is NULL for the last element.*/
} RevUpCtrl_PhaseParams_t;

  /**
  * @brief  Handle structure of the RevUpCtrl.
  *
  */

typedef struct
{
  uint16_t hRUCFrequencyHz;        /**< Frequency call to main RevUp procedure RUC_Exec. 
                                        This parameter is equal to speed loop frequency. */

  int16_t hStartingMecAngle;       /**< Starting angle of programmed RevUp.*/

  uint16_t hPhaseRemainingTicks;   /**< Number of clock events remaining to complete the phase. */
  
  int16_t hDirection;              /**< Motor direction.
                                        This parameter can be any value -1 or +1 */
  
  RevUpCtrl_PhaseParams_t *pCurrentPhaseParams; /**< Pointer on the current RevUp phase processed. */ 

  RevUpCtrl_PhaseParams_t ParamsData[RUC_MAX_PHASE_NUMBER]; /**< Start up Phases sequences used by RevUp controller.
                                                                Up to five phases can be used for the start up.             */

  uint8_t bPhaseNbr;               /**< Number of phases relative to the programmed RevUp sequence.
                                        This parameter can be any value from 1 to 5 */
  
  uint8_t bFirstAccelerationStage; /**< Indicate the phase to start the final acceleration.
                                        At start of this stage sensor-less algorithm cleared.*/
  uint16_t hMinStartUpValidSpeed;  /**< Minimum rotor speed required to validate the startup.
                                        This parameter is expressed in 01Hz */
  uint16_t hMinStartUpFlySpeed;    /**< Minimum rotor speed required to validate the on the fly.
                                        This parameter is expressed in 01Hz */
  int16_t hOTFFinalRevUpCurrent;   /**< Final targetted torque for OTF phase. */

  uint16_t hOTFSection1Duration;   /**< On-the-fly phase duration, millisecond.
                                        This parameter is expressed in millisecond.*/
  bool OTFStartupEnabled;          /**< Flag for OTF feature activation.
                                        Feature disabled when set to false */
  uint8_t bOTFRelCounter;          /**< Counts the number of reliability of state observer */
  
  bool OTFSCLowside;               /**< Flag to indicate status of low side switchs.
                                        This parameter can be true when Low Sides switch is ON otherwise set to false. */
  bool EnteredZone1;               /**< Flag to indicate that the minimum rotor speed has been reached. */
  
  uint8_t bResetPLLTh;            /**< Threshold to reset PLL during OTF */
  
  uint8_t bResetPLLCnt;           /**< Counter to reset PLL during OTF when the threshold is reached. */

  uint8_t bStageCnt;              /**< Counter of executed phases.
                                      This parameter can be any value from 0 to 5 */

  RevUpCtrl_PhaseParams_t OTFPhaseParams; /**< RevUp phase parameter of OTF feature.*/
  
  SpeednTorqCtrl_Handle_t * pSTC;                      /**< Speed and torque controller object used by RevUpCtrl.*/
  
  VirtualSpeedSensor_Handle_t * pVSS;                  /**< Virtual speed sensor object used by RevUpCtrl.*/

  STO_Handle_t * pSNSL;                 /**< STO sensor object used by OTF startup.*/

  PWMC_Handle_t* pPWM;                     /**< PWM object used by OTF startup.*/

} RevUpCtrl_Handle_t;


/* Exported functions ------------------------------------------------------- */

/*  Function used to initialize and configure the RevUpCtrl Component */
void RUC_Init(RevUpCtrl_Handle_t *pHandle,
              SpeednTorqCtrl_Handle_t * pSTC,
              VirtualSpeedSensor_Handle_t * pVSS,
              STO_Handle_t * pSNSL,
              PWMC_Handle_t* oPWM);

/*  Function used to reset internal state of the RevUpCtrl Component to its default state */
void RUC_Clear(RevUpCtrl_Handle_t *pHandle, int16_t hMotorDirection);

/* Main procedure which clock the RevUp controller. */
bool RUC_Exec(RevUpCtrl_Handle_t *pHandle);

/* Main procedure which clock the RevUp controller with on-the-fly feature. */
bool RUC_OTF_Exec(RevUpCtrl_Handle_t *pHandle);

/* Return information about current state of programmer RevUp sequence. */
bool RUC_Completed(RevUpCtrl_Handle_t *pHandle);

/* Function allow to stop the programmed RevUp at the current speed. */
void RUC_Stop(RevUpCtrl_Handle_t *pHandle);

/* Check that final acceleration during RevUp phase is reached  */
bool RUC_FirstAccelerationStageReached(RevUpCtrl_Handle_t *pHandle);

/* Function used to set the duration of a specific phase. */
void RUC_SetPhaseDurationms(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, uint16_t hDurationms);

/* Function used to set the targetted motor speed at the end of a specific phase. */
void RUC_SetPhaseFinalMecSpeed01Hz(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase,
                                   int16_t hFinalMecSpeed01Hz);

/* Function used to set the final motor torque targetted at the end of a specific phase. */
void RUC_SetPhaseFinalTorque(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, int16_t hFinalTorque);

/* Function used to read the duration of a specific RevUp phase */
uint16_t RUC_GetPhaseDurationms(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase);

/* Function used to read the targetted mechanical speed of a specific RevUp phase */
int16_t RUC_GetPhaseFinalMecSpeed01Hz(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase);

/* Function used to read the targetted torque of a specific RevUp phase */
int16_t RUC_GetPhaseFinalTorque(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase);

/* Function used to read the number of phase(s) used by RevUp procedure  */
uint8_t RUC_GetNumberOfPhases(RevUpCtrl_Handle_t *pHandle);

/* Return the state of low side switches. */
bool RUC_Get_SCLowsideOTF_Status(RevUpCtrl_Handle_t *pHandle);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __REVUP_CTRL_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
