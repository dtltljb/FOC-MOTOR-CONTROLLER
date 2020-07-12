/**
  ******************************************************************************
  * @file    sto_speed_pos_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          State Observer + PLL Speed & Position Feedback component of the Motor
  *          Control SDK.
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
#ifndef __STO_SPEEDNPOSFDBK_H
#define __STO_SPEEDNPOSFDBK_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
   
/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"
#include "pid_regulator.h" 

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @addtogroup SpeednPosFdbk_STO
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief This structure is used to handle an instance of the STO_SpeednPosFdbk component
  *
  */

typedef struct
{
  SpeednPosFdbk_Handle_t _Super;
  
  int16_t  hC1;                 /*!< Variable containing state observer constant
                                     C1 to speed-up computations */
  int16_t  hC2;                /*!< Variable containing state observer constant
                                     C2, it can be computed as F1 * K1/ State 
                                     observer execution rate [Hz] being K1 one
                                     of the two observer gains   */
  int16_t  hC3;                 /*!< Variable containing state observer constant
                                     C3 */
  int16_t  hC4;                  /*!< State Observer constant C4, it can 
                                      be computed as K2 * max measurable 
                                      current (A) / (Max application speed 
                                      [rpm] * Motor B-emf constant 
                                      [Vllrms/krpm] * sqrt(2) * F2 * State 
                                      observer execution rate [Hz]) being 
                                       K2 one of the two observer gains  */
  int16_t  hC5;                 /*!< Variable containing state observer constant 
                                     C5 */
  int16_t  hC6;                 /*!< State observer constant C6, computed with a 
                                    specific procedure starting from the other 
                                    constants */ 
  int16_t  hF1;                 /*!< Variable containing state observer scaling
                                     factor F1 */
  int16_t  hF2;                 /*!< Variable containing state observer scaling factor F2*/
  int16_t  hF3;                  /*!< State observer scaling factor F3 */
  uint16_t F3POW2;              /*!< State observer scaling factor F3 expressed as power of 2.
                                     E.g. if gain divisor is 512 the value 
                                     must be 9 because 2^9 = 512 */
  PID_Handle_t PIRegulator;     /*!< PI regulator component handle, used for 
                                     PLL implementation */									 
  int32_t Ialfa_est;           /*!< Estimated Ialfa current in int32 format */
  int32_t Ibeta_est;           /*!< Estimated Ibeta current in int32 format */
  int32_t wBemf_alfa_est;       /*!< Estimated B-emf alfa in int32_t format */
  int32_t wBemf_beta_est;       /*!< Estimated B-emf beta in int32_t format */
  int16_t hBemf_alfa_est;       /*!< Estimated B-emf alfa in int16_t format */
  int16_t hBemf_beta_est;       /*!< Estimated B-emf beta in int16_t format */
  int16_t Speed_Buffer[64];    /*!< Estimated speed FIFO, it contains latest 
                                     bSpeedBufferSize speed measurements*/
  uint8_t Speed_Buffer_Index;  /*!< Position of latest speed estimation in 
                                     estimated speed FIFO */  
  bool IsSpeedReliable;        /*!< Latest private speed reliability information,
                                     updated by SPD_CalcAvrgMecSpeed01Hz, it is 
                                     true if the speed measurement variance is 
                                     lower then threshold corresponding to 
                                     hVariancePercentage */
  uint8_t ConsistencyCounter;  /*!< Counter of passed tests for start-up 
                                     validation */
  uint8_t ReliabilityCounter; /*!< Counter for reliability check */
  bool IsAlgorithmConverged;   /*!< Boolean variable containing observer 
                                     convergence information */
  bool IsBemfConsistent;       /*!< Sensor reliability information, updated by
                                     SPD_CalcAvrgMecSpeed01Hz, it is true if the
                                     observed back-emfs are consistent with
                                     expectation*/
  
  int32_t Obs_Bemf_Level;      /*!< Observed back-emf Level*/
  int32_t Est_Bemf_Level;      /*!< Estimated back-emf Level*/
  bool EnableDualCheck;        /*!< Consistency check enabler*/
  int32_t DppBufferSum;        /*!< summation of speed buffer elements [dpp]*/
  int16_t SpeedBufferOldestEl; /*!< Oldest element of the speed buffer*/

  uint8_t SpeedBufferSize01Hz;       /*!< Depth of FIFO used to average 
                                           estimated speed exported by 
                                           SPD_GetAvrgMecSpeed01Hz. It 
                                           must be an integer number between 1 
                                           and 64 */  
  uint8_t SpeedBufferSizedpp;       /*!< Depth of FIFO used for both averaging 
                                           estimated speed exported by 
                                           SPD_GetElSpeedDpp and state 
                                           observer equations. It must be an 
                                           integer number between 1 and
                                           bSpeedBufferSize01Hz */
   uint16_t VariancePercentage;        /*!< Parameter expressing the maximum 
                                           allowed variance of speed estimation 
                                           */ 
   uint8_t SpeedValidationBand_H;   /*!< It expresses how much estimated speed
                                           can exceed forced stator electrical 
                                           frequency during start-up without 
                                           being considered wrong. The 
                                           measurement unit is 1/16 of forced 
                                           speed */
   uint8_t SpeedValidationBand_L;   /*!< It expresses how much estimated speed
                                           can be below forced stator electrical 
                                           frequency during start-up without 
                                           being considered wrong. The 
                                           measurement unit is 1/16 of forced 
                                           speed */
   uint16_t MinStartUpValidSpeed;     /*!< Absolute vaule of minimum mechanical
                                            speed (expressed in 01Hz) required 
                                            to validate the start-up */
   uint8_t StartUpConsistThreshold;   /*!< Number of consecutive tests on speed 
                                           consistency to be passed before 
                                           validating the start-up */
   uint8_t Reliability_hysteresys;    /*!< Number of reliability failed 
                                           consecutive tests before a speed 
                                           check fault is returned to _Super.bSpeedErrorNumber 
                                           */
   uint8_t BemfConsistencyCheck;      /*!< Degree of consistency of the observed
                                           back-emfs, it must be an integer
                                           number ranging from 1 (very high 
                                           consistency) down to 64 (very poor
                                           consistency) */
   uint8_t BemfConsistencyGain;       /*!< Gain to be applied when checking
                                           back-emfs consistency; default value
                                           is 64 (neutral), max value 105
                                           (x1.64 amplification), min value 1
                                           (/64 attenuation) */
   uint16_t MaxAppPositiveMecSpeed01Hz; /*!< Maximum positive value
                                             of rotor speed. It's expressed in 
                                             tenth of mechanical Hertz. It can be
                                             x1.1 greater than max application
                                             speed*/
   uint16_t F1LOG;                    /*!< F1 gain divisor expressed as power of 2.
                                            E.g. if gain divisor is 512 the value 
                                            must be 9 because 2^9 = 512 */
   uint16_t F2LOG;                    /*!< F2 gain divisor expressed as power of 2.
                                            E.g. if gain divisor is 512 the value 
                                            must be 9 because 2^9 = 512 */
   uint16_t SpeedBufferSizedppLOG;    /*!< bSpeedBufferSizedpp expressed as power of 2.
                                            E.g. if gain divisor is 512 the value 
                                            must be 9 because 2^9 = 512 */
  bool ForceConvergency;       /*!< Variable to force observer convergence.*/
  bool ForceConvergency2;      /*!< Variable to force observer convergence.*/
} STO_Handle_t;


/* Exported functions ------------------------------------------------------- */

/* It initializes the state observer object */
void STO_Init(STO_Handle_t *pHandle);

/* It only returns, necessary to implement fictitious IRQ_Handler */
void STO_Return(STO_Handle_t *pHandle, uint8_t flag);

/* It clears state observer object by re-initializing private variables*/
void STO_Clear(STO_Handle_t *pHandle);

/* It executes Luenberger state observer and calls PLL to compute a new speed 
*  estimation and update the estimated electrical angle. 
*/
int16_t STO_CalcElAngle(STO_Handle_t *pHandle, Observer_Inputs_t *pInputVars_str);

/*It computes through pMecSpeed01Hz the rotor average mechanical speed in 01Hz*/
bool STO_CalcAvrgMecSpeed01Hz(STO_Handle_t *pHandle, int16_t *pMecSpeed01Hz);

/* It resets integral term of PLL*/
void STO_ResetPLL(STO_Handle_t *pHandle);

/* It returns the result of the last variance check*/
bool STO_IsVarianceTight(STO_Handle_t *pHandle);

/* It set internal ForceConvergency1 to true*/
void STO_ForceConvergency1(STO_Handle_t *pHandle);

/* It set internal ForceConvergency2 to true*/
void STO_ForceConvergency2(STO_Handle_t *pHandle);

/* It checks whether the state observer algorithm converged.*/
bool STO_IsObserverConverged(STO_Handle_t *pHandle, int16_t hForcedMecSpeed01Hz);

/* It computes the estimated average electrical speed ElSpeedDpp expressed in dpp */
void STO_CalcAvrgElSpeedDpp(STO_Handle_t *pHandle);

/* It exports estimated Bemf alpha-beta in Volt_Components format */
Volt_Components STO_GetEstimatedBemf(STO_Handle_t *pHandle);

/* It exports the stator current alpha-beta as estimated by state  observer */
Curr_Components STO_GetEstimatedCurrent(STO_Handle_t *pHandle);

/* It set new values for observer gains*/
void STO_SetObserverGains(STO_Handle_t *pHandle, int16_t hC1, int16_t hC2);

/* It exports current observer gains through parameters pC2 and pC4 */
void STO_GetObserverGains(STO_Handle_t *pHandle, int16_t *pC2, int16_t *pC4);

/* It exports current PLL gains through parameters pPgain and pIgain */
void STO_GetPLLGains(STO_Handle_t *pHandle, int16_t *pPgain, int16_t *pIgain);

/* It set new values for PLL gains */
void STO_SetPLLGains(STO_Handle_t *pHandle, int16_t hPgain, int16_t hIgain);

void STO_SetPLL(STO_Handle_t *pHandle, int16_t hElSpeedDpp, int16_t hElAngle);

/* It exports estimated Bemf squared level*/
int32_t STO_GetEstimatedBemfLevel(STO_Handle_t *pHandle);

/* It exports observed Bemf squared level*/
int32_t STO_GetObservedBemfLevel(STO_Handle_t *pHandle);

/**
  * @}
  */

/**
  * @}
  */

/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__STO_SPEEDNPOSFDBK_H*/

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
