/**
  ******************************************************************************
  * @file    sto_cordic_speed_pos_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          State Observer + CORDIC Speed & Position Feedback component of the
  *          Motor Control SDK.
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
#ifndef __STO_CORDIC_SPEEDNPOSFDBK_H
#define __STO_CORDIC_SPEEDNPOSFDBK_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
   
/* Includes ------------------------------------------------------------------*/   
#include "speed_pos_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */
  
/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @addtogroup STO_CORDIC_SpeednPosFdbk
  * @{
  */

/** 
  * @brief  This structure is used to handle an instance of the STO_CORDIC component
  */
typedef struct
{
   SpeednPosFdbk_Handle_t _Super;
   int16_t  hC1;                      /*!< State observer constant C1, it can 
                                           be computed as F1 * Rs(ohm)/(Ls[H] * 
                                           State observer execution rate [Hz])*/
   int16_t  hC2;                     /*!<  Variable containing state observer 
                                           constant C2, it can be computed as 
                                           F1 * K1/ State observer execution 
                                           rate [Hz] being K1 one of the two 
                                           observer gains   */   
   int16_t  hC3;                      /*!< State observer constant C3, it can 
                                           be computed as F1 * Max application 
                                           speed [rpm] * Motor B-emf constant 
                                           [Vllrms/krpm] * sqrt(2)/ (Ls [H] *
                                           max measurable current (A) * State 
                                           observer execution rate [Hz])*/ 
   int16_t  hC4;                      /*!< State Observer constant C4, it can 
                                           be computed as K2 * max measurable 
                                           current (A) / (Max application speed 
                                           [rpm] * Motor B-emf constant 
                                           [Vllrms/krpm] * sqrt(2) * F2 * State 
                                           observer execution rate [Hz]) being 
                                           K2 one of the two observer gains  */
   int16_t  hC5;                      /*!< State Observer constant C5, it can 
                                           be computed as F1 * max measurable 
                                           voltage / (2 * Ls [Hz] * max 
                                           measurable current * State observer 
                                           execution rate [Hz]) */
   int16_t  hC6;                      /*!< State observer constant C6, computed with a 
                                           specific procedure starting from the other 
                                           constants */ 
   int16_t  hF1;                      /*!< State observer scaling factor F1 */
   int16_t  hF2;                      /*!< State observer scaling factor F2 */
   int16_t  hF3;                      /*!< State observer scaling factor F3 */
   uint16_t F3POW2;                  /*!< State observer scaling factor F3 expressed as power of 2.
                                     E.g. if gain divisor is 512 the value 
                                     must be 9 because 2^9 = 512 */
   int32_t Ialfa_est;           /*!< Estimated Ialfa current in int32 format */
   int32_t Ibeta_est;           /*!< Estimated Ialfa current in int32 format */
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
   uint8_t  ReliabilityCounter; /*!< Counter for reliability check internal to 
                                     derived class */
   bool IsAlgorithmConverged;   /*!< Boolean variable containing observer 
                                     convergence information */
   int16_t Orig_Speed_Buffer[64];    /*!< Estimated speed FIFO, it contains latest 
                                     bSpeedBufferSize speed measurements, cordic
                                     outputs not modified*/
   int16_t Orig_ElSpeedDpp;
   bool IsBemfConsistent;       /*!< Sensor reliability information, updated by
                                     SPD_CalcAvrgMecSpeed01Hz, it is true if the
                                     observed back-emfs are consistent with
                                     expectation*/
   int32_t Obs_Bemf_Level;      /*!< Observed back-emf Level*/
   int32_t Est_Bemf_Level;      /*!< Estimated back-emf Level*/
   bool EnableDualCheck;        /*!< Consistency check enabler*/
   int32_t DppBufferSum;        /*!< summation of speed buffer elements [dpp]*/
   int32_t DppOrigBufferSum;    /*!< summation of not modified speed buffer elements [dpp]*/
   int16_t SpeedBufferOldestEl; /*!< Oldest element of the speed buffer*/
   int16_t OrigSpeedBufferOldestEl; /*!< Oldest element of the not modified speed buffer*/  
  
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
   uint16_t MinStartUpValidSpeed;     /*!< Minimum mechanical speed (expressed in 
                                          01Hz required to validate the start-up
                                          */
   uint8_t StartUpConsistThreshold;   /*!< Number of consecutive tests on speed 
                                           consistency to be passed before 
                                           validating the start-up */
   uint8_t Reliability_hysteresys;    /*!< Number of reliability failed 
                                           consecutive tests before a speed 
                                           check fault is returned to base class 
                                           */
   int16_t MaxInstantElAcceleration;   /*!< maximum instantaneous electrical
                                        acceleration (dpp per control period) */
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
}STO_CR_Handle_t;

/* Exported functions ------------------------------------------------------- */

/* It only returns, necessary to implement fictitious IRQ_Handler */
void STO_CR_Return(STO_CR_Handle_t* pHandle, uint8_t flag);

/* It initializes the state observer object */
void STO_CR_Init(STO_CR_Handle_t* pHandle);

/* It clears state observer object by re-initializing private variables*/
void STO_CR_Clear(STO_CR_Handle_t* pHandle);

/* It executes Luenberger state observer and calls CORDIC to compute a new speed 
*  estimation and update the estimated electrical angle. 
*/
int16_t STO_CR_CalcElAngle(STO_CR_Handle_t* pHandle, Observer_Inputs_t *pInputVars_str);

/*It computes through pMecSpeed01Hz the rotor average mechanical speed in 01Hz*/
bool STO_CR_CalcAvrgMecSpeed01Hz(STO_CR_Handle_t* pHandle, int16_t *pMecSpeed01Hz);

/* It checks whether the state observer algorithm converged.*/
bool STO_CR_IsObserverConverged(STO_CR_Handle_t* pHandle, int16_t hForcedMecSpeed01Hz);

/* It exports estimated Bemf alpha-beta in Volt_Components format */
Volt_Components STO_CR_GetEstimatedBemf(STO_CR_Handle_t* pHandle);

/* It exports the stator current alpha-beta as estimated by state observer */  
Curr_Components STO_CR_GetEstimatedCurrent(STO_CR_Handle_t* pHandle);

/* It exports current observer gains through parameters hC2 and hC4 */
void STO_CR_GetObserverGains(STO_CR_Handle_t* pHandle, int16_t *pC2, int16_t *pC4);

/* It allows setting new values for observer gains hC1 and hC2 */
void STO_CR_SetObserverGains(STO_CR_Handle_t* pHandle, int16_t hC1, int16_t hC2);

/* It computes and update the estimated average electrical speed  */
void STO_CR_CalcAvrgElSpeedDpp(STO_CR_Handle_t* pHandle);

/* It exports estimated Bemf squared level */
int32_t STO_CR_GetEstimatedBemfLevel(STO_CR_Handle_t* pHandle);

/* It exports observed Bemf squared level */
int32_t STO_CR_GetObservedBemfLevel(STO_CR_Handle_t* pHandle);

/* It enables/disables the bemf consistency check */
void STO_CR_BemfConsistencyCheckSwitch(STO_CR_Handle_t* pHandle, bool bSel);

/* It returns the result of the Bemf consistency check */
bool STO_CR_IsBemfConsistent(STO_CR_Handle_t* pHandle);

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

#endif /*__STO_CORDIC_SPEEDNPOSFDBK_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
