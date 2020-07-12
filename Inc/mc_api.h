/**
  ******************************************************************************
  * @file    mc_api.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file defines the high level interface of the Motor Control SDK.
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
#ifndef __MC_API_H
#define __MC_API_H

#include "mc_type.h"
#include "mc_interface.h"
#include "state_machine.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCIAPI
  * @{
  */

typedef enum {UDRC_STATE_IDLE, UDRC_STATE_REQUESTED, UDRC_STATE_EOC} UDRC_State_t;

/* Starts Motor 1 */
bool MC_StartMotor1(void);

/* Stops Motor 1 */
void MC_StopMotor1(void);

/* Programs a Speed ramp for Motor 1 */
void MC_ProgramSpeedRampMotor1( int16_t hFinalSpeed, uint16_t hDurationms );

/* Programs a Torque ramp for Motor 1 */
void MC_ProgramTorqueRampMotor1( int16_t hFinalTorque, uint16_t hDurationms );

/* Programs a current reference for Motor 1 */
void MC_SetCurrentReferenceMotor1( Curr_Components Iqdref );

/* Returns the state of the last submited command for Motor 1 */
MCI_CommandState_t  MC_GetCommandStateMotor1( void);

/* Stops the execution of the current speed ramp for Motor 1 if any */
bool MC_StopSpeedRampMotor1(void);

/* Returns true if the last submited ramp for Motor 1 has completed, false otherwise */
bool MC_HasRampCompletedMotor1(void);

/* Returns the current mechanical rotor speed reference set for Motor 1, expressed in dHz (tenth of Hertz) */
int16_t MC_GetMecSpeedReferenceMotor1(void);

/* Returns the last computed average mechanical rotor speed for Motor 1, expressed in dHz (tenth of Hertz) */
int16_t MC_GetMecSpeedAverageMotor1(void);

/* Returns the final speed of the last ramp programmed for Motor 1, if this ramp was a speed ramp */
int16_t MC_GetLastRampFinalSpeedMotor1(void);

/* Returns the current Control Mode for Motor 1 (either Speed or Torque) */
STC_Modality_t MC_GetControlModeMotor1(void);

/* Returns the direction imposed by the last command on Motor 1 */
int16_t MC_GetImposedDirectionMotor1(void);

/* Returns the current reliability of the speed sensor used for Motor 1 */
bool MC_GetSpeedSensorReliabilityMotor1(void);

/* returns the amplitude of the phase current injected in Motor 1 */
int16_t MC_GetPhaseCurrentAmplitudeMotor1(void);

/* returns the amplitude of the phase voltage applied to Motor 1 */
int16_t MC_GetPhaseVoltageAmplitudeMotor1(void);

/* returns current Ia and Ib values for Motor 1 */
Curr_Components MC_GetIabMotor1(void);

/* returns current Ialpha and Ibeta values for Motor 1 */
Curr_Components MC_GetIalphabetaMotor1(void);

/* returns current Iq and Id values for Motor 1 */
Curr_Components MC_GetIqdMotor1(void);

/* returns Iq and Id reference values for Motor 1 */
Curr_Components MC_GetIqdrefMotor1(void);

/* returns current Vq and Vd values for Motor 1 */
Volt_Components MC_GetVqdMotor1(void);

/* returns current Valpha and Vbeta values for Motor 1 */
Volt_Components MC_GetValphabetaMotor1(void);

/* returns the electrical angle of the rotor of Motor 1, in DDP format */
int16_t MC_GetElAngledppMotor1(void);

/* returns the current electrical torque reference for Motor 1 */
int16_t MC_GetTerefMotor1(void);

/* Sets the reference value for Id */
void MC_SetIdrefMotor1( int16_t hNewIdref );

/* re-initializes Iq and Id references to their default values */
void MC_Clear_IqdrefMotor1(void);

/* Acknowledge a Motor Control fault on Motor 1 */
bool MC_AcknowledgeFaultMotor1( void );

/* Returns a bitfiled showing faults that occured since the State Machine of Motor 1 was moved to FAULT_NOW state */
uint16_t MC_GetOccurredFaultsMotor1(void);

/* Returns a bitfield showing all current faults on Motor 1 */
uint16_t MC_GetCurrentFaultsMotor1(void);

/* returns the current state of Motor 1 state machine */
State_t  MCI_GetSTMStateMotor1(void);

/* programs a user defined ADC regular conversion */
void MC_ProgramRegularConversion(uint8_t bChannel, uint8_t bSampleTime);

/* Returns the value of the last executed user defined ADC regular conversion */
uint16_t MC_GetRegularConversionValue(void);

/* Returns the status of the last requested user defined ADC regular conversion */
UDRC_State_t MC_GetRegularConversionState(void);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __MC_API_H */
/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
