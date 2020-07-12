/**
  ******************************************************************************
  * @file    user_interface.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          user interface component of the Motor Control SDK.
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
#ifndef __UART1_USERINTERFACE_H
#define __UART1_USERINTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "mc_extended_api.h"

/** @addtogroup MCSDK
  * @{
  */

/** 
 * @addtogroup UILib
 */

/** @addtogroup MCUI
  * @{
  */
//
///* Exported types ------------------------------------------------------------*/
///* @brief To configure the UI use MAIN Sensor (4bit)|AUX Sensor (4 bit) as first byte of CFG.*/
//#define UI_SCODE_NONE     0x0u
//#define UI_SCODE_HALL     0x1u /*!< This code identifies the Hall sensor.*/
//#define UI_SCODE_ENC      0x2u /*!< This code identifies the Encoder sensor.*/
//#define UI_SCODE_STO_PLL  0x9u /*!< This code identifies the State observer + PLL sensor.*/
//#define UI_SCODE_STO_CR   0xAu /*!< This code identifies the State observer + CORDIC sensor.*/
//#define UI_SCODE_HFINJ    0xBu /*!< This code identifies the HF injection sensor.*/
//
//#define UI_CFGOPT_NONE            0x00000000u /*!< Enable this option when no other
//                                              option is selected.*/
//#define UI_CFGOPT_FW              0x00000001u /*!< Enable this option when the flux
//                                              weakening is enabled in the MC
//                                              firmware.*/
//#define UI_CFGOPT_SPEED_KD        0x00000002u /*!< Enable this option when the speed
//                                              controller has derivative action.
//                                              */
//#define UI_CFGOPT_Iq_KD           0x00000004u /*!< Enable this option when the Iq
//                                              controller has derivative action.
//                                              */
//#define UI_CFGOPT_Id_KD           0x00000008u /*!< Enable this option when the Id
//                                              controller has derivative action.
//                                              */
//#define UI_CFGOPT_DAC             0x00000010u /*!< Enable this option if a DAC object
//                                              will be associated with the UI.*/
//#define UI_CFGOPT_SETIDINSPDMODE  0x00000020u /*!< Enable this option to allow setting
//                                              the Id reference when MC is in
//                                              speed mode.*/
//#define UI_CFGOPT_PLLTUNING       0x00000040u /*!< Enable this option to allow setting
//                                              the PLL KP and KI.*/
//#define UI_CFGOPT_PFC             0x00000080u /*!< Enable this option to allow PFC tuning.*/
//
//#define UI_CFGOPT_PFC_I_KD        0x00000100u /*!< Enable this option when PFC current
//                                              controller has derivative action.*/
//#define UI_CFGOPT_PFC_V_KD        0x00000200u /*!< Enable this option when PFC voltage
//                                              controller has derivative action.*/


/* FCLP_Frame_t->code ==> command code number */
#define MB_MC_PROTOCOL_CODE_OUT_CMD    	0X02		//OUTPUT control
#define MB_MC_PROTOCOL_CODE_GET_CMD    	0x03		//read single		register
#define MB_MC_PROTOCOL_CODE_SET_CMD    	0x10		//write	mult 		register	

/*	share user_interface.h mcro define 
#define MC_PROTOCOL_CODE_SET_REG        0x01		//externded reg CmdID place in mc_extended_api.h
#define MC_PROTOCOL_CODE_GET_REG        0x02
#define MC_PROTOCOL_CODE_EXECUTE_CMD    0x03

#define MC_PROTOCOL_CODE_STORE_TOADDR   0x04
#define MC_PROTOCOL_CODE_LOAD_FROMADDR  0x05
#define MC_PROTOCOL_CODE_GET_BOARD_INFO 0x06

#define MC_PROTOCOL_CODE_SET_RAMP       0x07
#define MC_PROTOCOL_CODE_GET_REVUP_DATA 0x08
#define MC_PROTOCOL_CODE_SET_REVUP_DATA 0x09
#define MC_PROTOCOL_CODE_SET_CURRENT_REF 0x0A
#define MC_PROTOCOL_CODE_GET_MP_INFO    0x0B
*/

/* 
*	MC_PROTOCOL_CODE_EXECUTE_CMD subversion control command code
*	FCLP_Frame_t->Buffer[0] ==> CmdID
 */
 
 /*		share user_interface.h mcro define 
#define MC_PROTOCOL_CMD_START_MOTOR   0x01
#define MC_PROTOCOL_CMD_STOP_MOTOR    0x02
#define MC_PROTOCOL_CMD_STOP_RAMP     0x03
#define MC_PROTOCOL_CMD_RESET         0x04
#define MC_PROTOCOL_CMD_PING          0x05
#define MC_PROTOCOL_CMD_START_STOP    0x06
#define MC_PROTOCOL_CMD_FAULT_ACK     0x07
#define MC_PROTOCOL_CMD_ENCODER_ALIGN 0x08
#define MC_PROTOCOL_CMD_IQDREF_CLEAR  0x09
#define MC_PROTOCOL_CMD_PFC_ENABLE    0x0A
#define MC_PROTOCOL_CMD_PFC_DISABLE   0x0B
#define MC_PROTOCOL_CMD_PFC_FAULT_ACK 0x0C
#define MC_PROTOCOL_CMD_SC_START      0x0D
#define MC_PROTOCOL_CMD_SC_STOP       0x0E
#define GUI_ERROR_CODE 0xFFFFFFFF
*/
/**
 * @brief Enumerates the variables and information that can be exchanged across the Motor Control Protocol
 *	 READ NON RESPONSE
 */
 
typedef enum {
	MC_PROTOCOL_CMD_CONTROL_MOTOR	=	130,		//MC_PROTOCOL_REG_UNDEFINED
	MC_PROTOCOL_CMD_CLEAR_FAULT_ACK	,
	MC_PROTOCOL_CMD_CLEAR_IQDREF,
	MC_PROTOCOL_CMD_ALIGN_ENCODER,
	
} MC_Protocol_CMD_t;

/* Exported defines ----------------------------------------------------------*/
/*
#define LCD_LIGHT 0x01
#define LCD_FULL  0x02
*/

/**
  * @brief  UserInterface class parameters definition
  */
typedef const void UserInterfaceParams_t, *pUserInterfaceParams_t;

/**
  * @brief This structure is used to handle an instance of the UI component
  *
  */
typedef struct U1UI_Handle U1UI_Handle_t;

/**
  * @brief Polymorphic function. The function called can change in run-time and
  *        is assigned by the library to one of the callback pointers
  *        pFctSwitchOffPwm
  *        pFctSwitchOnPwm
  *        pFctCurrReadingCalib
  *        pFctTurnOnLowSides
  *        pFctRLDetectionModeEnable
  *        pFctRLDetectionModeDisable
  *
  *
  */
typedef void (*U1UI_Generic_Cb_t)( U1UI_Handle_t *pHandle);

/**
  * @brief Polymorphic function. The function called can change in run-time and
  *        is assigned by the library to the callback pointer pFctIrqHandler
  *
  */
typedef void* (*U1UI_IrqHandler_Cb_t)( void *pHandle, unsigned char flag, unsigned short rx_data);

/**
  * @brief UI_Handle structure used for User Interface
  *
  */
struct U1UI_Handle
{

  UI_IrqHandler_Cb_t pFctIrqHandler;

  uint8_t bDriveNum;      /*!< Total number of MC objects.*/
  MCI_Handle_t** pMCI;             /*!< Pointer of MC interface list.*/
  MCT_Handle_t** pMCT;             /*!< Pointer of MC tuning list.*/
  uint32_t* pUICfg;       /*!< Pointer of UI configuration list.*/
  uint8_t bSelectedDrive; /*!< Current selected MC object in the list.*/
};

/**
  * @brief  Initialization of UI object. It perform the link between the UI
  *         object and the MC interface and MC tuning objects. It must be called
  *         before the derived class initialization.
  */
void U1UI_Init(U1UI_Handle_t *pHandle, uint8_t bMCNum, MCI_Handle_t** pMCI, MCT_Handle_t** pMCT, uint32_t* pUICfg);

/**
  * @brief  It is used to select the MC on which UI operates.
  */
bool U1UI_SelectMC(U1UI_Handle_t *pHandle,uint8_t bSelectMC);

/**
  * @brief  It is used to retrive the MC on which UI currently operates.
  * @param  pHandle pointer on the target component handle.
  * @retval uint8_t It returns the currently selected MC, zero based, on which
  *         UI operates.
  */
uint8_t U1UI_GetSelectedMC(U1UI_Handle_t *pHandle);

/**
  * @brief  It is used to retrive the configuration of the MC on which UI
  *         currently operates.
  * @param  pHandle pointer on the target component handle.
  * @retval uint32_t It returns the currently configuration of selected MC on
  *         which UI operates.
  *         It represents a bit field containing one (or more) of
  *         the exported configuration option UI_CFGOPT_xxx (eventually OR-ed).
  */
uint32_t U1UI_GetSelectedMCConfig(U1UI_Handle_t *pHandle);

/**
  * @brief  It is used to execute a SetReg command coming from the user.
  * @param  pHandle pointer on the target component handle.
  * @param  bRegID Code of register to be updated. Valid code is one of the
  *         MC_PROTOCOL_REG_xxx values exported by UserInterfaceClass.
  * @param  wValue is the new value to be set.
  * @retval bool It returns true if the SetReg command has been performed
  *         succesfully otherwise returns false.
  */
bool U1UI_SetReg(U1UI_Handle_t *pHandle, MC_Protocol_REG_t bRegID, int32_t wValue);

/**
  * @brief  It is used to execute a GetReg command coming from the user.
  * @param  pHandle pointer on the target component handle.
  * @param  bRegID Code of register to be updated. Valid code is one of the
  *         MC_PROTOCOL_REG_xxx values exported by UserInterfaceClass.
  * @retval int32_t is the current value of register bRegID.
  */
int32_t U1UI_GetReg(U1UI_Handle_t *pHandle, MC_Protocol_REG_t bRegID);

/**
  * @brief  It is used to retrieve the current selected MC tuning object.
  * @param  pHandle pointer on the target component handle.
  * @retval MCT_Handle_t motor control tuning handler on which UI operates.
  */
MCT_Handle_t* U1UI_GetCurrentMCT(U1UI_Handle_t *pHandle);

/**
  * @brief  It is used to execute a command coming from the user.
  * @param  pHandle pointer on the target component handle.
  * @param  bCmdID Code of register to be updated. Valid code is one of the
  *         MC_PROTOCOL_CMD_xxx define exported by UserInterfaceClass.
  * @retval bool It returns true if the command has been performed
  *         succesfully otherwise returns false.
  */
bool U1UI_ExecCmd(U1UI_Handle_t *pHandle, uint8_t bCmdID);

/**
  * @brief  It is used to execute a speed ramp command coming from the user.
  * @param  pHandle pointer on the target component handle.
  * @param  wFinalMecSpeedRPM final speed value expressed in RPM.
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the value.
  * @retval bool It returns true if the command has been performed
  *         succesfully otherwise returns false.
  */
bool U1UI_ExecSpeedRamp(U1UI_Handle_t *pHandle, int32_t wFinalMecSpeedRPM, uint16_t hDurationms);

/**
  * @brief  It is used to execute a torque ramp command coming from the user.
  * @param  pHandle pointer on the target component handle.
  * @param  hTargetFinal final torque value. See MCI interface for more
            details.
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the value.
  * @retval bool It returns true if the command has been performed
  *         succesfully otherwise returns false.
  */
bool U1UI_ExecTorqueRamp(U1UI_Handle_t *pHandle, int16_t hTargetFinal, uint16_t hDurationms);

/**
  * @brief  It is used to execute a get Revup data command coming from the user.
  * @param  pHandle pointer on the target component handle.
  * @param  bStage is the rev up phase, zero based, to be read.
  * @param  pDurationms is the pointer to an uint16_t variable used to retrieve
  *         the duration of the Revup stage.
  * @param  pFinalMecSpeed01Hz is the pointer to an int16_t variable used to
  *         retrieve the mechanical speed at the end of that stage expressed in
  *         0.1Hz.
  * @param  pFinalTorque is the pointer to an int16_t variable used to
  *         retrieve the value of motor torque at the end of that
  *         stage. This value represents actually the Iq current expressed in
  *         digit.
  * @retval bool It returns true if the command has been performed
  *         succesfully otherwise returns false.
  */
bool U1UI_GetRevupData(U1UI_Handle_t *pHandle, uint8_t bStage, uint16_t* pDurationms,
                     int16_t* pFinalMecSpeed01Hz, int16_t* pFinalTorque );

/**
  * @brief  It is used to execute a set Revup data command coming from the user.
  * @param  pHandle pointer on the target component handle.
  * @param  bStage is the rev up phase, zero based, to be modified.
  * @param  hDurationms is the new duration of the Revup stage.
  * @param  hFinalMecSpeed01Hz is the new mechanical speed at the end of that
  *         stage expressed in 0.1Hz.
  * @param  hFinalTorque is the new value of motor torque at the end of that
  *         stage. This value represents actually the Iq current expressed in
  *         digit.
  * @retval bool It returns true if the command has been performed
  *         succesfully otherwise returns false.
  */
bool U1UI_SetRevupData(U1UI_Handle_t *pHandle, uint8_t bStage, uint16_t hDurationms,
                     int16_t hFinalMecSpeed01Hz, int16_t hFinalTorque );

/**
  * @brief  It is used to execute a set current reference command coming from
  *         the user.
  * @param  pHandle pointer on the target component handle.
  * @param  hIqRef is the current Iq reference on qd reference frame. This value
  *         is expressed in digit. To convert current expressed in digit to
  *         current expressed in Amps is possible to use the formula:
  *         Current(Amp) = [Current(digit) * Vdd micro] / [65536 * Rshunt * Aop]
  * @param  hIdRef is the current Id reference on qd reference frame. This value
  *         is expressed in digit. See hIqRef param description.
  * @retval none.
  */
void U1UI_SetCurrentReferences(U1UI_Handle_t *pHandle, int16_t hIqRef, int16_t hIdRef);

/**
  * @brief  Function to get information about MP registers available for each
  *         step. PC send to the FW the list of steps to get the available
  *         registers. The FW returs the list of available registers for that
  *         steps.
  * @param  stepList List of requested steps.
  * @param  pMPInfo The returned list of register.
  *         It is populated by this function.
  * @retval true if MP is enabled, false otherwise.
  */
bool U1UI_GetMPInfo(pMPInfo_t stepList, pMPInfo_t MPInfo);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__USERINTERFACE_H*/

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
