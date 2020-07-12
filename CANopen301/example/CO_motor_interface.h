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
#ifndef __CO_MOTOR_INTERFACE_H
#define __CO_MOTOR_INTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "mc_extended_api.h"
#include "user_interface.h"

#include "CANopen.h"

/** @addtogroup MCSDK
  * @{
  */

/** 
 * @addtogroup UILib
 */

/** @addtogroup MCUI
  * @{
  */

/* Exported types ------------------------------------------------------------*/


/* Exported defines ----------------------------------------------------------*/

/* stardand device parameters,store to ram */
#define				CO_Index_Motor_STATUS		0x200D
#define				CO_Index_BUS_VOLTAGE		0x200E
#define				CO_Index_HEATS_TEMP			0x200F
#define				CO_Index_MOTOR_POWER		0x2010

/* motor driver parameters,	store to flash  */
#define				CO_Index_SPEED_REF				0x2300
#define				CO_Index_SPEED_KP					0x2301
#define				CO_Index_SPEED_KP_DIV			0x2302
#define				CO_Index_SPEED_KI					0x2303
#define				CO_Index_SPEED_KI_DIV			0x2304
#define				CO_Index_SPEED_KD					0x2305
#define				CO_Index_MAX_APP_SPEED		0x2306
#define				CO_Index_MIN_APP_SPEED		0x2307

#define				CO_Index_TORQUE_REF			0x2308
#define				CO_Index_TORQUE_KP			0x2309
#define				CO_Index_TORQUE_KI			0x230A
#define				CO_Index_TORQUE_KD			0x230B

/* stardand device parameters,store to ram */
#define				CO_Index_CMD_START_STOP		0x6040
#define				CO_Index_Fault_FLAGS			0x6041
#define				CO_Index_SPEED_MEAS				0x6042
#define				CO_Index_TORQUE_MEAS			0x6043
#define				CO_Index_MEAS_EL_ANGLE		0x6044
#define				CO_Index_MEAS_ROT_SPEED		0x6045

/* motor parameters ,store to flash */

#define				CO_Index_HALL_PHASE_SHIFT_N		0x6050	
#define				CO_Index_HALL_PHASE_SHIFT_P		0x6051

#define				CO_Index_POLE_PAIR_NUM				0x6052
#define				CO_Index_PHASE_RS							0x6053
#define				CO_Index_PHASE_LS							0x6054

/* motor control ,store to ram */
#define				CO_Index_CMD_START_MOTOR			0x6046
#define				CO_Index_CMD_STOP_MOTOR				0x6047
#define				CO_Index_CMD_STOP_RAMP				0x6048
#define				CO_Index_CMD_FAULT_ACK				0x6049
#define				CO_Index_CMD_ENCODER_ALIGN		0x604A
#define				CO_Index_CMD_IQDREF_CLEAR			0x604B
#define				CO_Index_CONTROL_MODE					0x6060
#define				CO_Index_RAMP_FINAL_SPEED			0x6099






/**
  * @brief  Initialization of UI object. It perform the link between the UI
  *         object and the MC interface and MC tuning objects. It must be called
  *         before the derived class initialization.
  */
void MI_Init(UI_Handle_t *pHandle, uint8_t bMCNum, MCI_Handle_t** pMCI, MCT_Handle_t** pMCT, uint32_t* pUICfg);

/**
  * @brief  It is used to select the MC on which UI operates.
  */
bool MI_SelectMC(UI_Handle_t *pHandle,uint8_t bSelectMC);


/**
  * @brief  It is used to retrieve the current selected MC tuning object.
  * @param  pHandle pointer on the target component handle.
  * @retval MCT_Handle_t motor control tuning handler on which UI operates.
  */
MCT_Handle_t* MI_GetCurrentMCT(UI_Handle_t *pHandle);

/**
  * @brief  It is used to retrive the configuration of the MC on which UI
  *         currently operates.
  * @param  pHandle pointer on the target component handle.
  * @retval uint32_t It returns the currently configuration of selected MC on
  *         which UI operates.
  *         It represents a bit field containing one (or more) of
  *         the exported configuration option UI_CFGOPT_xxx (eventually OR-ed).
  */
uint32_t MI_GetSelectedMCConfig(UI_Handle_t *pHandle);

/**
  * @brief  It is used to execute a SetReg command coming from the user.
  * @param  pHandle pointer on the target component handle.
  * @param  pSDO: CO_SDO_PROCESS update.
  *         
  * @param  
  * @retval bool It returns true if the SetReg command has been performed
  *         succesfully otherwise returns false.
  */
bool MI_SetReg(UI_Handle_t *pHandle,  CO_SDO_t 	*pSDO);

/**
  * @brief  It is used to execute a GetReg command coming from the user.
  * @param  pHandle pointer on the target component handle.
  * @param  bRegID Code of register to be updated. Valid code is one of the
  *         MC_PROTOCOL_REG_xxx values exported by UserInterfaceClass.
  * @retval int32_t is the current value of register bRegID.
  */
int32_t MI_GetReg(UI_Handle_t *pHandle, CO_SDO_t 	*pSDO);




#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__USERINTERFACE_H*/

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
