/**
  ******************************************************************************
  * @file    ui_exported_functions.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions of UI exported functions.
  *
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
#ifndef __UIEXPORTEDFUNCTIONS_H
#define __UIEXPORTEDFUNCTIONS_H

enum {
EF_UI_GetReg,
EF_UI_ExecSpeedRamp,
EF_UI_SetReg,
EF_UI_ExecCmd,
EF_UI_GetSelectedMCConfig,
EF_UI_SetRevupData,
EF_UI_GetRevupData,
EF_UI_DACChannelConfig,
EF_UI_SetCurrentReferences,
EF_UI_NUMBERS
};

typedef int32_t (*pUI_GetReg_t) (UI_Handle_t *pHandle,MC_Protocol_REG_t);
typedef bool (*pUI_ExecSpeedRamp_t)(UI_Handle_t *pHandle,int32_t,uint16_t);
typedef bool (*pUI_SetReg_t)(UI_Handle_t *pHandle,MC_Protocol_REG_t,int32_t);
typedef bool (*pUI_ExecCmd_t)(UI_Handle_t *pHandle,uint8_t);
typedef uint32_t (*pUI_GetSelectedMCConfig_t)(UI_Handle_t *pHandle);
typedef bool (*pUI_SetRevupData_t)(UI_Handle_t *pHandle,uint8_t,uint16_t,int16_t,int16_t);
typedef bool (*pUI_GetRevupData_t)(UI_Handle_t *pHandle,uint8_t,uint16_t*,int16_t*,int16_t*);
typedef void (*pUI_DACChannelConfig_t)(UI_Handle_t *pHandle,DAC_Channel_t,MC_Protocol_REG_t);
typedef void (*pUI_SetCurrentReferences_t)(UI_Handle_t *pHandle,int16_t,int16_t); 

#endif /*__UIEXPORTEDFUNCTIONS_H*/
/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
