/**
  ******************************************************************************
  * @file    lcd_manager_ui.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the  LCD
  *          Manager User Interface component.
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

/* Includes ------------------------------------------------------------------*/
#include "user_interface.h"
#include "lcd_manager_ui.h"
#include "lcd_exported_functions.h"
#include "mc_type.h"


static void* const* g_ImportedFunctions = MC_NULL;


/** @addtogroup MCSDK
  * @{
  */

/**
 * @addtogroup UILib
 * @{
 */

/**
 * @addtogroup MCUI
 * @{
 */

/**
 * @defgroup LCD_Manager_UserInterface LCD Manager User Interface
 *
 * @brief LCD Manager User Interface
 *
 * @todo Complete Documentation.
 * @{
 */

/**
  * @brief  Initialization of LCD object. It must be called after the UI_Init.
  * @param  pHandle pointer on the target component handle. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @param  pDAC related DAC object upcasted to CUI. It can be MC_NULL.
  * @param  s_fwVer String contating firmware version.
  * @retval none.
  */
void LCD_Init(UI_Handle_t *pHandle, UI_Handle_t *pDAC, const char* s_fwVer)
{
  pLCDI_Init_t pLCDI_Init = (pLCDI_Init_t)(g_ImportedFunctions[EF_LCDI_Init]);
  (*pLCDI_Init)(pHandle, pDAC, s_fwVer);
}

/**
  * @brief  Execute the LCD execution and refreshing. It must be called
  *         periodically.
  * @param  pHandle related object of class CUI. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @retval none.
  */
void LCD_Exec(UI_Handle_t *pHandle, UI_Handle_t *pDAC)
{
  pLCDI_Polling_t pLCDI_Polling = (pLCDI_Polling_t)(g_ImportedFunctions[EF_LCDI_Polling]);
  (*pLCDI_Polling)();
}

/**
  * @brief  It is used to force a refresh of all LCD values.
  * @param  pHandle related object of class CUI. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @retval none.
  */
void LCD_UpdateAll(UI_Handle_t *pHandle)
{
  pLCDI_UpdateAll_t pLCDI_UpdateAll = (pLCDI_UpdateAll_t)(g_ImportedFunctions[EF_LCDI_UpdateAll]);
  (*pLCDI_UpdateAll)(pHandle);
}

/**
  * @brief  It is used to force a refresh of only measured LCD values.
  * @param  pHandle pointer on the target component handle. It must be a LCDx_UI object casted
  *         to CUI otherwise the method will have no effect.
  * @retval none.
  */
void LCD_UpdateMeasured(UI_Handle_t *pHandle)
{
  pLCDI_UpdateMeasured_t pLCDI_UpdateMeasured = (pLCDI_UpdateMeasured_t)(g_ImportedFunctions[EF_LCDI_UpdateMeasured]);
  (*pLCDI_UpdateMeasured)(pHandle);
}

/**
  * @brief  It is used to store the LCDI Imported functions.
  * @param  void** List of imported functions.
  * @retval none.
  */
void LCD_SetLCDIImportedFunctions(void* const* ImportedFunctions)
{
  g_ImportedFunctions = ImportedFunctions;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
