/**
  ******************************************************************************
  * @file    dac_common_ui.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          dac_common_ui component of the Motor Control SDK.
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
#ifndef __DACCOMMONUI_H
#define __DACCOMMONUI_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "user_interface.h"


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

/** @addtogroup dac_common_ui
  * @{
  */

#define DAC_CH_NBR 2
#define DAC_CH_USER 2



typedef struct
{
  UI_Handle_t _Super;      							/*!<   */

  MC_Protocol_REG_t bChannel_variable[DAC_CH_NBR]; /*!< Array of codes of
                                                        variables to be provided
                                                        in out to the related
                                                        channel.*/
  int16_t hUserValue[DAC_CH_USER];                 /*!< Array of user defined
                                                        DAC values.*/

  uint16_t hDAC_CH1_ENABLED;  /*!< Set to ENABLE to assign the channel 1 to the
                                   DAC object otherwise set DISABLE */
  uint16_t hDAC_CH2_ENABLED;  /*!< Set to ENABLE to assign the channel 2 to the
                                   DAC object otherwise set DISABLE */
} DAC_UI_Handle_t;


void DAC_SetChannelConfig(UI_Handle_t *pHandle, DAC_Channel_t bChannel,
                              MC_Protocol_REG_t bVariable);

MC_Protocol_REG_t DAC_GetChannelConfig(UI_Handle_t *pHandle, DAC_Channel_t bChannel);

void DAC_SetUserChannelValue(UI_Handle_t *pHandle, uint8_t bUserChNumber,
                              int16_t hValue);

int16_t DAC_GetUserChannelValue(UI_Handle_t *pHandle, uint8_t bUserChNumber);

/** @} */
/** @} */
/** @} */
/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __DACCOMMONUI_H */

 /************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
