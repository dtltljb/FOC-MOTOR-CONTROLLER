 /**
  ******************************************************************************
  * @file    max_torque_per_ampere.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Maximum torque per ampere (MTPA) control for I-PMSM motors
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
#ifndef __MAXTORQUEPERAMPERE_H
#define __MAXTORQUEPERAMPERE_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MTPA Maximum Torque Per Ampere Control
  * @{
  */

/* Exported types ------------------------------------------------------------*/
#define SEGMENT_NUM         ((uint8_t)7)          /* coeff no. -1 */
#define MTPA_ARRAY_SIZE     SEGMENT_NUM+1
 /**
  * @brief  Handle structure of max_torque_per_ampere.c
  */
typedef struct
{
  int16_t  SegDiv;               /**< Segments divisor */
  int32_t  AngCoeff[MTPA_ARRAY_SIZE];          /**< Angular coefficients table */
  int32_t  Offset[MTPA_ARRAY_SIZE];            /**< Offsets table */
} MTPA_Handle_t;

/* Exported functions ------------------------------------------------------- */

/*  Function used to set configure an instance of the CCC Component *****/
Curr_Components MTPA_CalcCurrRefFromIq(MTPA_Handle_t *pHandle, Curr_Components Iqdref);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __MAXTORQUEPERAMPERE_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
