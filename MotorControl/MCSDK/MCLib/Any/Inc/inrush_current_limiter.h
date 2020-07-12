/**
  ******************************************************************************
  * @file    inrush_current_limiter.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the 
  *          InrushCurrentLimiter component featuring the Motor Control SDK
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
#ifndef __INRUSHCURRENTLIMITER_H
#define __INRUSHCURRENTLIMITER_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "bus_voltage_sensor.h"
#include "digital_output.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup ICL
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/*
  * @brief ICL_State_t defines all the existing ICL states of the state machine
  */
typedef enum
{
  ICL_IDLE,             /* stable state */
  ICL_ACTIVATION,       /* transition state */
  ICL_ACTIVE,           /* stable state */
  ICL_DEACTIVATION,     /* transition state */
  ICL_INACTIVE          /* stable state */
} ICL_State_t;


/** 
  * @brief  ICL_Handle_t is used to handle an instance of the InrushCurrentLimiter component  
  */
typedef struct
{
  BusVoltageSensor_Handle_t *pVBS;  /*!< CVBS object used for the automatic ICL component activation/deactivation */
  DOUT_handle_t *pDOUT;                  /*!< DOUT object used to physically activate/deactivate the ICL component */

  ICL_State_t ICLstate;         /*!< Current state of the ICL state machine */
  uint16_t hICLTicksCounter;    /*!< Number of clock events remaining to complete the ICL activation/deactivation */
  uint16_t hICLTotalTicks;      /*!< Total number of clock events to complete the ICL activation/deactivation */
  uint16_t hICLFrequencyHz;     /*!< Clock frequency used (Hz) to trigger the ICL_Exec() method */
  uint16_t hICLDurationms;      /*!< ICL activation/deactivation duration (ms)*/
} ICL_Handle_t;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void ICL_Init(ICL_Handle_t *pHandle, BusVoltageSensor_Handle_t *pVBS, DOUT_handle_t *pDOUT);
ICL_State_t ICL_Exec(ICL_Handle_t *pHandle);
ICL_State_t ICL_GetState(ICL_Handle_t *pHandle);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __INRUSHCURRENTLIMITER_H */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
