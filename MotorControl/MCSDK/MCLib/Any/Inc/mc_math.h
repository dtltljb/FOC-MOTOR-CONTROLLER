/**
  ******************************************************************************
  * @file    mc_math.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides mathematics functions useful for and specific to
  *          Motor Control.
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
#ifndef __MC_MATH_H
#define __MC_MATH_H

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MC_Math
  * @{
  */

/** 
  * @brief  Macro to compute logarithm of two
  */
#define LOG2(x) \
((x) == 65535 ? 16 : \
((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2*2*2 ? 15 : \
((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2*2 ? 14 : \
((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2 ? 13 : \
((x) == 2*2*2*2*2*2*2*2*2*2*2*2 ? 12 : \
((x) == 2*2*2*2*2*2*2*2*2*2*2 ? 11 : \
((x) == 2*2*2*2*2*2*2*2*2*2 ? 10 : \
((x) == 2*2*2*2*2*2*2*2*2 ? 9 : \
((x) == 2*2*2*2*2*2*2*2 ? 8 : \
((x) == 2*2*2*2*2*2*2 ? 7 : \
((x) == 2*2*2*2*2*2 ? 6 : \
((x) == 2*2*2*2*2 ? 5 : \
((x) == 2*2*2*2 ? 4 : \
((x) == 2*2*2 ? 3 : \
((x) == 2*2 ? 2 : \
((x) == 2 ? 1 : \
((x) == 1 ? 0 : -1)))))))))))))))))

/** 
  * @brief  Trigonometrical functions type definition 
  */
  typedef struct
  {
    int16_t hCos;
    int16_t hSin;
  } Trig_Components;

/**
  * @brief  This function transforms stator currents Ia and qIb (which are 
  *         directed along axes each displaced by 120 degrees) into currents 
  *         Ialpha and Ibeta in a stationary qd reference frame.
  *                               Ialpha = Ia
  *                       Ibeta = -(2*Ib+Ia)/sqrt(3)
  * @param  Curr_Input: stator current Ia and Ib in Curr_Components format
  * @retval Stator current Ialpha and Ibeta in Curr_Components format
  */
Curr_Components MCM_Clarke(Curr_Components Curr_Input);

/**
  * @brief  This function transforms stator currents Ialpha and Ibeta, which 
  *         belong to a stationary qd reference frame, to a rotor flux 
  *         synchronous reference frame (properly oriented), so as Iq and Id.
  *                   Id= Ialpha *sin(theta)+qIbeta *cos(Theta)
  *                   Iq=qIalpha *cos(Theta)-qIbeta *sin(Theta)            
  * @param  Curr_Input: stator current Ialpha and Ibeta in Curr_Components format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Stator current Iq and Id in Curr_Components format
  */
Curr_Components MCM_Park(Curr_Components Curr_Input, int16_t Theta);

/**
  * @brief  This function transforms stator voltage qVq and qVd, that belong to 
  *         a rotor flux synchronous rotating frame, to a stationary reference 
  *         frame, so as to obtain qValpha and qVbeta:
  *                  Valfa= Vq*Cos(theta)+ Vd*Sin(theta)
  *                  Vbeta=-Vq*Sin(theta)+ Vd*Cos(theta)     
  * @param  Curr_Input: stator voltage Vq and Vd in Volt_Components format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Stator voltage Valpha and Vbeta in Volt_Components format
  */
Volt_Components MCM_Rev_Park(Volt_Components Volt_Input, int16_t Theta);


/**
  * @brief  This function returns cosine and sine functions of the angle fed in 
  *         input
  * @param  hAngle: angle in q1.15 format
  * @retval Trig_Components Cos(angle) and Sin(angle) in Trig_Components format
  */
Trig_Components MCM_Trig_Functions(int16_t hAngle);

/**
  * @brief  It calculates the square root of a non-negative s32. It returns 0 
  *         for negative s32.
  * @param  Input int32_t number
  * @retval int32_t Square root of Input (0 if Input<0)
  */
int32_t MCM_Sqrt(int32_t wInput);

/**
  * @brief  This function codify a floting point number into the relative
  *         32bit integer.
  * @param  float Floting point number to be coded.
  * @retval uint32_t Coded 32bit integer.
  */
uint32_t MCM_floatToIntBit(float x);

/**
  * @}
  */

/**
  * @}
  */
#endif /*__MC_MATH_H*/
/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
