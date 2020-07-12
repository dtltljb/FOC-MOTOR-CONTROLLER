/**
  ******************************************************************************
  * @file    mc_tasks.h 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implementes tasks definition.
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
#ifndef __MCTASKS_H
#define __MCTASKS_H

/* Includes ------------------------------------------------------------------*/
#include "mc_tuning.h"
#include "mc_interface.h"
#include "pwm_curr_fdbk.h"
#include "hall_speed_pos_fdbk.h"
/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MCTasks Motor Control Tasks
  * 
  * @brief Motor Control subsystem configuration and operation routines.  
  *
  * @{
  */

extern HALL_Handle_t *oSpeedSensor[];
extern PWMC_Handle_t* pwmcHandle[];
  
/**
  * @brief  It initializes the whole MC core according to user defined 
  *         parameters.
  * @param  pMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the 
  *         number of motor drives.
  * @param  pMCTList pointer to the vector of MCTuning objects that will be
  *         created and initialized. The vector must have length equal to the 
  *         number of motor drives.
  * @retval None
  */
void MCboot( MCI_Handle_t* pMCIList[], MCT_Handle_t* pMCTList[] );

/**
  * @brief  It executes MC tasks: safety task and medium frequency for all
  *         drive instances. It have to be clocked with Systick frequnecy.
  * @param  None
  * @retval None
  */
void MC_Scheduler(void);

/**
  * @brief  It executes safety checks (e.g. bus voltage and temperature) for all
  *         drive instances. Faults flags are also here updated     
  * @param  None
  * @retval None
  */
void TSK_SafetyTask(void);

/**
  * @brief  Accordingly with the present state(s) of the state machine(s), it 
  *         executes those motor control duties requiring a high frequency rate
  *         and a precise timing (e.g. FOC current control loop)
  * @param  None
  * @retval uint8_t It return the motor instance number of last executed FOC.
  */
uint8_t TSK_HighFrequencyTask(void);

/**
  * @brief  This function is called by TIMx_UP_IRQHandler in case of dual MC and
  *         it allows to reserve half PWM period in advance the FOC execution on
  *         ADC ISR
  * @param  oDrive pointer to a CFOC object
  * @retval None
  */
void TSK_DualDriveFIFOUpdate(void *oDrive);

/**
  * @brief  It is executed when a general hardware failure has been detected by
  *         the microcontroller and is used to put the system in safety 
  *         condition.
  * @param  None
  * @retval None
  */
void TSK_HardwareFaultTask(void);

 /**
  * @brief  This function locks GPIO pins used for Motor Control. This prevents accidental reconfiguration 
  *
  * @param  None
  * @retval None
  */

void mc_lock_pins (void);
/**
  * @}
  */

/**
  * @}
  */

#endif /* __MCTASKS_H */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
